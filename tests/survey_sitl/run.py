"""Replay and record a survey scene in an explicitly selected SITL release."""
import argparse
from contextlib import ExitStack, contextmanager
from functools import partial
import json
from pathlib import Path
from shutil import which
import subprocess
import time
from urllib.request import urlopen
from uuid import uuid4

HERE = Path(__file__).resolve().parent


def run(args, **kwargs):
    return subprocess.run(args, check=True, **kwargs)


def status(url):
    deadline = time.monotonic() + 15
    while True:
        try:
            with urlopen(url + "/status", timeout=4) as response:
                state = json.load(response)
            if state["age_seconds"].get("mission", 999) <= 2:
                return {"age_seconds": state["age_seconds"], "mission": state["mission"]}
            failure = RuntimeError("Mission dashboard status is stale")
        except OSError as exc:
            failure = exc
        if time.monotonic() >= deadline:
            raise RuntimeError("No fresh mission dashboard status") from failure
        time.sleep(.25)


@contextmanager
def recorder(command, stop):
    process = subprocess.Popen(command)
    try:
        yield process
    finally:
        try:
            stop()
            if process.wait(timeout=60) != 0:
                raise RuntimeError("An evidence recorder exited unsuccessfully")
        finally:
            if process.poll() is None:
                process.kill()
                process.wait()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scene", choices=("early", "late", "empty"))
    parser.add_argument("--namespace", required=True)
    parser.add_argument("--release", required=True)
    parser.add_argument("--ground", choices=("grass-gravel", "plain"), default="grass-gravel",
                        help="Choose the visual surface while preserving the collision floor")
    parser.add_argument("--dashboard", help="Dashboard Route URL (discovered from the release by default)")
    parser.add_argument("--chrome-debug-url", required=True)
    parser.add_argument("--record-dashboard-video", action="store_true",
                        help="Record the survey dashboard through touchdown (requires local ffmpeg)")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--confirm-sitl", action="store_true", required=True)
    args = parser.parse_args()
    if which("node") is None or (args.record_dashboard_video and which("ffmpeg") is None):
        parser.error("Recording requires Node.js and video capture also requires ffmpeg")
    out = args.output
    out.mkdir(parents=True)
    oc = ["oc", "-n", args.namespace]
    mission = oc + ["exec", f"deploy/{args.release}-mission", "-c", "mission", "--"]
    gazebo = oc + ["exec", f"deploy/{args.release}-simulator", "-c", "px4-gazebo", "--"]
    entry = ["/opt/mission/entrypoint.sh", "python3", "-u"]
    route_host = run(oc + ["get", "route", args.release + "-dashboard", "-o", "jsonpath={.spec.host}"],
                     capture_output=True, text=True).stdout.strip()
    if not route_host:
        parser.error("The selected release has no dashboard Route host")
    dashboard_url = "https://" + route_host
    if args.dashboard and args.dashboard.rstrip("/") != dashboard_url:
        parser.error("--dashboard must match the dashboard Route of the selected release")
    current = status(dashboard_url)
    state = current["mission"]
    if not (state["landed"] and not state["armed"] and state["telemetry_fresh"]
            and state["phase"] == "ready" and "survey_mode_id" in state):
        raise RuntimeError("Require fresh, landed, disarmed SignSurvey telemetry at the start")
    reported_namespace = run(mission + ["printenv", "SITL_NAMESPACE"], capture_output=True, text=True)
    if reported_namespace.stdout.strip() != args.namespace:
        raise RuntimeError("Mission container namespace mismatch")
    (out / "initial.json").write_text(json.dumps(current, indent=2) + "\n")

    prefix = f"/tmp/survey-{args.scene}-{uuid4().hex[:8]}"

    def stage(target, source):
        path = prefix + "-" + source.name
        with source.open("rb") as stream:
            run(target[:4] + ["-i"] + target[4:] + ["tee", path],
                stdin=stream, stdout=subprocess.DEVNULL)
        return path

    fixture = stage(gazebo, HERE / "fixture.py")
    with (out / "fixture.json").open("w") as stream:
        run(gazebo + ["python3", fixture, args.scene, "--ground", args.ground], stdout=stream)
    replay = stage(mission, HERE.parent / "replay_sitl.py")
    recorder_script = stage(mission, HERE / "record.py")
    truth = stage(gazebo, HERE / "record_gazebo.py")
    telemetry_path = prefix + "-telemetry.jsonl"
    truth_path = prefix + "-truth.csv"
    browser_folder = out / "dashboard"
    mission_start = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())

    browser_folder.mkdir()
    def collect(command, filename):
        with (out / filename).open("w") as stream:
            run(command, stdout=stream, timeout=60)

    recordings = [
        (mission, entry + [recorder_script, telemetry_path, "--namespace", args.namespace], telemetry_path, "telemetry.jsonl"),
        (gazebo, ["python3", truth, truth_path], truth_path, "gazebo.csv"),
    ]
    # Stop every recorder before collecting its final output, including on failed replays.
    with ExitStack() as artifacts, ExitStack() as stack:
        artifacts.callback(collect, oc + ["logs", f"deploy/{args.release}-mission", "-c", "mission",
                                          "--since-time", mission_start], "mission.log")
        processes = []
        for target, args_remote, remote, filename in recordings:
            stop_cmd = target + ["touch", str(Path(remote).with_suffix(".stop"))]
            stop = partial(run, stop_cmd, timeout=15)
            processes.append(stack.enter_context(recorder(target + args_remote, stop)))
            artifacts.callback(collect, target + ["cat", remote], filename)
        browser_cmd = ["node", str(HERE / "browser.mjs"), args.chrome_debug_url, dashboard_url, str(browser_folder)]
        if args.record_dashboard_video:
            browser_cmd.append("--video")
        processes.append(stack.enter_context(recorder(browser_cmd, (browser_folder / "stop").touch)))
        for _ in range(30):
            if any(process.poll() is not None for process in processes):
                raise RuntimeError("An evidence recorder exited before arming")
            if ((browser_folder / "rendered.jsonl").exists()
                    and "Ready for the next run" in (browser_folder / "rendered.jsonl").read_text()
                    and subprocess.run(mission + ["test", "-s", telemetry_path]).returncode == 0
                    and subprocess.run(gazebo + ["test", "-s", truth_path]).returncode == 0):
                break
            time.sleep(.4)
        else:
            raise RuntimeError("The dashboard and telemetry recorders did not become ready")

        # Sample after fixture changes and recorder discovery, before arming.
        # A live PX4 clock alone cannot establish camera or detector health.
        for _ in range(6):
            current = status(dashboard_url)
            ages = current["age_seconds"]
            if any(ages.get(feed, 999) > .5 for feed in ("raw", "processed", "yolo")):
                raise RuntimeError(f"Camera feeds became stale before takeoff: {ages}")
            time.sleep(.25)

        with (out / "flight.log").open("w") as stream:
            run(mission + entry + [replay, "--confirm-sitl", "--namespace", args.namespace,
                                   "--mission", "SignSurvey", "--cycles", "1"], stdout=stream, stderr=subprocess.STDOUT, timeout=550)
        final = status(dashboard_url)
        (out / "final.json").write_text(json.dumps(final, indent=2) + "\n")
        time.sleep(1)
    print(f"Recorded {args.scene} survey and separate return-to-start flight.", flush=True)


if __name__ == "__main__":
    main()

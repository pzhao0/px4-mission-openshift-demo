"""Build the project-owned AMD64 images in an OpenShift namespace."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import time
import uuid

ROOT = Path(__file__).resolve().parents[1]
LOCK = json.loads((ROOT / "dependencies.lock.json").read_text())
IGNORE = shutil.ignore_patterns("__pycache__", "*.pyc", ".DS_Store")


def run(*args, **kwargs):
    return subprocess.run(args, check=True, text=True, timeout=600, **kwargs)


def stage_mission(mission, survey_model):
    shutil.copytree(ROOT / "containers" / "mission", mission, ignore=IGNORE)
    for name in ("mission_demo", "aruco_tracker"):
        shutil.copytree(ROOT / "applications" / name, mission / "src" / name, ignore=IGNORE)
    for name in ("dashboard", "runtime"):
        shutil.copytree(ROOT / "applications" / name, mission / name, ignore=IGNORE)
    models = mission / "runtime" / "models"
    models.mkdir()
    shutil.copy(survey_model or ROOT / "models/coco-yolo26n.onnx", models / "coco-yolo26n.onnx")
    shutil.copytree(ROOT / "licenses", mission / "notices")
    shutil.copy(ROOT / "docs" / "THIRD_PARTY_NOTICES.md", mission / "notices")


def write_build_info(mission, base_image=None):
    hashes = {str(p.relative_to(mission)): hashlib.sha256(p.read_bytes()).hexdigest()
              for p in sorted(mission.rglob("*")) if p.is_file()}
    info = {"dependencies": LOCK, "files": hashes}
    if base_image:
        info["base_image"] = base_image
    (mission / "build-info.json").write_text(json.dumps(info, indent=2))


def stage(contexts, survey_model=None):
    """Stage the message definitions that match the published PX4 binary."""
    px4 = contexts / "px4"
    run("git", "init", "--quiet", px4)
    git = ["git", "-C", px4]
    source = LOCK["px4"]
    run(*git, "remote", "add", "origin", source["url"])
    run(*git, "sparse-checkout", "set", "msg", "srv")
    run(*git, "fetch", "--quiet", "--depth=1", "--filter=blob:none", "origin", source["revision"])
    run(*git, "checkout", "--quiet", "--detach", source["revision"])

    for role in ("desktop", "mavlink-router"):
        shutil.copytree(ROOT / "containers" / role, contexts / role, ignore=IGNORE)
    mission = contexts / "mission"
    stage_mission(mission, survey_model)
    for category, folders in (("msg", ("msg", "msg/versioned")), ("srv", ("srv",))):
        target = mission / "px4_msgs" / category
        target.mkdir(parents=True)
        for folder in folders:
            for path in (px4 / folder).glob(f"*.{category}"):
                shutil.copy(path, target)
    write_build_info(mission)


def build_one(namespace, context, run_id, arguments, dockerfile="Dockerfile"):
    name = f"px4-{context.name}-{run_id}"
    label = {"app.kubernetes.io/part-of": "px4-mission-build"}
    meta = {"name": name, "labels": label}
    config = json.loads((ROOT / "build/openshift/buildconfig.json").read_text())
    config["metadata"] = meta
    config["spec"]["strategy"]["dockerStrategy"]["dockerfilePath"] = dockerfile
    config["spec"]["strategy"]["dockerStrategy"]["buildArgs"] = [
        {"name": key, "value": value} for key, value in arguments.items()]
    config["spec"]["output"] = {"to": {"kind": "ImageStreamTag", "name": name + ":build"}}
    stream = {"apiVersion": "image.openshift.io/v1", "kind": "ImageStream", "metadata": meta}
    oc = ["oc", "-n", namespace]
    run(*oc, "create", "-f", "-", input=json.dumps({"apiVersion": "v1", "kind": "List", "items": [stream, config]}))
    build = run(*oc, "start-build", name, f"--from-dir={context}", "-o", "name",
                stdout=subprocess.PIPE).stdout.strip()
    print(f"Building {context.name}. Follow with oc -n {namespace} logs -f {build}", flush=True)
    deadline = time.monotonic() + 7500
    while time.monotonic() < deadline:
        obj = json.loads(run(*oc, "get", build, "-o", "json", stdout=subprocess.PIPE).stdout)
        status = obj["status"]
        if status["phase"] == "Complete":
            digest = status["output"]["to"]["imageDigest"]
            if not re.fullmatch(r"sha256:[0-9a-f]{64}", digest):
                raise ValueError(f"Missing output digest for {build}")
            reference = status["outputDockerImageReference"].rsplit(":", 1)[0] + "@" + digest
            print(f"Built {context.name}: {reference}", flush=True)
            return reference
        if status["phase"] in {"Failed", "Error", "Cancelled"}:
            raise RuntimeError(f"{build} {status['phase']}. Inspect oc -n {namespace} logs {build}")
        time.sleep(10)
    raise TimeoutError(f"Timed out waiting for {build}")


def build(namespace, output, survey_model=None):
    run_id = uuid.uuid4().hex[:10]
    with tempfile.TemporaryDirectory(prefix="px4-build-") as tmp:
        contexts = Path(tmp)
        stage(contexts, survey_model)
        arguments = {
            "mission": {"BASE_IMAGE": LOCK["ros"], "MSGS_REV": LOCK["px4_msgs"], "INTERFACE_REV": LOCK["interface"]},
            "desktop": {"BASE_IMAGE": LOCK["px4"]["image"]},
            "mavlink-router": {"BASE_IMAGE": LOCK["alpine"], "ROUTER_REV": LOCK["router"]},
        }
        with ThreadPoolExecutor(max_workers=3) as pool:
            jobs = {role: pool.submit(build_one, namespace, contexts / role, run_id, args)
                    for role, args in arguments.items()}
            images = {role: future.result() for role, future in jobs.items()}
        images["simulator"] = LOCK["px4"]["image"]
        images["zenoh"] = LOCK["zenoh"]
        write_images(output, images)


def write_images(output, images):
    with tempfile.NamedTemporaryFile("w", dir=output.parent, delete=False) as out:
        json.dump({"images": images}, out, indent=2)
        out.write("\n")
    Path(out.name).replace(output)
    print(f"Wrote {output}", flush=True)


def build_survey(namespace, output, base_image, survey_model):
    if output.exists():
        previous = json.loads(output.read_text())
        if set(previous.get("images", {})) != {"mission"}:
            raise ValueError(f"{output} is not a mission-only image lock. Choose a separate output file.")
    with tempfile.TemporaryDirectory(prefix="px4-survey-build-") as tmp:
        mission = Path(tmp) / "mission"
        stage_mission(mission, survey_model)
        write_build_info(mission, base_image)
        reference = build_one(namespace, mission, uuid.uuid4().hex[:10],
                              {"BASE_IMAGE": base_image}, "Dockerfile.survey")
    write_images(output, {"mission": reference})


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--namespace", required=True)
    parser.add_argument("--output", type=Path, help="Image lock output (mission-only builds default to mission.image.json)")
    parser.add_argument("--survey-model", type=Path, help="Override the bundled survey ONNX model")
    parser.add_argument("--mission-base", help="Build only the survey mission image from this pinned mission image")
    args = parser.parse_args()
    if args.mission_base:
        if not re.fullmatch(r"[^ @]+@sha256:[0-9a-f]{64}", args.mission_base):
            parser.error("--mission-base requires a digest-pinned image")
        build_survey(args.namespace, args.output or Path("mission.image.json"), args.mission_base, args.survey_model)
    else:
        build(args.namespace, args.output or Path("images.lock.json"), args.survey_model)

"""Check the drone camera, hold, rendered map and landing against simulator truth."""
import argparse
import csv
import json
import math
from pathlib import Path
import re
from statistics import median


def rows(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def evaluate(folder):
    fixture = json.loads((folder / 'fixture.json').read_text())
    placement = fixture['gazebo_sign_en_m']
    mission_log = (folder / 'mission.log').read_text()
    assert 'PASS consecutive SITL' in (folder / 'flight.log').read_text()
    final = json.loads((folder / 'final.json').read_text())['mission']
    assert final['landed'] and not final['armed'] and final['at_start'] and final['phase'] == 'ready'
    screens = rows(folder/'dashboard/rendered.jsonl')
    telemetry = rows(folder/'telemetry.jsonl')
    with (folder/'gazebo.csv').open() as stream:
        truth = [(int(p['sim_ns']), float(p['east_m']), float(p['north_m']), float(p['up_m']))
                 for p in csv.DictReader(stream)]
    returns = re.findall(r'SURVEY returning sim_ns=(\d+) spotted=(\d) coverage_complete=(\d)', mission_log)
    assert len(returns) == 1
    returning, spotted, coverage = map(int, returns[0])
    assert coverage and 'SURVEY detector_gap' not in mission_log
    assert bool(spotted) == bool(placement)
    ends = re.findall(r'SURVEY pass_end sim_ns=(\d+) pass=(\d+) fresh=(\d+) max_gap_ns=(\d+) outcome=(\w+)', mission_log)
    assert [int(p) for _,p,_,_,_ in ends] == list(range(1, len(ends)+1))
    assert all(int(n) > 0 for _,_,n,_,_ in ends)
    decisions = [r for r in telemetry if r['kind'] == 'decision']
    assert any(r.get('phase') == 'demo_complete' and r['landed'] and not r['armed'] for r in telemetry)
    pad = next(p for p in truth if p[0] > returning and p[3] < .05)
    pad_error = math.dist(pad[1:3], (8,-4))
    assert pad_error <= .35
    assert any(s.get('feed') == 'YOLO / STOP SIGN' and s['feedVisible'] for s in screens)
    landing = next(s for s in screens if s['heading'] == 'Demo · Precision landing' and s['feedVisible'] and s['feed']=='ARUCO')
    # Older recordings pinned the approach frame and labeled the map marker Hold.
    hold_photo_dashboard = 'hold_photo_ns' in landing
    marker = 'Sign found' if hold_photo_dashboard else 'Hold'
    assert landing['map_legend'] == ['Planned lanes', 'Flight path', marker] and landing['trail_points'] > 10
    report = dict(scene=fixture['scene'], sign_en_m=placement, passes=len(ends), hold_en_m=landing['hold_en'],
                  touchdown_pad_offset_m=round(pad_error,3), returned_to_start=True,
                  fresh_survey_frames=sum(int(n) for _,_,n,_,_ in ends),
                  max_search_frame_gap_s=round(max(int(g) for _,_,_,g,_ in ends)/1e9, 3))
    report["ground"] = fixture.get("ground", "plain")
    if placement is None:
        if hold_photo_dashboard:
            assert not landing['pinnedVisible'] and not landing['hold_photo_ns']
        assert len(ends) == 6 and all(outcome=='complete' for *_,outcome in ends)
        assert 'SURVEY sign_spotted' not in mission_log and 'SURVEY hold_start' not in mission_log
        assert all(s.get('hold_en') is None for s in screens if s['heading'] != 'Ready for the next run')
        assert not any(d['detected'] for d in decisions if d['frame_ns'] >= final['survey']['run_ns'] and d['frame_ns']<=returning)
        return report
    completed = re.findall(r'SURVEY hold_complete frame_ns=(\d+) start_ns=(\d+)', mission_log)
    assert len(completed) == 1
    end, start = map(int, completed[0])
    samples = sorted((r for r in decisions if start <= r['frame_ns'] <= end), key=lambda r:r['frame_ns'])
    assert len(samples)>=3 and end-start >= 2_000_000_000
    assert all(r['face_valid'] and r['detected'] and r['confidence']>=.25 for r in samples)
    gaps = [t-s for s,t in zip([start]+[r['frame_ns'] for r in samples], [r['frame_ns'] for r in samples]+[end])]
    assert max(gaps)<=250_000_000
    offsets=[max(abs(d['box_offset'][axis]) for d in samples) for axis in range(2)]
    assert max(offsets)<=.0625
    # Frame stamps establish visual centering. PX4 evaluates speed when those
    # frames arrive, so compare telemetry to the actual phase transitions.
    hold_states = re.findall(r'Demo phase: search_investigating .*?\(sim_ns=(\d+)\)', mission_log)
    decision_start = int(hold_states[-1])
    decision_end = int(ends[-1][0])
    assert start <= decision_start <= start + 500_000_000
    assert end <= decision_end <= end + 500_000_000
    held=[p for p in truth if start<=p[0]<=decision_end]
    assert held
    speeds = [math.hypot(*r["velocity_ned_mps"]) for r in telemetry
              if r["kind"] == "position" and r["valid"] and decision_start <= r["sim_ns"] <= decision_end]
    assert speeds
    error=max(math.dist(p[1:3],placement[:2]) for p in held)
    assert error <= .55
    assert landing['pinnedVisible'] and math.dist(landing['hold_en'],placement[:2]) <= .55
    if hold_photo_dashboard:
        assert landing['hold_photo_ns'] == start
        assert any(s['heading'] == 'Sign found' and s['feedVisible'] for s in screens)
    else:
        assert any(s['heading'] == 'Investigating' and s['feedVisible'] for s in screens)
    report.update(hold_seconds=round((end-start)/1e9,3), scored_hold_frames=len(samples),
                  max_box_offset_px=[round(offsets[0]*320,2),round(offsets[1]*240,2)],
                  max_horizontal_hold_error_m=round(error,3),
                  max_recorded_hold_speed_mps=round(max(speeds),3),
                  minimum_hold_confidence=round(min(r['confidence'] for r in samples),3))
    search_start = int(re.search(r"SURVEY pass_start sim_ns=(\d+) pass=1\b", mission_log)[1])
    accepted = re.search(r"SURVEY sign_spotted sim_ns=\d+ frame_ns=(\d+) pass=\d+ confidence=([\d.]+)", mission_log)
    accepted_frame = int(accepted[1])
    first_detection = min((d for d in decisions if d["detected"] and
                           search_start <= d["frame_ns"] <= accepted_frame), key=lambda d: d["frame_ns"])
    report.update(first_yolo_detection_s=round((first_detection["frame_ns"] - search_start) / 1e9, 3),
                  first_yolo_confidence=round(first_detection["confidence"], 3),
                  first_accepted_sighting_s=round((accepted_frame - search_start) / 1e9, 3),
                  first_accepted_confidence=float(accepted[2]),
                  median_hold_confidence=round(median(r["confidence"] for r in samples), 3))
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder',type=Path)
    args=parser.parse_args()
    result=evaluate(args.folder)
    (args.folder/'measurements.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result))

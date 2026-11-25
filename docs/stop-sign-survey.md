# Face-up stop-sign survey

The PX4 SITL drone uses its own downward camera at 320 × 240 and 10 Hz. Stock COCO YOLO26n recognizes the sign. The YOLO box midpoint selects the inspection point. The calibrated camera and known 1.5 m octagonal face locate that point in metres. A green circle shows the measured midpoint against the white camera-center cross. The flight mode receives the observation timestamp, measured point on the sign, confidence and YOLO box offset. The Gazebo world defines the default sign placement. The replay fixture records the placement for evaluation.

A hold needs a fresh complete sign observation with the YOLO midpoint within 20 horizontal and 15 vertical pixels of the camera center, aircraft speed below 0.25 m/s and altitude within 0.25 m of 5 m. The conditions must hold for two continuous simulation seconds. Losing the sign before confirmation returns the aircraft to its interrupted pass. The ArUco marker serves the separate landing phase.

On desktop, the dashboard shows the raw camera, live detector output and flight path from left to right. When the centered hold begins, the detector panel pins the annotated frame that started the hold. The **Sign found** marker shows the aircraft location during the hold. An interrupted attempt clears the photo, and a successful capture stays visible through landing and return.

## Install

Build and deploy both missions from the repository root.

```sh
./demo build --namespace my-mission
./demo deploy --namespace my-mission --images images.lock.json
```

Connect QGroundControl and open the dashboard using the addresses printed by the launcher. The flight mode menu offers **DemoWaypoints**, **SignSurvey**, and **ReturnToStart** in QGroundControl. The world starts with the face-up sign at the early placement. Select **SignSurvey** to fly this scene manually. [Connection instructions](../README.md#connect-to-the-simulator)

## Record a flight

Connect QGroundControl and wait for **Ready for the next run**. Start an isolated Chrome session with a local DevTools port. From the repository root run the harness once per scene, choosing a new output directory each time. The harness restores the world’s default sign for `early`, moves it for `late`, or removes it for `empty` while the aircraft is disarmed, flies the survey, records the dashboard, and flies the separate return to start. It requires Python, Node.js 22 or later, and OpenShift CLI access. Install ffmpeg when using `--record-dashboard-video`.

```sh
python3 tests/survey_sitl/run.py early --namespace my-mission --release px4-mission \
  --chrome-debug-url http://127.0.0.1:9227 \
  --output /tmp/sign-early --record-dashboard-video --confirm-sitl
python3 tests/survey_sitl/evaluate.py /tmp/sign-early
```

The replay uses `--ground grass-gravel` by default. Pass `--ground plain` to reproduce the original background. Repeat with `late` and `empty` for the other placements. The evaluator reads frame-level detections, the independently recorded Gazebo trajectory, the rendered dashboard and PX4 mission events. The fixture uses the same aircraft, camera and model for all three runs.

## Model

The mission image includes [models/coco-yolo26n.onnx](../models/coco-yolo26n.onnx). The 640 × 640 end-to-end ONNX model comes from the Ultralytics assets release tagged v8.4.0. Its SHA-256 is `2e947b787d9e787b93a16772a5f55b1d4d8c4d86f53146149c5d6a642442d6f7`. The sign mesh and texture are local demo assets. [Upstream model](https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo26n.onnx)

# Build and operations

Run these commands from the repository root. Build and deploy use the same OpenShift project so the workload service account can pull the project images. Builds require the binary BuildConfig and ImageStream APIs, an integrated image registry, AMD64 workers, and outbound access to GitHub, Docker Hub and the package repositories configured in the base images. The launcher needs Git with sparse-checkout support and access to GitHub.

```sh
./demo build --namespace my-mission
./demo deploy --namespace my-mission
```

The three binary builds run concurrently. Each requests 500m CPU and 3 GiB memory and has a two-hour deadline. Progress includes an `oc logs` command for each build. Each invocation creates uniquely named BuildConfigs and ImageStreams and writes `images.lock.json` only after all three builds succeed. Retain the ImageStreams while a deployment references their images. To keep separate locks for multiple projects, set `--output` at build time and pass the same path with `--images` at deploy time.

The builder fetches message and service definitions from the PX4 source revision that produced the simulator image. The mission build uses those definitions, compiles both ROS packages and runs the executor regression. Source revisions and base images are pinned in [dependencies.lock.json](../dependencies.lock.json). Update the PX4 image digest and source revision together.

The standard build packages the YOLO model and runtime for `SignSurvey` alongside `DemoWaypoints`.

## Runtime images

| Component | Image source |
| --- | --- |
| PX4 and Gazebo | Pulls the prebuilt `docker.io/px4io/px4-sitl-gazebo` image by digest from the PX4 `v1.18.0-rc2` release. Helm mounts the world and startup script from a ConfigMap. [PX4 packages](https://docs.px4.io/main/en/simulation/px4_sitl_prebuilt_packages) · [Release](https://github.com/PX4/PX4-Autopilot/releases/tag/v1.18.0-rc2) |
| Mission, camera bridge, dashboard, and Micro XRCE-DDS Agent | Builds on `docker.io/px4io/px4-dev-ros2-gazebo` for ROS 2 Jazzy and the Agent. The build adds the mission executor, detector, PX4 messages, camera bridge configuration, and dashboard. [Mission image recipe](../containers/mission/Dockerfile) |
| Gazebo desktop | Builds on the same prebuilt PX4/Gazebo image and adds Xvfb, noVNC, and the Gazebo GUI configuration. [Desktop image recipe](../containers/desktop/Dockerfile) |
| MAVLink router | Builds the router from a pinned source revision using Alpine. [Router image recipe](../containers/mavlink-router/Dockerfile) |
| Zenoh camera transport | Both pods pull `docker.io/eclipse/zenoh-bridge-ros2dds` by digest. [Zenoh ROS 2 DDS bridge](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds#docker-image) |

## DDS domain configuration

The default DDS domains are 61 for camera data in the simulator pod and 62 for the mission pod. The PX4 client uses the mission domain. The simulator startup script resolves the Agent Service to an IPv4 address for the PX4 parameter `UXRCE_DDS_AG_IP`, and PX4 uses `ROS_DOMAIN_ID` to set `UXRCE_DDS_DOM_ID`. See the [architecture diagram](../README.md#architecture) for the transport paths.

Use distinct DDS domains for releases sharing a network. For example, put the following in a local values file and pass it to every deploy of the second release. The Gazebo partition and bootstrap Services already include release identity. Leave domain 101 free for regression checks.

```yaml
domains:
  simulator: 63
  mission: 64
```

```sh
./demo build --namespace another-mission --output another-mission.images.json
./demo deploy --namespace another-mission --images another-mission.images.json --values domains.yaml
```

## Troubleshooting

- **ImagePullBackOff** — Check the repositories and digests in the image lock. For project images, verify that the lock was built on this cluster in the deployment namespace. For upstream images, check the cluster's registry access.
- **Pending pods** — Check node architecture, requested CPU and memory, and project quota with `oc describe pod`.
- **Waiting for data** — Inspect `oc logs deploy/px4-mission-mission -c mission`, then the Agent and camera-transport logs. Check the distinct DDS domains, release Service names and image versions. An advancing clock, valid position and fresh camera frames are required. Marker detection becomes relevant during flight.
- **Failed or partial installation** — Inspect `helm status px4-mission`, `oc get events` and the named container logs in the target project. For a failed, unused install, remove the release, correct the input and redeploy. For an active simulation, land and disarm before teardown. A direct Helm change bypasses the launcher's update check.
- **QGC connection lost after an update** — Restart the local port forward and reconnect the TCP link. A normal replay uses the existing session.

Supply `-n PROJECT` to the diagnostic commands. Build resources carry the label `app.kubernetes.io/part-of=px4-mission-build`.

## Maintainer checks

For a build with an image lock, render and validate the chart before installing.

```sh
helm lint deploy/chart -f images.lock.json
helm template px4-mission deploy/chart -n my-mission -f images.lock.json |
  oc -n my-mission apply --dry-run=server -f -
```

To exercise the existing observer and HTTP tests after install, use an isolated ROS domain and the launch environment from the mission image.

```sh
oc -n my-mission exec -i deploy/px4-mission-simulator -c camera-bridge -- \
  env ROS_DOMAIN_ID=101 ROS_LOCALHOST_ONLY=1 /opt/mission/entrypoint.sh python3 - \
  < applications/runtime/test_observe.py
oc -n my-mission exec -i deploy/px4-mission-mission -c mission -- \
  env ROS_DOMAIN_ID=101 ROS_LOCALHOST_ONLY=1 /opt/mission/entrypoint.sh python3 - \
  --viewer /opt/camera-view/camera_view.py < tests/dashboard_http.py
```

For the opt-in replay check, connect QGC to the selected simulation, confirm its fresh, disarmed starting state, and run the existing driver inside its mission container. This exercises normal mode selection and arming for two full cycles. The replay defaults to `DemoWaypoints` when both missions are available. To choose a mission explicitly, pass `--mission DemoWaypoints` or `--mission SignSurvey` to the replay driver. Record the pod UIDs and restart counts before and after. Installing the chart and running the observer and HTTP tests never initiate a flight.

```sh
oc -n my-mission exec -i deploy/px4-mission-mission -c mission -- \
  /opt/mission/entrypoint.sh python3 - --namespace my-mission --confirm-sitl --cycles 2 \
  < tests/replay_sitl.py
```

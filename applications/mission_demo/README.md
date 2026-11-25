# Repeatable mission demo

This package owns the OpenShift demonstration. Its waypoint and precision-landing
modes are adapted from the workshop's `precision_land_executor` example under
CC BY-SA 4.0. The original authors are listed in the workshop package metadata.
[Source and attribution](../../docs/THIRD_PARTY_NOTICES.md)

`DemoWaypoints` runs takeoff → four waypoints → `DemoPrecisionLand` → touchdown
and disarm. The first demo activation records the original local NED start and
heading. `ReturnToStart` is a separate operator-selected sequence: take off,
fly above that saved start, land and disarm. It preserves the start across rearming
and marks the session ready for another demonstration. No controller, PX4, Gazebo,
or pod restart is needed between complete demo/return cycles.

`SignSurvey` searches up to six lanes, holds over a confirmed stop sign, and uses the same precision landing and return sequence. [Survey guide](../../docs/stop-sign-survey.md)

## Implementation

| File | Responsibility |
|---|---|
| `DemoSession.*` | Saved start, position freshness, session progress and status |
| `MissionModes.*` | Demo waypoints and marker landing; per-activation state reset |
| `SurveyMode.*`, `SurveyLanes.hpp` | Stop-sign survey, alignment, hold and recovery |
| `ReturnToStart.*` | Return transit at takeoff height and original heading; arrival/velocity tolerance |
| `MissionExecutor.*` | Demo and return sequences, failure handling and return-mode selection |
| `main.cpp` | Register waypoint and return executors, the shared landing mode, and the survey executor |

The dashboard's **Return to start** button calls `/mission/prepare_return`
(`std_srvs/srv/Trigger`). This only selects `ReturnToStart`; the operator must arm
in QGroundControl. Direct QGC selection of `ReturnToStart` is also supported after
a demo. Arming is never exposed through the dashboard. Hold/Land in QGC continue
to interrupt a sequence without being overridden by the executor.

`/mission/demo_status` (`std_msgs/msg/String`, JSON) reports `phase`, `message`,
`can_return`, `return_reason`, `telemetry_fresh`, `armed`, `at_start`,
`completed_runs`, `completed_returns`, mode IDs, `start_ned`, and horizontal
`distance_to_start_metres`. The dashboard disables selection when this status is
stale. The mission independently rejects requests without a saved start, while
airborne/armed, during a sequence, after stale telemetry, or after a local-frame
reset. Return arrival requires position within 0.35 m and speed below 0.3 m/s for
one second before landing; the completed flight must finish within 0.75 m
horizontally and 0.5 m vertically of the start to become ready.

The mission process holds the saved start and counters in memory. The demo
waypoints and marker location use the simulation world's local NED coordinates.
[Demo walkthrough](../../README.md#demo-walkthrough)

## Validation

The regression executable uses the production executor and PX4 ROS 2 library
with fake autopilot registration/status/completion messages on isolated domain
101. It checks takeoff altitude semantics, stage failures, cancellation, disarm
completion, guarded return selection, two demo/return cycles with one saved
start, stale telemetry, and coordinate resets.

Inside the built image, after sourcing `/opt/mission_ws/install/setup.bash`:

```bash
cmake -S /opt/mission_ws/src/mission_demo/test -B /tmp/mission-demo-tests
cmake --build /tmp/mission-demo-tests -j2
ctest --test-dir /tmp/mission-demo-tests --output-on-failure
```

Image builds run these checks. For OpenShift validation commands, see the
[maintainer checks](../../docs/build.md#maintainer-checks).

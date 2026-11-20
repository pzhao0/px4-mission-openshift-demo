#!/usr/bin/env python3
"""Run two demo/return flights against the explicitly selected simulated vehicle."""
import argparse
import json
import os
import time
import urllib.request


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--confirm-sitl', action='store_true', required=True,
                        help='Explicitly authorize arming this simulation for the validation flights')
    parser.add_argument('--cycles', type=int, default=2)
    parser.add_argument('--namespace', required=True, help='Expected namespace of the selected SITL pod')
    args = parser.parse_args()
    if args.namespace != os.environ.get('SITL_NAMESPACE') or os.environ.get('ROS_DOMAIN_ID') is None:
        parser.error('Run inside the mission container of the selected SITL installation')
    if args.cycles < 1:
        parser.error('Select at least one replay cycle')
    dashboard_url = 'http://127.0.0.1:8080'
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from px4_msgs.msg import VehicleCommand, VehicleCommandAck

    rclpy.init()
    node = Node('replay_sitl_validation')
    suffix = lambda cls: f'_v{cls.MESSAGE_VERSION}' if cls.MESSAGE_VERSION else ''
    publisher = node.create_publisher(VehicleCommand, '/fmu/in/vehicle_command' + suffix(VehicleCommand), 1)
    acknowledgements = []
    subscription = node.create_subscription(
        VehicleCommandAck, '/fmu/out/vehicle_command_ack' + suffix(VehicleCommandAck),
        lambda msg: acknowledgements.append(msg), qos_profile_sensor_data)

    def dashboard():
        with urllib.request.urlopen(dashboard_url + '/status', timeout=5) as response:
            data = json.load(response)
        if data['age_seconds'].get('mission', 999) > 2:
            raise RuntimeError('Mission dashboard status is stale')
        return data

    def command(command_id, param1):
        acknowledgements.clear()
        msg = VehicleCommand(command=command_id, param1=float(param1), source_system=255,
                             source_component=192, target_system=1, target_component=1, from_external=True)
        deadline = time.monotonic() + 8
        next_publish = 0
        while time.monotonic() < deadline:
            if time.monotonic() >= next_publish:
                publisher.publish(msg)
                next_publish = time.monotonic() + 1
            rclpy.spin_once(node, timeout_sec=.1)
            replies = [ack for ack in acknowledgements if ack.command == command_id and ack.target_component == 192]
            if replies:
                if replies[-1].result != VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
                    raise RuntimeError(f'Command {command_id} rejected: {replies[-1].result}')
                return
        raise RuntimeError(f'Command {command_id} acknowledgement timed out')

    def wait_phase(phase, timeout=240):
        deadline = time.monotonic() + timeout
        previous = None
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            state = dashboard()['mission']
            current = state['phase']
            if current != previous:
                print(json.dumps({key: state.get(key) for key in ('phase', 'message', 'completed_runs', 'completed_returns', 'start_ned', 'distance_to_start_metres')}), flush=True)
                previous = current
            if current == 'stopped':
                raise RuntimeError(state['message'])
            if current == phase:
                return state
            time.sleep(.25)
        raise TimeoutError(f'Timed out waiting for {phase}')

    try:
        # Establish DDS discovery before the first command.
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=.1)
        initial = dashboard()['mission']
        if initial['armed'] or not initial['telemetry_fresh'] or initial['phase'] != 'ready':
            raise RuntimeError('Start validation only with a fresh, disarmed, ready simulator session')
        original_start = None
        for cycle in range(args.cycles):
            print(f'SITL cycle {cycle + 1}: demonstration', flush=True)
            command(VehicleCommand.VEHICLE_CMD_SET_NAV_STATE, initial['demo_mode_id'])
            command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1)
            finished = wait_phase('demo_complete')
            if finished['armed']:
                raise RuntimeError('Demo completed before disarm')
            if original_start is None:
                original_start = finished['start_ned']
            assert finished['start_ned'] == original_start
            state = dashboard()
            request = urllib.request.Request(dashboard_url + '/return-to-start', data=b'{}', headers={
                'Content-Type': 'application/json', 'X-Mission-Token': state['csrf_token']})
            with urllib.request.urlopen(request, timeout=10) as response:
                result = json.load(response)
            if not result['success']:
                raise RuntimeError(result['message'])
            wait_phase('return_awaiting_arm', timeout=10)
            print(f'SITL cycle {cycle + 1}: return (normal arming checks)', flush=True)
            command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1)
            ready = wait_phase('ready')
            assert not ready['armed'] and ready['at_start'] and ready['start_ned'] == original_start
            assert ready['completed_runs'] == initial['completed_runs'] + cycle + 1
            assert ready['completed_returns'] == initial['completed_returns'] + cycle + 1
        print('PASS consecutive SITL demo/return cycles: ' + json.dumps(ready), flush=True)
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

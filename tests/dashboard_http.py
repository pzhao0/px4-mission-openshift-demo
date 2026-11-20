#!/usr/bin/env python3
"""Exercise the real HTTP/ROS dashboard without connecting to the demo vehicle."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import threading
import time
import urllib.error
import urllib.request


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--viewer', type=Path, required=True)
    parser.add_argument('--port', type=int, default=8081)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '101':
        parser.error('Use isolated ROS_DOMAIN_ID=101, never the live vehicle domain')

    import rclpy
    from rclpy.executors import ExternalShutdownException
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    rclpy.init()
    node = rclpy.create_node('dashboard_http_fixture')
    publisher = node.create_publisher(String, '/mission/demo_status', 1)
    state = {'can_return': False, 'calls': 0}

    def publish():
        publisher.publish(String(data=json.dumps({'phase': 'demo_complete', 'can_return': state['can_return']})))

    def select(_request, response):
        state['calls'] += 1
        response.success = True
        response.message = 'ReturnToStart selected; arm separately in QGroundControl.'
        return response

    node.create_timer(.1, publish)
    node.create_service(Trigger, '/mission/prepare_return', select)

    def spin():
        try:
            rclpy.spin(node)
        except ExternalShutdownException:
            pass

    worker = threading.Thread(target=spin)
    worker.start()
    process = subprocess.Popen(['python3', str(args.viewer)], env={**os.environ, 'CAMERA_VIEW_PORT': str(args.port)})
    base = f'http://127.0.0.1:{args.port}'

    def request(path, headers=None, data=None):
        try:
            response = urllib.request.urlopen(urllib.request.Request(base + path, data=data, headers=headers or {}), timeout=10)
        except urllib.error.HTTPError as error:
            response = error
        with response:
            content = response.read()
            return response.status, json.loads(content) if response.headers.get_content_type() == 'application/json' else content

    def wait_status(predicate):
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            try:
                code, status = request('/status')
                if code == 200 and predicate(status):
                    return status
            except OSError:
                pass
            time.sleep(.1)
        raise AssertionError('Dashboard did not receive expected status')

    try:
        status = wait_status(lambda result: result.get('mission') is not None)
        assert request('/')[0] == 200
        assert request('/raw.jpg')[0] == 503
        assert request('/return-to-start', {'Content-Type': 'application/json'}, b'{}')[0] == 403
        headers = {'Content-Type': 'application/json', 'X-Mission-Token': status['csrf_token']}
        assert request('/return-to-start', headers, b'{}')[0] == 409
        state['can_return'] = True
        wait_status(lambda result: result['mission']['can_return'])
        assert request('/return-to-start', {**headers, 'Origin': 'https://unrelated.invalid'}, b'{}')[0] == 403
        code, result = request('/return-to-start', headers, b'{}')
        assert code == 200 and result['success']
        assert state['calls'] == 1, 'A rejected HTTP request reached the ROS selection interface'
        print('PASS dashboard: page/images, CSRF, origin, mission guard, ROS selection acknowledgement')
    finally:
        process.terminate()
        process.wait(timeout=5)
        rclpy.shutdown()
        worker.join(timeout=5)
        node.destroy_node()


if __name__ == '__main__':
    main()

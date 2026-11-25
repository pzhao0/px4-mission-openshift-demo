#!/usr/bin/env python3
"""Camera dashboard and operator-requested return-mode selection; never arms."""
import json
import os
import secrets
from pathlib import Path
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from px4_msgs.msg import VehicleLocalPosition
from rosgraph_msgs.msg import Clock
from survey_view import SurveyView
from std_msgs.msg import String
from std_srvs.srv import Trigger

PAGE = Path(__file__).with_name('camera_view.html').read_bytes()


class Viewer(Node):
    def __init__(self):
        super().__init__('mission_camera_viewer')
        self.lock = threading.Lock()
        self.frames = {}
        self.received = {}
        self.target = None
        self.mission = None
        self.csrf_token = secrets.token_urlsafe(32)
        self.bridge = CvBridge()
        self.subs = [self.create_subscription(
            Image, topic, lambda msg, key=key: self.frame(key, msg), qos_profile_sensor_data
        ) for key, topic in [('raw', '/camera'), ('processed', '/image_proc')]]
        self.subs.append(self.create_subscription(
            PoseStamped, '/target_pose', self.pose, qos_profile_sensor_data))
        self.subs.append(self.create_subscription(String, '/mission/demo_status', self.demo_status, 1))
        self.survey = None
        self.sim_ns = 0
        if os.environ.get('SURVEY_ENABLED') == 'true':
            self.survey = SurveyView()
            position = VehicleLocalPosition
            version = f'_v{position.MESSAGE_VERSION}' if position.MESSAGE_VERSION else ''
            for cls, topic, callback, qos in [
                (position, '/fmu/out/vehicle_local_position' + version, self.position, qos_profile_sensor_data),
                (Clock, '/clock', self.clock, qos_profile_sensor_data),
                (CompressedImage, '/mission/survey_preview', self.preview, 10),
            ]:
                self.subs.append(self.create_subscription(cls, topic, callback, qos))
        self.return_client = self.create_client(Trigger, '/mission/prepare_return')

    def clock(self, msg):
        self.sim_ns = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec

    def position(self, msg):
        if msg.xy_valid and msg.z_valid:
            with self.lock:
                self.survey.position(self.sim_ns, msg.y, msg.x, -msg.z)

    def preview(self, msg):
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        data = bytes(msg.data)
        with self.lock:
            self.survey.preview(stamp, data)
            self.frames['yolo'] = data
            self.received['yolo'] = time.monotonic()

    def demo_status(self, msg):
        try:
            mission = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if isinstance(mission, dict):
            with self.lock:
                if self.survey:
                    self.survey.status(mission)
                self.mission = mission
                self.received['mission'] = time.monotonic()

    def prepare_return(self):
        with self.lock:
            age = time.monotonic() - self.received.get('mission', 0)
            available = self.mission and self.mission.get('can_return') and age < 2
        if not available:
            return 409, {'success': False, 'message': 'Return is unavailable. Check the current mission status.'}
        if not self.return_client.service_is_ready():
            return 503, {'success': False, 'message': 'Mission controller is unavailable.'}
        done = threading.Event()
        future = self.return_client.call_async(Trigger.Request())
        future.add_done_callback(lambda _: done.set())
        if not done.wait(7):
            self.return_client.remove_pending_request(future)
            return 504, {'success': False, 'message': 'Selection acknowledgement timed out. Check QGroundControl before retrying.'}
        try:
            result = future.result()
            return (200 if result.success else 409), {'success': result.success, 'message': result.message}
        except Exception:
            return 503, {'success': False, 'message': 'Mission controller could not acknowledge the selection.'}

    def frame(self, key, msg):
        ok, encoded = cv2.imencode('.jpg', self.bridge.imgmsg_to_cv2(msg, 'bgr8'))
        if ok:
            with self.lock:
                self.frames[key] = encoded.tobytes()
                self.received[key] = time.monotonic()

    def pose(self, msg):
        p = msg.pose.position
        with self.lock:
            self.target = dict(x=p.x, y=p.y, z=p.z, frame=msg.header.frame_id)
            self.received['target'] = time.monotonic()


def main():
    rclpy.init()
    viewer = Viewer()

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            path = self.path.split('?', 1)[0]
            code = 200
            if path == '/':
                body, mime = PAGE, 'text/html; charset=utf-8'
            elif path == '/status':
                with viewer.lock:
                    data = {'age_seconds': {k: round(time.monotonic() - v, 2)
                                            for k, v in viewer.received.items()},
                            'last_target_metres': viewer.target,
                            'mission': viewer.mission, 'csrf_token': viewer.csrf_token,
                            'survey': viewer.survey.snapshot() if viewer.survey and viewer.mission and 'survey' in viewer.mission else None}
                body, mime = json.dumps(data).encode(), 'application/json'
            elif path in ('/raw.jpg', '/processed.jpg', '/yolo.jpg', '/sighting.jpg', '/sign-found.jpg'):
                with viewer.lock:
                    if path == '/sign-found.jpg' and viewer.survey:
                        body = viewer.survey.hold_image
                    elif path == '/sighting.jpg' and viewer.survey:
                        body = viewer.survey.first_image
                    else:
                        body = viewer.frames.get(path[1:-4])
                mime = 'image/jpeg'
                if body is None:
                    code, body, mime = 503, b'Waiting for an image', 'text/plain'
            else:
                code, body, mime = 404, b'Not found', 'text/plain'
            self.send_response(code)
            self.send_header('Content-Type', mime)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def log_message(self, *_):
            pass

        def do_POST(self):
            code, data = 404, {'success': False, 'message': 'Not found'}
            if self.path == '/return-to-start':
                origin = self.headers.get('Origin')
                valid_origin = origin is None or (
                    urlsplit(origin).scheme in ('http', 'https') and
                    urlsplit(origin).netloc.lower() == self.headers.get('Host', '').lower())
                token = self.headers.get('X-Mission-Token', '')
                if (not valid_origin or not secrets.compare_digest(token, viewer.csrf_token) or
                        self.headers.get_content_type() != 'application/json'):
                    code, data = 403, {'success': False, 'message': 'Refresh this page before requesting a return.'}
                else:
                    try:
                        length = int(self.headers.get('Content-Length', '0'))
                    except ValueError:
                        length = -1
                    if not 0 <= length <= 256:
                        code, data = 400, {'success': False, 'message': 'Invalid request size.'}
                    else:
                        self.rfile.read(length)
                        code, data = viewer.prepare_return()
            body = json.dumps(data).encode()
            self.send_response(code)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

    # The Service/edge-TLS Route connects to the pod interface.
    port = int(os.environ.get('CAMERA_VIEW_PORT', '8080'))
    server = ThreadingHTTPServer((os.environ.get('CAMERA_VIEW_HOST', '0.0.0.0'), port), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f'Mission camera dashboard listening on port {port} (arming remains in QGroundControl)', flush=True)
    try:
        rclpy.spin(viewer)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        server.shutdown()
        server.server_close()
        viewer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

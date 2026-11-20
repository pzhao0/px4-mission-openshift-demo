#!/usr/bin/env python3
"""Serve noVNC and its WebSocket proxy with a useful root landing URL."""
from urllib.parse import urlsplit

from websockify.websocketproxy import ProxyRequestHandler, WebSocketProxy

LANDING_URL = '/vnc_lite.html?autoconnect=true&resize=scale'


class NoVNCHandler(ProxyRequestHandler):
    def do_GET(self):
        if urlsplit(self.path).path == '/' and self.headers.get('Upgrade', '').lower() != 'websocket':
            self.send_response(302)
            self.send_header('Location', LANDING_URL)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', '0')
            self.end_headers()
            return
        super().do_GET()

    def do_HEAD(self):
        if urlsplit(self.path).path == '/':
            self.do_GET()
            return
        super().do_HEAD()


if __name__ == '__main__':
    WebSocketProxy(RequestHandlerClass=NoVNCHandler, listen_host='0.0.0.0',
                   listen_port=6080, target_host='127.0.0.1', target_port=5901,
                   web='/usr/share/novnc/').start_server()

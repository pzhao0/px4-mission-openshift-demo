#!/usr/bin/env python3
import json
import sys
import time

try:
    with open("/tmp/mission-status.json") as stream:
        status = json.load(stream)
    sys.exit(0 if status["ready"] and time.time() - status["observed_wall_time"] < 5 else 1)
except (OSError, ValueError, KeyError):
    sys.exit(1)

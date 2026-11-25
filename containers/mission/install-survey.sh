#!/bin/bash
set -eo pipefail
# Install into an isolated import directory so ROS keeps its system NumPy and OpenCV.
python3 -m pip install --no-cache-dir --no-deps --target /opt/mission/onnx onnxruntime==1.22.1
PYTHONPATH="/opt/mission/onnx:${PYTHONPATH:-}" python3 -c 'import cv2, numpy, onnxruntime'

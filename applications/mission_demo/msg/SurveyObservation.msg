# Capture time on the Gazebo clock.
builtin_interfaces/Time stamp
bool sign_detected
float32 confidence
bool face_valid
# Live box midpoint on the verified inspection face, in camera optical metres.
float32[3] box_midpoint_on_sign_m
# YOLO box midpoint minus image center, as fractions of frame width and height.
float32[2] box_offset

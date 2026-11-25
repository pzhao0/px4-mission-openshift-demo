"""Recognize the COCO stop-sign class and measure the demo's flat octagonal face."""
import ast

import cv2
import numpy as np
import onnxruntime as ort

# Red insert inside the white border of the 1.50 m Gazebo sign, metres across flats.
HALF_FACE = .75 * .94
BEVEL = HALF_FACE * (np.sqrt(2.) - 1.)
OCTAGON = np.float64([(-BEVEL, -HALF_FACE, 0), (BEVEL, -HALF_FACE, 0),
                      (HALF_FACE, -BEVEL, 0), (HALF_FACE, BEVEL, 0),
                      (BEVEL, HALF_FACE, 0), (-BEVEL, HALF_FACE, 0),
                      (-HALF_FACE, BEVEL, 0), (-HALF_FACE, -BEVEL, 0)])


class StopSignDetector:
    def __init__(self, model):
        options = ort.SessionOptions()
        options.intra_op_num_threads = 2
        self.session = ort.InferenceSession(str(model), options, providers=['CPUExecutionProvider'])
        classes = ast.literal_eval(self.session.get_modelmeta().custom_metadata_map['names'])
        self.sign_class = next(key for key, name in classes.items() if name == 'stop sign')
        self.input = self.session.get_inputs()[0].name

    def detect(self, frame):
        h, w = frame.shape[:2]
        scale = 640 / max(h, w)
        image = cv2.resize(frame, (round(w*scale), round(h*scale)))
        dx, dy = (640-image.shape[1])//2, (640-image.shape[0])//2
        canvas = np.full((640, 640, 3), 114, dtype=np.uint8)
        canvas[dy:dy+image.shape[0], dx:dx+image.shape[1]] = image
        blob = cv2.dnn.blobFromImage(canvas, 1/255., swapRB=True)
        predictions = self.session.run(None, {self.input: blob})[0][0]
        signs = predictions[(predictions[:, 5] == self.sign_class) & (predictions[:, 4] >= .25)]
        if not len(signs):
            return 0., None
        best = signs[signs[:, 4].argmax()]
        box = np.clip((best[:4] - [dx,dy,dx,dy])/scale, 0, [w-1,h-1,w-1,h-1])
        return float(best[4]), box.round().astype(int).tolist()


def inspection_point(frame, box, camera):
    """Locate the YOLO midpoint on the calibrated face of the inspection sign."""
    if camera is None or box is None:
        return None
    x1, y1, x2, y2 = box
    h, w = frame.shape[:2]
    if min(x1, y1, w-1-x2, h-1-y2) < 2:
        return None
    x, y = max(0, x1-3), max(0, y1-3)
    roi = frame[y:y2+4, x:x2+4]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    red = cv2.inRange(hsv, (0, 90, 45), (12, 255, 255)) | cv2.inRange(hsv, (168, 90, 45), (180, 255, 255))
    contours, _ = cv2.findContours(red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    face = max(contours, key=cv2.contourArea)
    if cv2.contourArea(face) < .50 * (x2-x1)*(y2-y1):
        return None
    corners = cv2.approxPolyDP(face, .025*cv2.arcLength(face, True), True)
    if len(corners) != 8 or not cv2.isContourConvex(corners):
        return None
    pixels = corners.reshape(8, 2).astype(np.float64) + [x, y]
    centered = pixels - pixels.mean(axis=0)
    pixels = pixels[np.argsort(np.arctan2(centered[:,1], centered[:,0]))]
    ok, rotation, translation = cv2.solvePnP(OCTAGON, pixels, camera, None, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok or not np.isfinite(translation).all() or not 3 < translation[2, 0] < 8:
        return None
    reprojection = cv2.projectPoints(OCTAGON, rotation, translation, camera, None)[0].reshape(8, 2)
    if np.max(np.linalg.norm(reprojection-pixels, axis=1)) > 3:
        return None
    # Map the live box midpoint into the fitted sign plane. Only a point on
    # the central part of this verified inspection face is an eligible aim point.
    midpoint = np.float64([[(x1 + x2) / 2, (y1 + y2) / 2]])
    to_face, _ = cv2.findHomography(reprojection, OCTAGON[:, :2])
    if to_face is None:
        return None
    face_xy = cv2.perspectiveTransform(midpoint[None, :, :], to_face)[0, 0]
    if not np.isfinite(face_xy).all() or np.linalg.norm(face_xy) > HALF_FACE / 2:
        return None
    axes, _ = cv2.Rodrigues(rotation)
    return (translation[:, 0] + axes[:, :2] @ face_xy).tolist()

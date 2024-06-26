"""Person detection with OAK-D and ROS2

The script uses an OAK-D camera and Spectacular AI to detect
persons and then a ROS2 Node publishes its position
relative to the camera onto the ``/detected_persons`` topic.

Positions:

 - X: distance from camera
 - Y: horizontal position
 - Z: vertical position
"""
import depthai as dai
import time
import cv2
import matplotlib.pyplot as plt
import spectacularAI
import threading
from pathlib import Path
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point


class PersonDetectorNode(Node):

    def __init__(self):
        super().__init__('person_detector')
        self.publisher_ = self.create_publisher(Point, 'detected_persons', 10)

def make_pipelines(nnBlobPath, showRgb):
    syncNN = True

    # Create pipeline
    pipeline = dai.Pipeline()
    vio_pipeline = spectacularAI.depthai.Pipeline(pipeline)
    spatialCalc = pipeline.create(dai.node.SpatialLocationCalculator)

    # Define sources and outputs
    camRgb = pipeline.createColorCamera()
    spatialDetectionNetwork = pipeline.createYoloSpatialDetectionNetwork()

    if showRgb:
        xoutRgb = pipeline.createXLinkOut()
    xoutNN = pipeline.createXLinkOut()
    xoutBoundingBoxDepthMapping = pipeline.createXLinkOut()

    if showRgb:
        xoutRgb.setStreamName("rgb")
    xoutNN.setStreamName("detections")
    xoutBoundingBoxDepthMapping.setStreamName("boundingBoxDepthMapping")

    # Properties
    camRgb.setPreviewSize(416, 416)
    camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    camRgb.setInterleaved(False)
    camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

    spatialDetectionNetwork.setBlobPath(nnBlobPath)
    spatialDetectionNetwork.setConfidenceThreshold(0.5)
    spatialDetectionNetwork.input.setBlocking(False)
    spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
    spatialDetectionNetwork.setDepthLowerThreshold(100)
    spatialDetectionNetwork.setDepthUpperThreshold(5000)

    # Yolo specific parameters
    spatialDetectionNetwork.setNumClasses(80)
    spatialDetectionNetwork.setCoordinateSize(4)
    spatialDetectionNetwork.setAnchors(np.array([10,14, 23,27, 37,58, 81,82, 135,169, 344,319]))
    spatialDetectionNetwork.setAnchorMasks({ "side26": np.array([1,2,3]), "side13": np.array([3,4,5]) })
    spatialDetectionNetwork.setIouThreshold(0.5)

    camRgb.preview.link(spatialDetectionNetwork.input)
    if showRgb:
        if syncNN:
            spatialDetectionNetwork.passthrough.link(xoutRgb.input)
        else:
            camRgb.preview.link(xoutRgb.input)

    spatialDetectionNetwork.out.link(xoutNN.input)
    spatialDetectionNetwork.boundingBoxMapping.link(xoutBoundingBoxDepthMapping.input)

    vio_pipeline.stereo.depth.link(spatialDetectionNetwork.inputDepth)

    return pipeline, vio_pipeline, spatialCalc

def make_tracker():
    """
    Simple tracker/smoother/clustring for the YOLO-detected objects.
    (The raw YOLO results look quite, well, raw, especially in 3D)
    """
    tracked_objects = []
    next_id = 1

    class TrackedObject:
        def __init__(self, t, p, l):
            self.position = p
            self.label = l
            self.last_seen = t
            self.n_detections = 1

            nonlocal next_id
            self.id = next_id
            next_id += 1

        def update(self, other):
            UPDATE_ALPHA = 0.2
            self.last_seen = other.last_seen
            self.position = UPDATE_ALPHA * other.position + (1.0 - UPDATE_ALPHA) * self.position
            self.n_detections += 1

        def __repr__(self):
            return '%s %d' % (self.label, self.id)

    CLUSTERING_DISTANCE_AT_1M = 0.3

    def find_best_match(new_obj, w_to_c_mat):
        best = None
        best_dist = CLUSTERING_DISTANCE_AT_1M
        MIN_DEPTH = 0.5

        local_pos = lambda p: (w_to_c_mat @ np.array(list(p) + [1]))[:3]

        for old in tracked_objects:
            if old.label != new_obj.label: continue

            # ignore depth difference in clustering
            loc_old = local_pos(old.position)
            loc_new = local_pos(new_obj.position)
            z = max([MIN_DEPTH, loc_old[2], loc_new[2]])
            dist = np.linalg.norm((loc_old - loc_new)[:2]) / z

            if dist < best_dist:
                best_dist = dist
                best = old
        # if best: print(f'matched with {best} (seen {best.n_detections} time(s))')
        return best

    def track(t, detections, view_mat):
        SCALE = 0.001 # output is millimeters
        MIN_DETECTIONS = 8
        DETECTION_WINDOW = 1.0
        MAX_UNSEEN_AGE = 8.0

        w_to_c_mat = np.linalg.inv(view_mat)

        for d in detections:
            p_local = np.array([
                d.spatialCoordinates.x * SCALE,
                -d.spatialCoordinates.y * SCALE, # note: flipped y
                d.spatialCoordinates.z * SCALE,
                1
            ])
            p_world = (view_mat @ p_local)[:3]
            try:
                label = LABEL_MAP[d.label]
            except:
                label = d.label

            # simple O(n^2)
            for o in tracked_objects:
                if o.label != label: continue
                dist = np.linalg.norm(o.position - p_world)

            if label in SELECTED_LABELS:
                new_obj = TrackedObject(t, p_world, label)
                existing = find_best_match(new_obj, w_to_c_mat)
                if existing:
                    existing.update(new_obj)
                else:
                    tracked_objects.append(new_obj)

        def should_remove(o):
            if o.n_detections < MIN_DETECTIONS and o.last_seen < t - DETECTION_WINDOW: return True
            if o.last_seen < t - MAX_UNSEEN_AGE: return True
            return False

        # remove cruft
        i = 0
        while i < len(tracked_objects):
            if should_remove(tracked_objects[i]):
                # print(f'removing ${o}')
                del tracked_objects[i]
            else:
                i += 1

        # print(tracked_objects)
        return [o for o in tracked_objects if o.n_detections >= MIN_DETECTIONS]

    return track

# Tiny yolo v3/4 label texts
LABEL_MAP = [
    "person",         "bicycle",    "car",           "motorbike",     "aeroplane",   "bus",           "train",
    "truck",          "boat",       "traffic light", "fire hydrant",  "stop sign",   "parking meter", "bench",
    "bird",           "cat",        "dog",           "horse",         "sheep",       "cow",           "elephant",
    "bear",           "zebra",      "giraffe",       "backpack",      "umbrella",    "handbag",       "tie",
    "suitcase",       "frisbee",    "skis",          "snowboard",     "sports ball", "kite",          "baseball bat",
    "baseball glove", "skateboard", "surfboard",     "tennis racket", "bottle",      "wine glass",    "cup",
    "fork",           "knife",      "spoon",         "bowl",          "banana",      "apple",         "sandwich",
    "orange",         "broccoli",   "carrot",        "hot dog",       "pizza",       "donut",         "cake",
    "chair",          "sofa",       "pottedplant",   "bed",           "diningtable", "toilet",        "tvmonitor",
    "laptop",         "mouse",      "remote",        "keyboard",      "cell phone",  "microwave",     "oven",
    "toaster",        "sink",       "refrigerator",  "book",          "clock",       "vase",          "scissors",
    "teddy bear",     "hair drier", "toothbrush"
]

SELECTED_LABELS = ['person']

if __name__ == '__main__':
    rclpy.init()
    nnBlobPath = 'models/yolo-v4-tiny-tf_openvino_2021.4_6shave.blob'
    if len(sys.argv) > 1:
        nnBlobPath = sys.argv[1]

    if not Path(nnBlobPath).exists():
        raise FileNotFoundError(f'Could not find {nnBlobPath}"')

    showRgb = True
    pipeline, vio_pipeline, spatialCalc = make_pipelines(nnBlobPath, showRgb)

    with dai.Device(pipeline) as device:

        detector_node = PersonDetectorNode()

        def main_loop():
            startTime = time.monotonic()
            counter = 0
            fps = 0
            color = (255, 255, 255)

            vio_session = vio_pipeline.startSession(device)
            tracker = make_tracker()

            # If I delete the following line the camera just throws this:
            # SpectacularAI WARN: IMU buffer max size exceeded, not waiting for more frames
            # and it freezes
            if showRgb: previewQueue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            
            detectionNNQueue = device.getOutputQueue(name="detections", maxSize=4, blocking=False)

            vio_matrix = None

            while True:
                if vio_session.hasOutput():
                    vio_out = vio_session.getOutput()
                    vio_matrix = vio_out.pose.asMatrix()
                elif detectionNNQueue.has():
                    
                    inDet = detectionNNQueue.get()

                    counter+=1
                    current_time = time.monotonic()
                    if (current_time - startTime) > 1 :
                        fps = counter / (current_time - startTime)
                        counter = 0
                        startTime = current_time


                    detections = inDet.detections

                    msg = Point()

                    if vio_matrix is not None:
                        detections_world = tracker(current_time, detections, vio_matrix)
                        
                        for detection in detections_world:
                             msg.x = detection.position[0] # distance from camera
                             msg.y = detection.position[1] # horizontal
                             msg.z = detection.position[2] # vertical

                             detector_node.publisher_.publish(msg)                        
                else:
                    time.sleep(0.005)

            vio_session.close()

        main_loop()
        
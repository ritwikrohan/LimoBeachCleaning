import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import json
import sys
import os
from ament_index_python.packages import get_package_share_directory

# Add the easy-yolov7 folder to the Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'easy-yolov7'))

from algorithm.object_detector import YOLOv7
from utils.detections import draw

easy_yolov7_dir = os.path.join(get_package_share_directory('yolo'), 'easy-yolov7')
weights = os.path.join(easy_yolov7_dir, 'coco.weights')
classes = os.path.join(easy_yolov7_dir, 'coco.yaml')


class YOLOv7DetectorSubscriber(Node):
    def __init__(self):
        super().__init__('yolov7_detector_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        self.yolov7 = YOLOv7()
        self.yolov7.load(weights, classes=classes, device='cpu')  # use 'gpu' for CUDA GPU inference

    def listener_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f'Error converting ROS Image message to OpenCV image: {e}')
            return
        frame = cv2.resize(frame, (640, 480))
        detections = self.yolov7.detect(frame)
        detected_frame = draw(frame, detections)
        #self.get_logger().info(json.dumps(detections, indent=4))

        resized_frame = cv2.resize(detected_frame, (640, 480))
        
        cv2.imshow('YOLOv7 Detection', detected_frame)
        cv2.waitKey(1)

    def __del__(self):
        self.yolov7.unload()

def main(args=None):
    rclpy.init(args=args)
    yolov7_detector_subscriber = YOLOv7DetectorSubscriber()
    rclpy.spin(yolov7_detector_subscriber)
    yolov7_detector_subscriber.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()


# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# from roboflow import Roboflow
# import cv2
# import threading
# import time

# class ImageSubscriber(Node):
#     def __init__(self):
#         super().__init__('image_subscriber')
#         self.subscription = self.create_subscription(
#             Image,
#             '/camera/color/image_raw',
#             self.listener_callback,
#             10)
#         self.bridge = CvBridge()
#         self.frame = None
#         self.lock = threading.Lock()
#         self.detect_thread = threading.Thread(target=self.detect_loop)
#         self.detect_thread.start()

#         # Initialize the Roboflow model
#         self.rf = Roboflow(api_key="0v5OjrYswOJZ0BCCmA6r")
#         self.project = self.rf.workspace().project("yolov7-trash-dataset-v5-05-04-2023")
#         self.model = self.project.version(1).model

#     def listener_callback(self, msg):
#         with self.lock:
#             self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

#     def detect_loop(self):
#         while rclpy.ok():
#             time.sleep(1)
#             with self.lock:
#                 if self.frame is not None:
#                     self.detect_from_frame(self.frame)

#     def detect_from_frame(self, frame):
#         cv2.imwrite("temp_frame.jpg", frame)  # Save the frame as a temporary image
#         result = self.model.predict("temp_frame.jpg", confidence=40, overlap=30).json()
#         labels = [item["class"] for item in result["predictions"]]
        
#         # Draw bounding boxes and labels on the frame
#         for prediction in result["predictions"]:
#             x1 = int(prediction["x"] - prediction["width"] / 2)
#             y1 = int(prediction["y"] - prediction["height"] / 2)
#             x2 = int(prediction["x"] + prediction["width"] / 2)
#             y2 = int(prediction["y"] + prediction["height"] / 2)
#             label = prediction["class"]
#             confidence = prediction["confidence"]
            
#             # Draw rectangle and label
#             cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
#             cv2.putText(frame, f"{label} ({confidence:.2f})", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
#         if labels:
#             print("Detected objects:", labels)
        
#         # Display the frame
#         cv2.imshow("Detected Frame", frame)
#         cv2.waitKey(1)  # Wait for a short period to display the frame

# def main(args=None):
#     rclpy.init(args=args)
#     image_subscriber = ImageSubscriber()
#     rclpy.spin(image_subscriber)
#     image_subscriber.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# from roboflow import Roboflow
# import cv2
# import threading
# import time
# from matplotlib import pyplot as plt

# class ImageSubscriber(Node):
#     def __init__(self):
#         super().__init__('image_subscriber')
#         self.subscription = self.create_subscription(
#             Image,
#             '/camera/color/image_raw',
#             self.listener_callback,
#             10)
#         self.bridge = CvBridge()
#         self.frame = None
#         self.lock = threading.Lock()
#         self.detect_thread = threading.Thread(target=self.detect_loop)
#         self.detect_thread.start()

#         # Initialize the Roboflow model
#         self.rf = Roboflow(api_key="0v5OjrYswOJZ0BCCmA6r")
#         self.project = self.rf.workspace().project("yolov7-trash-dataset-v5-05-04-2023")
#         self.model = self.project.version(1).model

#     def listener_callback(self, msg):
#         with self.lock:
#             self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

#     def detect_loop(self):
#         while rclpy.ok():
#             time.sleep(1)
#             with self.lock:
#                 if self.frame is not None:
#                     self.detect_from_frame(self.frame)

#     def detect_from_frame(self, frame):
#         cv2.imwrite("temp_frame.jpg", frame)  # Save the frame as a temporary image
#         result = self.model.predict("temp_frame.jpg", confidence=40, overlap=30).json()
#         labels = [item["class"] for item in result["predictions"]]
        
#         # Draw bounding boxes and labels on the frame
#         for prediction in result["predictions"]:
#             x1 = int(prediction["x"] - prediction["width"] / 2)
#             y1 = int(prediction["y"] - prediction["height"] / 2)
#             x2 = int(prediction["x"] + prediction["width"] / 2)
#             y2 = int(prediction["y"] + prediction["height"] / 2)
#             label = prediction["class"]
#             confidence = prediction["confidence"]
            
#             # Draw rectangle and label
#             cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
#             cv2.putText(frame, f"{label} ({confidence:.2f})", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
#         if labels:
#             print("Detected objects:", labels)
        
#         # Convert the frame from BGR to RGB for matplotlib
#         frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
#         # Display the frame using matplotlib
#         plt.imshow(frame_rgb)
#         plt.axis('off')  # Turn off axis numbers and ticks
#         plt.show()

# def main(args=None):
#     rclpy.init(args=args)
#     image_subscriber = ImageSubscriber()
#     rclpy.spin(image_subscriber)
#     image_subscriber.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

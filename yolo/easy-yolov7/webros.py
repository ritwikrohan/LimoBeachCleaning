import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import json
from algorithm.object_detector import YOLOv7
from utils.detections import draw

class YOLOv7Detector(Node):
    def __init__(self):
        super().__init__('yolov7_detector')
        self.publisher_ = self.create_publisher(Image, 'detected_frames', 10)
        self.bridge = CvBridge()
        self.yolov7 = YOLOv7()
        self.yolov7.load('coco.weights', classes='coco.yaml', device='cpu') # use 'gpu' for CUDA GPU inference

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('Error opening the webcam')

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            detections = self.yolov7.detect(frame)
            detected_frame = draw(frame, detections)
            self.get_logger().info(json.dumps(detections, indent=4))
            msg = self.bridge.cv2_to_imgmsg(detected_frame, encoding="bgr8")
            self.publisher_.publish(msg)
        else:
            self.get_logger().error('Failed to capture frame')

    def __del__(self):
        self.cap.release()
        self.get_logger().info('Webcam closed')
        self.yolov7.unload()

def main(args=None):
    rclpy.init(args=args)
    yolov7_detector = YOLOv7Detector()
    rclpy.spin(yolov7_detector)
    yolov7_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

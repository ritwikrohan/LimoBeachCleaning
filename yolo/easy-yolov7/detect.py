import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import json
import sys
import os
#from ament_index_python.packages import get_package_share_directory



from algorithm.object_detector import YOLOv7
from utils.detections import draw



#weights = '/home/jagadeesh/detect/src/yolo/easy-yolov7/coco.weights'
#classes = '/home/jagadeesh/detect/src/yolo/easy-yolov7/coco.yaml'

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
        self.yolov7.load('coco.weights', classes='coco.yaml', device='cpu')  # use 'gpu' for CUDA GPU inference

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

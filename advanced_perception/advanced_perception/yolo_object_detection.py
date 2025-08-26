from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from custom_msgs.msg import InferenceResult, Yolov8Inference
import os
bridge = CvBridge()

class YoloObjectDetection(Node):
    def __init__(self) -> None:
        super().__init__('object_detection')

        # Load a pre-trained YOLOv8 object detection model
        self.model = YOLO('/home/ritwik/asimovo_ws/src/advanced_perception/data/yolov8n.pt') 
        self.yolov8_inference = Yolov8Inference()

        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.camera_callback,
            10)

        self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1)
        self.img_pub = self.create_publisher(Image, "/inference_result", 1)

    def camera_callback(self, msg: Image) -> None:
        """ Performs object detection using the loaded YOLO model
            and processes the detection results. The image with 
            annotated detections is also published for visualization """

        img = bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.model(img) 

        self.yolov8_inference.header.frame_id = "inference"
        self.yolov8_inference.header.stamp = self.get_clock().now().to_msg()

        for r in results:
            boxes = r.boxes
            for box in boxes:
                self.inf_result = InferenceResult()
                # get box coordinates in (top, left, bottom, right) format
                b = box.xyxy[0].to('cpu').detach().numpy().copy()  
                c = box.cls
                self.inf_result.class_name = self.model.names[int(c)]
                self.inf_result.left = int(b[0])
                self.inf_result.top = int(b[1])
                self.inf_result.right = int(b[2])
                self.inf_result.bottom = int(b[3])
                self.inf_result.box_width = (self.inf_result.right - self.inf_result.left) 
                self.inf_result.box_height = (self.inf_result.bottom - self.inf_result.top)
                self.inf_result.x = self.inf_result.left + (self.inf_result.box_width/2.0)
                self.inf_result.y = self.inf_result.top + (self.inf_result.box_height/2.0)
                self.yolov8_inference.yolov8_inference.append(self.inf_result)

        annotated_frame = results[0].plot()
        img_msg = bridge.cv2_to_imgmsg(annotated_frame)  
        self.img_pub.publish(img_msg)

        self.yolov8_pub.publish(self.yolov8_inference)
        self.yolov8_inference.yolov8_inference.clear()

def main(args=None) -> None:
    rclpy.init(args=args)
    object_detection = YoloObjectDetection()
    rclpy.spin(object_detection)
    object_detection.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

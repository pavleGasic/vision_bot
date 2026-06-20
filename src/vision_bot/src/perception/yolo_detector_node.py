import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        
        self.declare_parameter('model', 'yolov8n.pt')
        self.declare_parameter('confidence', 0.5)
        self.declare_parameter('device', 'cpu')
        
        model_path = self.get_parameter('model').get_parameter_value().string_value
        self.confidence = self.get_parameter('confidence').get_parameter_value().double_value
        self.device = self.get_parameter('device').get_parameter_value().string_value
        
        self.model = YOLO(model_path)
        self.bridge = CvBridge()
        
        self.pub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.pub_detections = self.create_publisher(Detection2DArray, '/detections', 10)
        self.pub_annotated = self.create_publisher(Image, '/camera/image_annotated', 10)
        
        self.get_logger().info(f'YOLO detector ready - model: {model_path}, device: {self.device}')
        
    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(frame, conf=self.confidence, device=self.device, verbose=False)
        
        detection_array = Detection2DArray()
        detection_array.header = msg.header
        
        for result in results:
            for box in result.boxes:
                det = Detection2D()
                det.header = msg.header
                
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                det.bbox.center.position.x = (x1 + x2) / 2.0
                det.bbox.center.position.y = (y1 + y2) / 2.0
                det.bbox.size_x = x2 - x1
                det.bbox.size_y = y2 - y1
                
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = self.model.names[int(box.cls)]
                hyp.hypothesis.score = float(box.conf)
                det.results.append(hyp)
                
                detection_array.detections.append(det)
                
        
        self.pub_detections.publish(detection_array)
        
        annotated = results[0].plot()
        self.pub_annotated.publish(self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8'))
        
        
def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
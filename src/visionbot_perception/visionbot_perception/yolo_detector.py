import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose

from cv_bridge import CvBridge
from ultralytics import YOLO

QOS_SENSOR = QoSProfile(
  reliability=ReliabilityPolicy.BEST_EFFORT,
  history=HistoryPolicy.KEEP_LAST,
  depth=1
)

class YoloDetector(Node):
  def __init__(self):
    super().__init__('yolo_detector')

    self._declare_parameters()
    model_path = self.get_parameter('model_path').value
    self.conf_threshold = self.get_parameter('confidence_threshold').value
    self.input_size = self.get_parameter('input_size').value

    if not model_path:
      self.get_logger().fatal('model_path parameter is required')
      raise SystemExit(1)

    self.model = YOLO(model_path)
    self.bridge = CvBridge()

    self.pub_detections = self.create_publisher(Detection2DArray, '/detections', 10)
    self.pub_latency = self.create_publisher(Float32, '/inference_latency_ms', 10)

    self.create_subscription(Image, '/camera/image_raw', self._image_callback, QOS_SENSOR)

    self.get_logger().info(
      f'Yolo detector ready - model: {model_path}, '
      f'confidence: {self.conf_threshold}, size: {self.input_size}'
    )

  def _declare_parameters(self):
    self.declare_parameter('model_path', '')
    self.declare_parameter('confidence_threshold', 0.5)
    self.declare_parameter('input_size', 640)

  def _image_callback(self, msg: Image):
    frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    t0 = time.perf_counter()
    results = self.model(frame, imgsz=self.input_size, conf=self.conf_threshold, verbose=False)
    inference_ms = (time.perf_counter() - t0) * 1000.0

    detections = self._build_detections(results[0], msg.header)
    self.pub_detections.publish(detections)

    latency_msg = Float32()
    latency_msg.data = float(inference_ms)
    self.pub_latency.publish(latency_msg)

  def _build_detections(self, result, header) -> Detection2DArray:
    msg = Detection2DArray()
    msg.header = header

    for box in result.boxes:
      detection = Detection2D()
      detection.header = header

      x1, y1, x2, y2 = box.xyxy[0].tolist()
      detection.bbox.center.position.x = (x1 + x2) / 2.0
      detection.bbox.center.position.y = (y1 + y2) / 2.0
      detection.bbox.size_x = x2 - x1
      detection.bbox.size_y = y2 - y1

      hyp = ObjectHypothesisWithPose()
      hyp.hypothesis.class_id = result.names[int(box.cls[0])]
      hyp.hypothesis.score = float(box.conf[0])
      detection.results.append(hyp)

      msg.detections.append(detection)

    return msg


def main(args=None):
  rclpy.init(args=args)
  node = YoloDetector()
  try:
    rclpy.spin(node)
  finally:
    node.destroy_node(node)
    rclpy.shutdown()

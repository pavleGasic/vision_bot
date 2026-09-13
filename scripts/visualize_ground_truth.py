#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

OBJECTS = [
    ("person_standing",  3.42,  4.03),
    ("visitor_kid",      0.69, -1.80),
    ("couch",            0.90, -1.51),
    ("refrigerator",     8.86, -0.71),
    ("suitcase",        -3.37, -4.26),
    ("chair_office",    -7.54, -3.78),
    ("chairs_kitchen",   6.63,  0.94),
    ("chairs_balcony",  -0.02,  3.95),
    ("bicycle",          2.59,  4.94),
    ("backpack",        -8.80, -2.49),
    ("bed",             -6.02,  1.90),
    ("laptop",          -4.64,  1.65),
    ("oven",             8.79, -1.73),
    ("vase",            -0.17,  5.07),
    ("couch",           -4.73, -4.85),
]


class GroundTruthVisualizer(Node):
    def __init__(self):
        super().__init__("ground_truth_visualizer")
        self.pub = self.create_publisher(MarkerArray, "/ground_truth_markers", 10)
        self.create_timer(1.0, self.publish_markers)
        self.get_logger().info(f"Publishing {len(OBJECTS)} ground truth markers on /ground_truth_markers")

    def publish_markers(self):
        arr = MarkerArray()
        for i, (name, x, y) in enumerate(OBJECTS):
            arr.markers.append(self._cylinder(i, name, x, y))
            arr.markers.append(self._label(i, name, x, y))
        self.pub.publish(arr)

    def _cylinder(self, i, name, x, y):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "gt_objects"
        m.id = i
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.5
        m.pose.orientation.w = 1.0
        m.scale.x = 0.3
        m.scale.y = 0.3
        m.scale.z = 1.0
        m.color.r = 1.0
        m.color.g = 0.2
        m.color.b = 0.2
        m.color.a = 0.8
        return m

    def _label(self, i, name, x, y):
        t = Marker()
        t.header.frame_id = "map"
        t.header.stamp = self.get_clock().now().to_msg()
        t.ns = "gt_labels"
        t.id = i + 1000
        t.type = Marker.TEXT_VIEW_FACING
        t.action = Marker.ADD
        t.pose.position.x = x
        t.pose.position.y = y
        t.pose.position.z = 1.3
        t.pose.orientation.w = 1.0
        t.scale.z = 0.3
        t.color.r = 1.0
        t.color.g = 1.0
        t.color.b = 1.0
        t.color.a = 1.0
        t.text = name
        return t


def main():
    rclpy.init()
    rclpy.spin(GroundTruthVisualizer())
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Visibility debug visualizer.

Publishes:
  /visibility_markers  (visualization_msgs/MarkerArray)
    - Sphere per object: GREEN = visible, RED = not visible
    - Line from robot to object when visible
    - FOV wedge (two lines showing camera cone)
    - Text label above each object sphere

Subscribes:
  /odometry/filtered  (nav_msgs/Odometry)
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

FOV_HALF_RAD = 0.8901  # 51 degrees, matches metrics_logger

OBJECTS = [
    {"id": "person_standing", "x":  3.42, "y":  4.03, "max_range_m": 5.0},
    {"id": "visitor_kid",     "x":  0.69, "y": -1.80, "max_range_m": 3.0},
    {"id": "couch",           "x":  0.90, "y": -1.51, "max_range_m": 4.0},
    {"id": "refrigerator",    "x":  8.86, "y": -0.71, "max_range_m": 5.0},
    {"id": "suitcase",        "x": -3.37, "y": -4.26, "max_range_m": 3.5},
    {"id": "couch_bedroom",   "x": -4.73, "y": -4.85, "max_range_m": 4.0},
    {"id": "chair_office",    "x": -7.54, "y": -3.78, "max_range_m": 3.5},
    {"id": "chairs_kitchen",  "x":  6.63, "y":  0.94, "max_range_m": 4.0},
    {"id": "chairs_balcony",  "x": -0.02, "y":  3.95, "max_range_m": 3.5},
    {"id": "bicycle",         "x":  2.59, "y":  4.94, "max_range_m": 4.0},
    {"id": "backpack",        "x": -8.80, "y": -2.49, "max_range_m": 3.0},
    {"id": "bed",             "x": -6.02, "y":  1.90, "max_range_m": 4.0},
    {"id": "laptop",          "x": -4.64, "y":  1.65, "max_range_m": 2.5},
    {"id": "oven",            "x":  8.79, "y": -1.73, "max_range_m": 3.5},
    {"id": "vase",            "x": -0.17, "y":  5.07, "max_range_m": 2.5},
]


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def is_visible(robot_x, robot_y, robot_yaw, obj):
    dx = obj["x"] - robot_x
    dy = obj["y"] - robot_y
    dist = math.hypot(dx, dy)
    if dist > obj["max_range_m"]:
        return False, dist
    angle_to_obj = math.atan2(dy, dx)
    angle_diff = angle_to_obj - robot_yaw
    while angle_diff >  math.pi: angle_diff -= 2 * math.pi
    while angle_diff < -math.pi: angle_diff += 2 * math.pi
    return abs(angle_diff) <= FOV_HALF_RAD, dist


class VisibilityVisualizer(Node):
    def __init__(self):
        super().__init__("visibility_visualizer")
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(Odometry, "/odometry/filtered", self.odom_cb, qos)
        self.pub = self.create_publisher(MarkerArray, "/visibility_markers", 10)
        self.get_logger().info("Visibility visualizer running")

    def odom_cb(self, msg):
        rx = msg.pose.pose.position.x
        ry = msg.pose.pose.position.y
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)

        markers = MarkerArray()
        mid = 0

        # --- FOV wedge lines ---
        fov_len = 6.0
        for sign in (+1, -1):
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "fov"
            m.id = mid; mid += 1
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.03
            m.color.r = 1.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 0.6
            edge_angle = yaw + sign * FOV_HALF_RAD
            p0 = Point(); p0.x = rx; p0.y = ry; p0.z = 0.3
            p1 = Point()
            p1.x = rx + fov_len * math.cos(edge_angle)
            p1.y = ry + fov_len * math.sin(edge_angle)
            p1.z = 0.3
            m.points = [p0, p1]
            markers.markers.append(m)

        # --- Per-object markers ---
        for obj in OBJECTS:
            visible, dist = is_visible(rx, ry, yaw, obj)

            # Sphere
            s = Marker()
            s.header.frame_id = "map"
            s.header.stamp = self.get_clock().now().to_msg()
            s.ns = "objects"
            s.id = mid; mid += 1
            s.type = Marker.SPHERE
            s.action = Marker.ADD
            s.pose.position.x = obj["x"]
            s.pose.position.y = obj["y"]
            s.pose.position.z = 0.5
            s.pose.orientation.w = 1.0
            s.scale.x = s.scale.y = s.scale.z = 0.4
            if visible:
                s.color.r = 0.0; s.color.g = 1.0; s.color.b = 0.0; s.color.a = 0.9
            else:
                s.color.r = 1.0; s.color.g = 0.0; s.color.b = 0.0; s.color.a = 0.5
            markers.markers.append(s)

            # Text label
            t = Marker()
            t.header.frame_id = "map"
            t.header.stamp = self.get_clock().now().to_msg()
            t.ns = "labels"
            t.id = mid; mid += 1
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = obj["x"]
            t.pose.position.y = obj["y"]
            t.pose.position.z = 1.1
            t.pose.orientation.w = 1.0
            t.scale.z = 0.25
            t.color.r = t.color.g = t.color.b = 1.0; t.color.a = 1.0
            status = f"VISIBLE  d={dist:.1f}m" if visible else f"d={dist:.1f}m"
            t.text = f"{obj['id']}\n{status}"
            markers.markers.append(t)

            # Line robot→object when visible
            if visible:
                l = Marker()
                l.header.frame_id = "map"
                l.header.stamp = self.get_clock().now().to_msg()
                l.ns = "lines"
                l.id = mid; mid += 1
                l.type = Marker.LINE_STRIP
                l.action = Marker.ADD
                l.scale.x = 0.05
                l.color.r = 0.0; l.color.g = 1.0; l.color.b = 0.4; l.color.a = 0.8
                p0 = Point(); p0.x = rx; p0.y = ry; p0.z = 0.3
                p1 = Point(); p1.x = obj["x"]; p1.y = obj["y"]; p1.z = 0.5
                l.points = [p0, p1]
                markers.markers.append(l)

        self.pub.publish(markers)


def main():
    rclpy.init()
    rclpy.spin(VisibilityVisualizer())
    rclpy.shutdown()


if __name__ == "__main__":
    main()

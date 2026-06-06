"""Publish LaserScan rays as RViz line markers."""

import math

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from rclpy.duration import Duration as RclpyDuration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
import tf2_ros
from visualization_msgs.msg import Marker


class ScanBeamMarkerNode(Node):
    """Convert LaserScan ranges into LINE_LIST markers for RViz."""

    def __init__(self):
        super().__init__('scan_beam_marker_node')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('marker_topic', '/scan_beams')
        self.declare_parameter('source_frame', '')
        self.declare_parameter('target_frame', '')
        self.declare_parameter('max_beams', 360)
        self.declare_parameter('line_width', 0.035)
        self.declare_parameter('alpha', 0.85)
        self.declare_parameter('lifetime_sec', 1.0)

        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.marker_topic = self.get_parameter('marker_topic').get_parameter_value().string_value
        self.source_frame = self.get_parameter('source_frame').get_parameter_value().string_value
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self.max_beams = max(
            1, self.get_parameter('max_beams').get_parameter_value().integer_value)
        self.line_width = self.get_parameter('line_width').get_parameter_value().double_value
        self.alpha = self.get_parameter('alpha').get_parameter_value().double_value
        self.lifetime_sec = self.get_parameter('lifetime_sec').get_parameter_value().double_value
        self._last_tf_warning_time = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.publisher = self.create_publisher(Marker, self.marker_topic, 10)
        self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, qos_profile_sensor_data)

        self.get_logger().info(
            f'Publishing scan beams from {self.scan_topic} to {self.marker_topic}')

    def scan_callback(self, scan):
        source_frame = self.source_frame or scan.header.frame_id
        target_frame = self.target_frame or source_frame
        transform = None

        if target_frame != source_frame:
            try:
                transform = self.tf_buffer.lookup_transform(
                    target_frame, source_frame, Time(),
                    timeout=RclpyDuration(seconds=0.05))
            except Exception as exc:
                self._warn_tf_failure(source_frame, target_frame, exc)
                target_frame = source_frame
                transform = None

        marker = Marker()
        marker.header.frame_id = target_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.frame_locked = True
        marker.ns = 'scan_beams'
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = self.line_width
        marker.color.r = 0.25
        marker.color.g = 0.75
        marker.color.b = 1.0
        marker.color.a = self.alpha
        marker.lifetime = Duration(
            sec=int(self.lifetime_sec),
            nanosec=int((self.lifetime_sec % 1.0) * 1e9),
        )

        ranges = list(scan.ranges)
        step = max(1, math.ceil(len(ranges) / self.max_beams))
        origin = self.transform_point(Point(), transform)

        for index in range(0, len(ranges), step):
            distance = ranges[index]
            if not math.isfinite(distance):
                continue
            if distance < scan.range_min or distance > scan.range_max:
                continue

            angle = scan.angle_min + index * scan.angle_increment
            endpoint = Point()
            endpoint.x = distance * math.cos(angle)
            endpoint.y = distance * math.sin(angle)
            endpoint.z = 0.0

            marker.points.append(Point(x=origin.x, y=origin.y, z=origin.z))
            marker.points.append(self.transform_point(endpoint, transform))

        self.publisher.publish(marker)

    def transform_point(self, point, transform):
        if transform is None:
            return point

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        x = point.x
        y = point.y
        z = point.z
        qx = rotation.x
        qy = rotation.y
        qz = rotation.z
        qw = rotation.w

        result = Point()
        result.x = (
            (1.0 - 2.0 * (qy * qy + qz * qz)) * x
            + 2.0 * (qx * qy - qz * qw) * y
            + 2.0 * (qx * qz + qy * qw) * z
            + translation.x
        )
        result.y = (
            2.0 * (qx * qy + qz * qw) * x
            + (1.0 - 2.0 * (qx * qx + qz * qz)) * y
            + 2.0 * (qy * qz - qx * qw) * z
            + translation.y
        )
        result.z = (
            2.0 * (qx * qz - qy * qw) * x
            + 2.0 * (qy * qz + qx * qw) * y
            + (1.0 - 2.0 * (qx * qx + qy * qy)) * z
            + translation.z
        )
        return result

    def _warn_tf_failure(self, source_frame, target_frame, exc):
        now = self.get_clock().now()
        if (
            self._last_tf_warning_time is None
            or (now - self._last_tf_warning_time).nanoseconds > 2_000_000_000
        ):
            self.get_logger().warn(
                f'Cannot publish scan beams: missing TF {source_frame} -> {target_frame}: {exc}')
            self._last_tf_warning_time = now


def main(args=None):
    rclpy.init(args=args)
    node = ScanBeamMarkerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

# Copyright 2026 wanjunkim
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Project a tilted 2D LaserScan into a TF-corrected PointCloud2."""

from laser_geometry import LaserProjection
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener

try:
    from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
except ImportError:
    from tf2_sensor_msgs import do_transform_cloud


class TiltedScanNode(Node):
    """Convert /scan into /tilted_pointcloud using the current TF tree."""

    def __init__(self):
        """Initialize subscriptions, publisher, projection, and TF buffer."""
        super().__init__('tilted_scan_node')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('pointcloud_topic', '/tilted_pointcloud')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('source_frame', '')
        self.declare_parameter('use_latest_tf', False)
        self.declare_parameter('filter_ground', True)
        self.declare_parameter('ground_min_z', 0.05)
        self.declare_parameter('accumulate_cloud', True)
        self.declare_parameter('max_accumulated_points', 200000)
        self.declare_parameter('tf_timeout_sec', 0.1)

        self._scan_topic = (
            self.get_parameter('scan_topic').get_parameter_value().string_value
        )
        self._pointcloud_topic = (
            self.get_parameter('pointcloud_topic')
            .get_parameter_value()
            .string_value
        )
        self._target_frame = (
            self.get_parameter('target_frame').get_parameter_value().string_value
        )
        self._source_frame = (
            self.get_parameter('source_frame').get_parameter_value().string_value
        )
        self._use_latest_tf = (
            self.get_parameter('use_latest_tf').get_parameter_value().bool_value
        )
        self._filter_ground = (
            self.get_parameter('filter_ground').get_parameter_value().bool_value
        )
        self._ground_min_z = (
            self.get_parameter('ground_min_z').get_parameter_value().double_value
        )
        self._accumulate_cloud = (
            self.get_parameter('accumulate_cloud').get_parameter_value().bool_value
        )
        self._max_accumulated_points = max(
            1,
            self.get_parameter('max_accumulated_points')
            .get_parameter_value()
            .integer_value,
        )
        self._tf_timeout = Duration(
            seconds=self.get_parameter('tf_timeout_sec').value
        )

        self._projector = LaserProjection()
        self._accumulated_points = []
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._last_tf_warning_ns = 0
        self._warning_period_ns = 5_000_000_000

        self._publisher = self.create_publisher(
            PointCloud2,
            self._pointcloud_topic,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
        )
        self._subscription = self.create_subscription(
            LaserScan,
            self._scan_topic,
            self._scan_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            'Projecting scans from %s to %s in frame %s'
            % (self._scan_topic, self._pointcloud_topic, self._target_frame)
        )

    def _scan_callback(self, scan_msg):
        """Convert a LaserScan message to a transformed PointCloud2."""
        if not scan_msg.header.frame_id:
            self.get_logger().warn('Received LaserScan without a frame_id')
            return

        cloud_msg = self._projector.projectLaser(scan_msg)
        source_frame = self._source_frame or scan_msg.header.frame_id
        cloud_msg.header.frame_id = source_frame

        if source_frame == self._target_frame:
            transformed_cloud = cloud_msg
        else:
            try:
                lookup_time = rclpy.time.Time()
                if not self._use_latest_tf:
                    lookup_time = rclpy.time.Time.from_msg(scan_msg.header.stamp)

                transform = self._tf_buffer.lookup_transform(
                    self._target_frame,
                    source_frame,
                    lookup_time,
                    timeout=self._tf_timeout,
                )
            except TransformException as exc:
                self._warn_tf_failure(source_frame, exc)
                return

            transformed_cloud = do_transform_cloud(cloud_msg, transform)

        if self._use_latest_tf:
            transformed_cloud.header.stamp = self.get_clock().now().to_msg()
        else:
            transformed_cloud.header.stamp = scan_msg.header.stamp
        transformed_cloud.header.frame_id = self._target_frame
        transformed_cloud = self._filter_ground_points(transformed_cloud)
        transformed_cloud = self._accumulate_points(transformed_cloud)
        self._publisher.publish(transformed_cloud)

    def _filter_ground_points(self, cloud_msg):
        """Remove points at or below the configured ground height."""
        if not self._filter_ground:
            return cloud_msg

        field_names = [field.name for field in cloud_msg.fields]
        if 'z' not in field_names:
            self.get_logger().warn('Cannot filter ground: cloud has no z field')
            return cloud_msg

        points = point_cloud2.read_points(cloud_msg, skip_nans=True)
        filtered_points = [
            tuple(point)
            for point in points
            if point['z'] > self._ground_min_z
        ]

        filtered_cloud = point_cloud2.create_cloud(
            cloud_msg.header,
            cloud_msg.fields,
            filtered_points,
        )
        filtered_cloud.is_dense = True
        return filtered_cloud

    def _accumulate_points(self, cloud_msg):
        """Keep previous transformed points fixed in the target frame."""
        if not self._accumulate_cloud:
            return cloud_msg

        new_points = [
            tuple(point)
            for point in point_cloud2.read_points(cloud_msg, skip_nans=True)
        ]
        if not new_points:
            return cloud_msg

        self._accumulated_points.extend(new_points)
        if len(self._accumulated_points) > self._max_accumulated_points:
            self._accumulated_points = self._accumulated_points[
                -self._max_accumulated_points:
            ]

        accumulated_cloud = point_cloud2.create_cloud(
            cloud_msg.header,
            cloud_msg.fields,
            self._accumulated_points,
        )
        accumulated_cloud.is_dense = True
        return accumulated_cloud

    def _warn_tf_failure(self, source_frame, exc):
        """Throttle TF lookup warnings so missing startup TF stays readable."""
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_tf_warning_ns < self._warning_period_ns:
            return

        self._last_tf_warning_ns = now_ns
        self.get_logger().warn(
            'Waiting for TF %s -> %s: %s'
            % (source_frame, self._target_frame, exc)
        )


def main(args=None):
    """Run the tilted scan projection node."""
    rclpy.init(args=args)
    node = TiltedScanNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

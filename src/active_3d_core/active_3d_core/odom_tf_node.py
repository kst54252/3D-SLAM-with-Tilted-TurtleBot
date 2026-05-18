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

"""Broadcast odom to base_footprint from Gazebo odometry."""

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster


class OdomTfNode(Node):
    """Convert nav_msgs/Odometry pose messages into an odom TF."""

    def __init__(self):
        """Initialize odometry subscription and TF broadcaster."""
        super().__init__('odom_tf_node')

        self.declare_parameter('odom_topic', '/model/tilted_turtlebot/odometry')
        self.declare_parameter('parent_frame', 'odom')
        self.declare_parameter('child_frame', 'base_footprint')
        self.declare_parameter('stamp_with_current_time', True)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_z', 0.0)

        self._odom_topic = (
            self.get_parameter('odom_topic').get_parameter_value().string_value
        )
        self._parent_frame = (
            self.get_parameter('parent_frame').get_parameter_value().string_value
        )
        self._child_frame = (
            self.get_parameter('child_frame').get_parameter_value().string_value
        )
        self._stamp_with_current_time = (
            self.get_parameter('stamp_with_current_time')
            .get_parameter_value()
            .bool_value
        )
        self._initial_x = (
            self.get_parameter('initial_x').get_parameter_value().double_value
        )
        self._initial_y = (
            self.get_parameter('initial_y').get_parameter_value().double_value
        )
        self._initial_z = (
            self.get_parameter('initial_z').get_parameter_value().double_value
        )
        self._x = self._initial_x
        self._y = self._initial_y
        self._z = self._initial_z
        self._rotation = Quaternion()
        self._rotation.w = 1.0
        self._latest_stamp = None

        self._tf_broadcaster = TransformBroadcaster(self)
        self._subscription = self.create_subscription(
            Odometry,
            self._odom_topic,
            self._odom_callback,
            qos_profile_sensor_data,
        )
        self._timer = self.create_timer(0.05, self._publish_transform)

        self.get_logger().info(
            'Broadcasting TF %s -> %s from %s'
            % (self._parent_frame, self._child_frame, self._odom_topic)
        )

    def _odom_callback(self, msg):
        """Update the latest odometry pose used by the TF timer."""
        self._x = self._initial_x + msg.pose.pose.position.x
        self._y = self._initial_y + msg.pose.pose.position.y
        self._z = self._initial_z + msg.pose.pose.position.z
        self._rotation = msg.pose.pose.orientation
        self._latest_stamp = msg.header.stamp

    def _publish_transform(self):
        """Publish the latest pose as a TransformStamped."""
        transform = TransformStamped()
        if self._stamp_with_current_time or self._latest_stamp is None:
            transform.header.stamp = self.get_clock().now().to_msg()
        else:
            transform.header.stamp = self._latest_stamp
        transform.header.frame_id = self._parent_frame
        transform.child_frame_id = self._child_frame
        transform.transform.translation.x = self._x
        transform.transform.translation.y = self._y
        transform.transform.translation.z = self._z
        transform.transform.rotation = self._rotation

        self._tf_broadcaster.sendTransform(transform)


def main(args=None):
    """Run the odometry TF bridge node."""
    rclpy.init(args=args)
    node = OdomTfNode()

    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

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

"""Keyboard teleop with an open-loop odom TF for the tilted lidar demo."""

import math
import select
import termios
import time
import tty

from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class KeyboardDriveNode(Node):
    """Drive the robot with WASD and trigger a one-turn scan with r."""

    def __init__(self):
        """Initialize keyboard input, command publisher, and TF broadcaster."""
        super().__init__('keyboard_drive_node')

        self.declare_parameter('cmd_vel_topic', '/model/tilted_turtlebot/cmd_vel')
        self.declare_parameter('linear_speed', 0.16)
        self.declare_parameter('angular_speed', 0.7)
        self.declare_parameter('spin_angular_speed', 0.45)
        self.declare_parameter('key_timeout', 0.3)
        self.declare_parameter('publish_cmd_vel', True)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('parent_frame', 'odom')
        self.declare_parameter('child_frame', 'base_footprint')
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_z', 0.0)
        self.declare_parameter('initial_yaw', 0.0)

        self._cmd_vel_topic = (
            self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        )
        self._linear_speed = (
            self.get_parameter('linear_speed').get_parameter_value().double_value
        )
        self._angular_speed = (
            self.get_parameter('angular_speed').get_parameter_value().double_value
        )
        self._spin_angular_speed = abs(
            self.get_parameter('spin_angular_speed')
            .get_parameter_value()
            .double_value
        )
        self._key_timeout = max(
            0.05,
            self.get_parameter('key_timeout').get_parameter_value().double_value,
        )
        self._publish_cmd_vel = (
            self.get_parameter('publish_cmd_vel').get_parameter_value().bool_value
        )
        self._publish_tf = (
            self.get_parameter('publish_tf').get_parameter_value().bool_value
        )
        self._parent_frame = (
            self.get_parameter('parent_frame').get_parameter_value().string_value
        )
        self._child_frame = (
            self.get_parameter('child_frame').get_parameter_value().string_value
        )

        self._x = (
            self.get_parameter('initial_x').get_parameter_value().double_value
        )
        self._y = (
            self.get_parameter('initial_y').get_parameter_value().double_value
        )
        self._z = (
            self.get_parameter('initial_z').get_parameter_value().double_value
        )
        self._yaw = (
            self.get_parameter('initial_yaw').get_parameter_value().double_value
        )

        self._linear_cmd = 0.0
        self._angular_cmd = 0.0
        self._last_key_time = 0.0
        self._last_update_time = time.monotonic()
        self._spin_remaining = 0.0
        self._terminal = None
        self._terminal_settings = None

        self._publisher = self.create_publisher(Twist, self._cmd_vel_topic, 10)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._setup_keyboard()
        self._timer = self.create_timer(0.05, self._timer_callback)

        self.get_logger().info(
            'WASD drive, space stop, r one-turn scan, Ctrl-C quit'
        )

    def _setup_keyboard(self):
        """Put the controlling terminal into cbreak mode when available."""
        try:
            self._terminal = open('/dev/tty', 'r')
            self._terminal_settings = termios.tcgetattr(self._terminal)
            tty.setcbreak(self._terminal.fileno())
        except OSError:
            self._terminal = None
            self.get_logger().warn(
                'No controlling terminal. Run this node in a terminal for WASD.'
            )

    def _timer_callback(self):
        """Read keys, integrate the pose, and publish command plus TF."""
        now = time.monotonic()
        dt = now - self._last_update_time
        self._last_update_time = now

        self._integrate_pose(dt)
        self._read_keys(now)
        self._update_auto_spin(dt)
        self._apply_key_timeout(now)
        self._publish_velocity()
        self._publish_open_loop_tf()

    def _read_keys(self, now):
        """Consume pending keyboard input without blocking the ROS timer."""
        if self._terminal is None:
            return

        while True:
            readable, _, _ = select.select([self._terminal], [], [], 0.0)
            if not readable:
                return
            key = self._terminal.read(1).lower()
            self._handle_key(key, now)

    def _handle_key(self, key, now):
        """Map a keyboard character to a velocity command."""
        if key == 'r':
            self._spin_remaining = 2.0 * math.pi
            self._linear_cmd = 0.0
            self._angular_cmd = self._spin_angular_speed
            self.get_logger().info('Starting one-turn scan')
            return

        if key in (' ', 'x'):
            self._spin_remaining = 0.0
            self._linear_cmd = 0.0
            self._angular_cmd = 0.0
            self._last_key_time = now
            return

        if key == 'w':
            self._set_manual_command(now, self._linear_speed, 0.0)
        elif key == 's':
            self._set_manual_command(now, -self._linear_speed, 0.0)
        elif key == 'a':
            self._set_manual_command(now, 0.0, self._angular_speed)
        elif key == 'd':
            self._set_manual_command(now, 0.0, -self._angular_speed)

    def _set_manual_command(self, now, linear, angular):
        """Cancel auto-spin and use the requested manual command."""
        self._spin_remaining = 0.0
        self._linear_cmd = linear
        self._angular_cmd = angular
        self._last_key_time = now

    def _update_auto_spin(self, dt):
        """Stop the r-triggered scan after one full revolution."""
        if self._spin_remaining <= 0.0:
            return

        self._spin_remaining -= abs(self._spin_angular_speed) * dt
        self._linear_cmd = 0.0
        self._angular_cmd = self._spin_angular_speed
        if self._spin_remaining <= 0.0:
            self._linear_cmd = 0.0
            self._angular_cmd = 0.0
            self.get_logger().info('Completed one-turn scan')

    def _apply_key_timeout(self, now):
        """Stop manual motion when a WASD key is no longer repeating."""
        if self._spin_remaining > 0.0:
            return
        if self._last_key_time <= 0.0:
            return
        if now - self._last_key_time <= self._key_timeout:
            return

        self._linear_cmd = 0.0
        self._angular_cmd = 0.0

    def _integrate_pose(self, dt):
        """Integrate the previous command into the open-loop odom pose."""
        if dt <= 0.0:
            return

        self._x += self._linear_cmd * dt * math.cos(self._yaw)
        self._y += self._linear_cmd * dt * math.sin(self._yaw)
        self._yaw += self._angular_cmd * dt

    def _publish_velocity(self):
        """Publish the current velocity command to Gazebo."""
        if not self._publish_cmd_vel:
            return

        command = Twist()
        command.linear.x = self._linear_cmd
        command.angular.z = self._angular_cmd
        self._publisher.publish(command)

    def _publish_open_loop_tf(self):
        """Publish odom to base_footprint for RViz point-cloud accumulation."""
        if not self._publish_tf:
            return

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self._parent_frame
        transform.child_frame_id = self._child_frame
        transform.transform.translation.x = self._x
        transform.transform.translation.y = self._y
        transform.transform.translation.z = self._z
        transform.transform.rotation.z = math.sin(self._yaw * 0.5)
        transform.transform.rotation.w = math.cos(self._yaw * 0.5)

        self._tf_broadcaster.sendTransform(transform)

    def destroy_node(self):
        """Restore terminal settings before shutting down."""
        if self._terminal is not None:
            termios.tcsetattr(
                self._terminal,
                termios.TCSADRAIN,
                self._terminal_settings,
            )
            self._terminal.close()
            self._terminal = None
        super().destroy_node()


def main(args=None):
    """Run the keyboard drive node."""
    rclpy.init(args=args)
    node = KeyboardDriveNode()

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

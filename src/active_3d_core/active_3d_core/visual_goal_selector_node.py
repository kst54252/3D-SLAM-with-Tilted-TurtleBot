"""Select a frontier goal with camera snapshots and a local vision LLM."""

import base64
import json
import math
from pathlib import Path
import threading
import time
import urllib.error
import urllib.request

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_msgs.msg import String
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class VisualGoalSelectorNode(Node):
    """Take candidate-facing images and ask a local LLM to choose one."""

    def __init__(self):
        super().__init__('visual_goal_selector_node')

        self.declare_parameter('candidate_pose_topic', '/frontier_candidate_poses')
        self.declare_parameter('scan_ready_topic', '/frontier_scan_ready')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('selected_goal_topic', '/visual_selected_frontier_goal')
        self.declare_parameter('llm_request_debug_topic', '/llm_request_debug')
        self.declare_parameter('llm_response_debug_topic', '/llm_response_debug')
        self.declare_parameter('llm_exchange_debug_topic', '/llm_exchange_debug')
        self.declare_parameter('llm_exchange_debug_chunk_topic', '/llm_exchange_debug_chunk')
        self.declare_parameter('cmd_vel_topic', '/model/tilted_turtlebot/cmd_vel')
        self.declare_parameter('fixed_frame', 'odom')
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('max_candidates', 5)
        self.declare_parameter('min_candidate_distance', 0.65)
        self.declare_parameter('turn_angular_speed', 0.35)
        self.declare_parameter('yaw_tolerance', 0.12)
        self.declare_parameter('image_settle_sec', 0.4)
        self.declare_parameter('candidate_timeout_sec', 5.0)
        self.declare_parameter('selection_cooldown_sec', 4.0)
        self.declare_parameter('capture_only', False)
        self.declare_parameter('capture_once', False)
        self.declare_parameter('call_llm_api', True)
        self.declare_parameter('require_llm_response', True)
        self.declare_parameter('disable_llm_reasoning', True)
        self.declare_parameter('debug_image_dir', '/tmp/active_3d_visual_goal_selector')
        self.declare_parameter('llm_api_url', 'http://localhost:11434/api/chat')
        self.declare_parameter('llm_model', 'gemma4:26b')
        self.declare_parameter('llm_request_timeout_sec', 20.0)
        self.declare_parameter('llm_debug_chunk_size', 2500)

        self.candidate_pose_topic = self.get_parameter(
            'candidate_pose_topic').get_parameter_value().string_value
        self.scan_ready_topic = self.get_parameter(
            'scan_ready_topic').get_parameter_value().string_value
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.selected_goal_topic = self.get_parameter(
            'selected_goal_topic').get_parameter_value().string_value
        self.llm_request_debug_topic = self.get_parameter(
            'llm_request_debug_topic').get_parameter_value().string_value
        self.llm_response_debug_topic = self.get_parameter(
            'llm_response_debug_topic').get_parameter_value().string_value
        self.llm_exchange_debug_topic = self.get_parameter(
            'llm_exchange_debug_topic').get_parameter_value().string_value
        self.llm_exchange_debug_chunk_topic = self.get_parameter(
            'llm_exchange_debug_chunk_topic').get_parameter_value().string_value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.fixed_frame = self.get_parameter('fixed_frame').get_parameter_value().string_value
        self.robot_frame = self.get_parameter('robot_frame').get_parameter_value().string_value
        self.max_candidates = max(
            1, self.get_parameter('max_candidates').get_parameter_value().integer_value)
        self.min_candidate_distance = max(
            0.0,
            self.get_parameter('min_candidate_distance').get_parameter_value().double_value)
        self.turn_angular_speed = abs(
            self.get_parameter('turn_angular_speed').get_parameter_value().double_value)
        self.yaw_tolerance = abs(
            self.get_parameter('yaw_tolerance').get_parameter_value().double_value)
        self.image_settle_sec = self.get_parameter(
            'image_settle_sec').get_parameter_value().double_value
        self.candidate_timeout_sec = self.get_parameter(
            'candidate_timeout_sec').get_parameter_value().double_value
        self.selection_cooldown_sec = self.get_parameter(
            'selection_cooldown_sec').get_parameter_value().double_value
        self.capture_only = self.get_parameter('capture_only').get_parameter_value().bool_value
        self.capture_once = self.get_parameter('capture_once').get_parameter_value().bool_value
        self.call_llm_api = self.get_parameter('call_llm_api').get_parameter_value().bool_value
        self.require_llm_response = self.get_parameter(
            'require_llm_response').get_parameter_value().bool_value
        self.disable_llm_reasoning = self.get_parameter(
            'disable_llm_reasoning').get_parameter_value().bool_value
        self.debug_image_dir = Path(
            self.get_parameter('debug_image_dir').get_parameter_value().string_value)
        self.llm_api_url = self.get_parameter('llm_api_url').get_parameter_value().string_value
        self.llm_model = self.get_parameter('llm_model').get_parameter_value().string_value
        self.llm_request_timeout_sec = self.get_parameter(
            'llm_request_timeout_sec').get_parameter_value().double_value
        self.llm_debug_chunk_size = max(
            200,
            self.get_parameter('llm_debug_chunk_size').get_parameter_value().integer_value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_image = None
        self.latest_candidates = None
        self.latest_image_lock = threading.Lock()
        self.selection_lock = threading.Lock()
        self.scan_ready = False
        self.selection_running = False
        self.captured_once = False
        self.captured_candidate_count = 0
        self.last_selection_time = 0.0

        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.selected_goal_pub = self.create_publisher(
            PoseStamped, self.selected_goal_topic, 1)
        debug_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.llm_request_debug_pub = self.create_publisher(
            String, self.llm_request_debug_topic, debug_qos)
        self.llm_response_debug_pub = self.create_publisher(
            String, self.llm_response_debug_topic, debug_qos)
        self.llm_exchange_debug_pub = self.create_publisher(
            String, self.llm_exchange_debug_topic, debug_qos)
        self.llm_exchange_debug_chunk_pub = self.create_publisher(
            String, self.llm_exchange_debug_chunk_topic, debug_qos)
        self.create_subscription(
            PoseArray, self.candidate_pose_topic, self.candidates_callback, 1)
        self.create_subscription(
            Bool, self.scan_ready_topic, self.scan_ready_callback, 1)
        self.create_subscription(
            Image, self.image_topic, self.image_callback, qos_profile_sensor_data)

        self.get_logger().info(
            f'Visual goal selector ready: candidates={self.candidate_pose_topic}, '
            f'scan_ready={self.scan_ready_topic}, image={self.image_topic}, '
            f'capture_only={self.capture_only}, '
            f'call_llm_api={self.call_llm_api}, require_llm_response={self.require_llm_response}, '
            f'disable_llm_reasoning={self.disable_llm_reasoning}, '
            f'api={self.llm_api_url}')

    def image_callback(self, msg):
        with self.latest_image_lock:
            self.latest_image = msg

    def candidates_callback(self, msg):
        if not msg.poses:
            return

        self.latest_candidates = msg
        if not self.scan_ready:
            self.get_logger().info(
                f'Received {len(msg.poses)} candidates, waiting for scan-ready signal.')
            return

        self.start_capture_if_ready(msg)

    def scan_ready_callback(self, msg):
        previous = self.scan_ready
        self.scan_ready = msg.data
        if not self.scan_ready:
            self.get_logger().info('Scan rotation started; candidate image capture paused.')
            with self.selection_lock:
                self.captured_once = False
                self.captured_candidate_count = 0
            return
        if not previous:
            self.get_logger().info('Scan rotation complete; candidate image capture enabled.')
        if self.latest_candidates is not None:
            self.start_capture_if_ready(self.latest_candidates)

    def start_capture_if_ready(self, msg):
        robot_pose = self.lookup_robot_pose()
        poses = self.filter_near_candidates(msg.poses, robot_pose)
        poses = poses[:self.max_candidates]
        if not poses:
            self.get_logger().warn(
                f'No candidates remain after filtering goals closer than '
                f'{self.min_candidate_distance:.2f} m.')
            return
        candidate_count = len(poses)
        now = time.monotonic()
        with self.selection_lock:
            if self.selection_running:
                return
            if (
                self.capture_once and
                self.captured_once and
                candidate_count <= self.captured_candidate_count
            ):
                return
            if self.capture_once and self.captured_once:
                return
            if now - self.last_selection_time < self.selection_cooldown_sec:
                return
            self.selection_running = True

        self.get_logger().info(
            f'Received {len(msg.poses)} candidates; capturing {len(poses)} after distance filtering.')
        header = msg.header
        thread = threading.Thread(
            target=self.select_goal_worker,
            args=(header, poses),
            daemon=True)
        thread.start()

    def select_goal_worker(self, header, poses):
        try:
            self.get_logger().info(f'Capturing images for {len(poses)} frontier candidates.')
            robot_pose = self.lookup_robot_pose()
            snapshots = []
            for index, pose in enumerate(poses):
                self.get_logger().info(
                    f'Capturing candidate {index + 1}/{len(poses)}: '
                    f'x={pose.position.x:.2f}, y={pose.position.y:.2f}')
                image = self.capture_candidate_image(index, pose)
                if image is not None:
                    snapshots.append((index, pose, image))
                else:
                    self.get_logger().warn(f'Candidate {index} image was not saved.')

            if self.capture_only:
                self.get_logger().info(
                    f'Capture-only mode complete: saved {len(snapshots)} candidate images.')
                return

            if not snapshots:
                self.get_logger().warn('No camera image was available; selected goal will not be published.')
                return
            elif self.call_llm_api:
                self.publish_velocity(0.0)
                self.get_logger().info('Waiting for LLM API response before publishing a goal.')
                selected_index = self.ask_llm(robot_pose, snapshots)
                if selected_index is None:
                    self.get_logger().warn(
                        'LLM did not return a valid selected_index; selected goal will not be published.')
                    return
            else:
                if self.require_llm_response:
                    self.get_logger().warn(
                        'LLM API is disabled and require_llm_response is true; '
                        'selected goal will not be published.')
                    return
                self.get_logger().info('LLM API disabled; selecting candidate 0 because fallback is allowed.')
                selected_index = snapshots[0][0]

            selected_index = max(0, min(selected_index, len(poses) - 1))
            selected_goal = PoseStamped()
            selected_goal.header = header
            if not selected_goal.header.frame_id:
                selected_goal.header.frame_id = self.fixed_frame
            selected_goal.header.stamp = self.get_clock().now().to_msg()
            selected_goal.pose = poses[selected_index]
            self.selected_goal_pub.publish(selected_goal)
            self.get_logger().info(
                f'Visual selector chose candidate {selected_index}: '
                f'x={selected_goal.pose.position.x:.2f}, y={selected_goal.pose.position.y:.2f}')
        finally:
            self.publish_velocity(0.0)
            with self.selection_lock:
                if snapshots:
                    self.captured_once = True
                    self.captured_candidate_count = max(
                        self.captured_candidate_count, len(poses))
                self.selection_running = False
                self.last_selection_time = time.monotonic()

    def capture_candidate_image(self, index, pose):
        try:
            self.face_candidate(pose)
        except TransformException as exc:
            self.get_logger().warn(f'Could not face candidate {index}: {exc}')
            return None

        time.sleep(max(0.0, self.image_settle_sec))
        with self.latest_image_lock:
            image_msg = self.latest_image
        if image_msg is None:
            self.get_logger().warn('No camera image received yet.')
            return None

        encoded = self.encode_image(image_msg)
        if encoded is None:
            return None
        image_base64, jpeg_bytes = encoded
        image_path = self.save_debug_image(index, pose, jpeg_bytes)
        return image_base64, image_path

    def face_candidate(self, pose):
        deadline = time.monotonic() + self.candidate_timeout_sec
        while time.monotonic() < deadline and rclpy.ok():
            transform = self.tf_buffer.lookup_transform(
                self.fixed_frame, self.robot_frame, Time())
            robot = transform.transform.translation
            target_yaw = math.atan2(
                pose.position.y - robot.y,
                pose.position.x - robot.x)
            current_yaw = yaw_from_quaternion(transform.transform.rotation)
            error = normalize_angle(target_yaw - current_yaw)
            if abs(error) <= self.yaw_tolerance:
                self.publish_velocity(0.0)
                return
            command = math.copysign(self.turn_angular_speed, error)
            self.publish_velocity(command)
            time.sleep(0.05)
        self.publish_velocity(0.0)

    def lookup_robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.fixed_frame, self.robot_frame, Time())
            return transform.transform
        except TransformException:
            return None

    def filter_near_candidates(self, poses, robot_pose):
        if robot_pose is None:
            self.get_logger().warn(
                'Robot pose is unavailable; candidate distance filter is skipped.')
            return list(poses)

        filtered = []
        robot_x = robot_pose.translation.x
        robot_y = robot_pose.translation.y
        for pose in poses:
            dx = pose.position.x - robot_x
            dy = pose.position.y - robot_y
            distance = math.hypot(dx, dy)
            if distance < self.min_candidate_distance:
                self.get_logger().info(
                    f'Skipping nearby candidate: x={pose.position.x:.2f}, '
                    f'y={pose.position.y:.2f}, distance={distance:.2f} m')
                continue
            filtered.append(pose)
        return filtered

    def encode_image(self, msg):
        try:
            channels = {
                'rgb8': 3,
                'bgr8': 3,
                'rgba8': 4,
                'bgra8': 4,
                'mono8': 1,
            }.get(msg.encoding)
            if channels is None:
                self.get_logger().warn(f'Unsupported camera encoding: {msg.encoding}')
                return None

            array = np.frombuffer(msg.data, dtype=np.uint8)
            if channels == 1:
                image = array.reshape((msg.height, msg.width))
            else:
                image = array.reshape((msg.height, msg.width, channels))

            if msg.encoding == 'rgb8':
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            elif msg.encoding == 'rgba8':
                image = cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
            elif msg.encoding == 'bgra8':
                image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)

            ok, jpeg = cv2.imencode('.jpg', image, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
            if not ok:
                self.get_logger().warn('Could not JPEG-encode camera image.')
                return None
            jpeg_bytes = jpeg.tobytes()
            return base64.b64encode(jpeg_bytes).decode('ascii'), jpeg_bytes
        except ValueError as exc:
            self.get_logger().warn(f'Could not reshape camera image: {exc}')
            return None

    def save_debug_image(self, index, pose, jpeg_bytes):
        try:
            self.debug_image_dir.mkdir(parents=True, exist_ok=True)
            stamp = int(time.time() * 1000)
            filename = (
                f'candidate_{index}_x_{pose.position.x:.2f}_'
                f'y_{pose.position.y:.2f}_{stamp}.jpg')
            path = self.debug_image_dir / filename
            path.write_bytes(jpeg_bytes)
            self.get_logger().info(f'Saved candidate image {index}: {path}')
            return str(path)
        except OSError as exc:
            self.get_logger().warn(f'Could not save candidate image {index}: {exc}')
            return ''

    def ask_llm(self, robot_pose, snapshots):
        prompt = self.build_prompt(robot_pose, snapshots)
        images = [image_data[0] for _, _, image_data in snapshots]

        payload = {
            'model': self.llm_model,
            'messages': [{
                'role': 'user',
                'content': prompt,
                'images': images,
            }],
            'stream': False,
            'options': {
                'temperature': 0.1,
                'num_predict': 128,
            },
        }
        if self.disable_llm_reasoning:
            payload['think'] = False

        try:
            self.get_logger().info(
                f'Sending {len(snapshots)} candidate images to LLM API: {self.llm_api_url}')
            request_file = self.save_llm_debug_json('request', payload)
            request_debug_text = self.build_llm_request_debug_text(prompt, snapshots, request_file)
            self.publish_llm_request_debug(request_debug_text)
            waiting_response_debug_text = self.build_llm_response_debug_text(
                'WAITING_FOR_RESPONSE', '')
            self.publish_llm_exchange_debug(
                request_debug_text, waiting_response_debug_text, 'WAITING')
            request = urllib.request.Request(
                self.llm_api_url,
                data=json.dumps(payload).encode('utf-8'),
                headers={'Content-Type': 'application/json'},
                method='POST')
            with urllib.request.urlopen(
                request, timeout=self.llm_request_timeout_sec) as response:
                response_payload = json.loads(response.read().decode('utf-8'))
            response_file = self.save_llm_debug_json('response', response_payload)
            text = response_payload['message']['content']
            response_debug_text = self.build_llm_response_debug_text(text, response_file)
            self.publish_llm_response_debug(response_debug_text)
            self.publish_llm_exchange_debug(request_debug_text, response_debug_text, 'COMPLETED')
            selected_index = self.parse_selected_index(text)
            if selected_index is not None:
                self.get_logger().info(f'LLM selected candidate index {selected_index}.')
                return selected_index
            self.get_logger().warn(f'LLM response did not contain selected_index: {text}')
        except (KeyError, json.JSONDecodeError, urllib.error.URLError, TimeoutError) as exc:
            self.get_logger().warn(f'LLM request failed: {exc}')
            response_debug_text = self.build_llm_response_debug_text(
                f'LLM request failed: {exc}', '')
            self.publish_llm_response_debug(response_debug_text)
            if 'request_debug_text' in locals():
                self.publish_llm_exchange_debug(request_debug_text, response_debug_text, 'FAILED')
        return None

    def save_llm_debug_json(self, kind, payload):
        try:
            self.debug_image_dir.mkdir(parents=True, exist_ok=True)
            stamp = int(time.time() * 1000)
            path = self.debug_image_dir / f'llm_{kind}_{stamp}.json'
            path.write_text(
                json.dumps(payload, ensure_ascii=False, indent=2),
                encoding='utf-8')
            return str(path)
        except OSError as exc:
            self.get_logger().warn(f'Could not save LLM {kind} debug JSON: {exc}')
            return ''

    def build_llm_request_debug_text(self, prompt, snapshots, request_file):
        lines = [
            f'POST {self.llm_api_url}',
            f'model: {self.llm_model}',
            f'candidate_images: {len(snapshots)}',
            f'full_request_json: {request_file or "(not saved)"}',
            '',
            'prompt:',
            prompt,
            '',
            'images:',
        ]
        for index, pose, image_data in snapshots:
            _, image_path = image_data
            lines.append(
                f'- index {index}: x={pose.position.x:.2f}, '
                f'y={pose.position.y:.2f}, image={image_path or "(not saved)"}')
        return '\n'.join(lines)

    def publish_llm_request_debug(self, request_debug_text):
        message = String()
        message.data = request_debug_text
        self.llm_request_debug_pub.publish(message)

    def build_llm_response_debug_text(self, response_text, response_file):
        lines = [
            f'full_response_json: {response_file or "(not saved)"}',
            '',
            'response_text:',
            response_text,
        ]
        return '\n'.join(lines)

    def publish_llm_response_debug(self, response_debug_text):
        message = String()
        message.data = response_debug_text
        self.llm_response_debug_pub.publish(message)

    def publish_llm_exchange_debug(self, request_debug_text, response_debug_text, status):
        message = String()
        message.data = '\n'.join([
            f'========== LLM EXCHANGE: {status} ==========',
            '',
            '========== LLM REQUEST ==========', request_debug_text,
            '',
            '========== LLM RESPONSE ==========', response_debug_text,
        ])
        exchange_file = self.save_llm_debug_text('exchange', message.data)
        if exchange_file:
            message.data = '\n'.join([message.data, '', f'full_exchange_text: {exchange_file}'])
        self.llm_exchange_debug_pub.publish(message)
        self.publish_llm_exchange_chunks(message.data, status)

    def publish_llm_exchange_chunks(self, text, status):
        total = max(1, math.ceil(len(text) / self.llm_debug_chunk_size))
        for index in range(total):
            start = index * self.llm_debug_chunk_size
            end = start + self.llm_debug_chunk_size
            message = String()
            message.data = '\n'.join([
                f'========== LLM EXCHANGE CHUNK {index + 1}/{total}: {status} ==========',
                text[start:end],
            ])
            self.llm_exchange_debug_chunk_pub.publish(message)

    def save_llm_debug_text(self, kind, text):
        try:
            self.debug_image_dir.mkdir(parents=True, exist_ok=True)
            stamp = int(time.time() * 1000)
            path = self.debug_image_dir / f'llm_{kind}_{stamp}.txt'
            path.write_text(text, encoding='utf-8')
            return str(path)
        except OSError as exc:
            self.get_logger().warn(f'Could not save LLM {kind} debug text: {exc}')
            return ''

    def build_prompt(self, robot_pose, snapshots):
        robot_text = 'unknown'
        if robot_pose is not None:
            robot_text = (
                f'x={robot_pose.translation.x:.2f}, '
                f'y={robot_pose.translation.y:.2f}, '
                f'yaw={yaw_from_quaternion(robot_pose.rotation):.2f}')

        lines = [
            'You are choosing the next exploration goal for a mobile robot.',
            'Prefer a candidate that appears traversable, open, and likely to reveal unseen space.',
            'Avoid candidates facing walls, clutter, or dead ends.',
            'Do not think step by step. Do not include reasoning, analysis, explanation, or a reason field.',
            f'Robot pose: {robot_text}.',
            'Candidates:',
        ]
        for index, pose, _ in snapshots:
            lines.append(
                f'- index {index}: x={pose.position.x:.2f}, y={pose.position.y:.2f}')
        lines.append('Return only JSON like {"selected_index": 0}.')
        return '\n'.join(lines)

    def parse_selected_index(self, text):
        start = text.find('{')
        end = text.rfind('}')
        if start < 0 or end < start:
            return None
        data = json.loads(text[start:end + 1])
        value = data.get('selected_index')
        if isinstance(value, int):
            return value
        return None

    def publish_velocity(self, angular_z):
        command = Twist()
        command.angular.z = angular_z
        self.cmd_vel_pub.publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = VisualGoalSelectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

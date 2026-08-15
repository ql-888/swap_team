#!/usr/bin/env python3

from collections import deque
import datetime
import json
import math
import os
import time

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Image
from std_msgs.msg import Empty, String
import yaml


CALIBRATION_METHODS = {
    'TSAI': cv2.CALIB_HAND_EYE_TSAI,
    'PARK': cv2.CALIB_HAND_EYE_PARK,
    'HORAUD': cv2.CALIB_HAND_EYE_HORAUD,
    'ANDREFF': cv2.CALIB_HAND_EYE_ANDREFF,
    'DANIILIDIS': cv2.CALIB_HAND_EYE_DANIILIDIS,
}


def pose_to_matrix(pose):
    quaternion = np.array([
        pose.orientation.x, pose.orientation.y,
        pose.orientation.z, pose.orientation.w,
    ], dtype=np.float64)
    norm = np.linalg.norm(quaternion)
    if not np.isfinite(norm) or norm < 1e-9:
        raise ValueError('Pose contains an invalid quaternion')
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = Rotation.from_quat(quaternion / norm).as_matrix()
    transform[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    if not np.all(np.isfinite(transform)):
        raise ValueError('Pose contains non-finite values')
    return transform


def rotation_angle(rotation_matrix):
    return float(Rotation.from_matrix(rotation_matrix).magnitude())


def transform_delta(first, second):
    relative = np.linalg.inv(first) @ second
    return float(np.linalg.norm(relative[:3, 3])), rotation_angle(relative[:3, :3])


def calibration_inputs(robot_transforms, target_transforms, mode):
    if mode == 'eye_in_hand':
        robot_inputs = robot_transforms
    elif mode == 'eye_to_hand':
        robot_inputs = [np.linalg.inv(transform) for transform in robot_transforms]
    else:
        raise ValueError("mode must be 'eye_in_hand' or 'eye_to_hand'")
    return robot_inputs, target_transforms


def handeye_residual(robot_inputs, target_inputs, result):
    translation_errors = []
    rotation_errors = []
    for first in range(len(robot_inputs)):
        for second in range(first + 1, len(robot_inputs)):
            motion_robot = np.linalg.inv(robot_inputs[second]) @ robot_inputs[first]
            motion_target = target_inputs[second] @ np.linalg.inv(target_inputs[first])
            error = np.linalg.inv(motion_robot @ result) @ (result @ motion_target)
            translation_errors.append(np.linalg.norm(error[:3, 3]))
            rotation_errors.append(rotation_angle(error[:3, :3]))
    return {
        'translation_rms_m': float(np.sqrt(np.mean(np.square(translation_errors)))),
        'translation_max_m': float(np.max(translation_errors)),
        'rotation_rms_deg': float(np.degrees(np.sqrt(np.mean(np.square(rotation_errors))))),
        'rotation_max_deg': float(np.degrees(np.max(rotation_errors))),
    }


def solve_handeye(robot_transforms, target_transforms, mode):
    robot_inputs, target_inputs = calibration_inputs(
        robot_transforms, target_transforms, mode)
    rotations_robot = [item[:3, :3] for item in robot_inputs]
    translations_robot = [item[:3, 3] for item in robot_inputs]
    rotations_target = [item[:3, :3] for item in target_inputs]
    translations_target = [item[:3, 3] for item in target_inputs]

    solutions = []
    for name, method in CALIBRATION_METHODS.items():
        try:
            rotation, translation = cv2.calibrateHandEye(
                rotations_robot, translations_robot,
                rotations_target, translations_target, method=method)
            result = np.eye(4, dtype=np.float64)
            result[:3, :3] = rotation
            result[:3, 3] = np.asarray(translation).reshape(3)
            if not np.all(np.isfinite(result)) or abs(np.linalg.det(rotation) - 1.0) > 1e-3:
                continue
            residual = handeye_residual(robot_inputs, target_inputs, result)
            score = residual['translation_rms_m'] + math.radians(
                residual['rotation_rms_deg']) * 0.05
            solutions.append({
                'method': name,
                'transform': result,
                'residual': residual,
                'score': score,
            })
        except (cv2.error, ValueError, np.linalg.LinAlgError):
            continue
    solutions.sort(key=lambda item: item['score'])
    return solutions


def motion_span(transforms):
    max_translation = 0.0
    max_rotation = 0.0
    for first in range(len(transforms)):
        for second in range(first + 1, len(transforms)):
            translation, rotation = transform_delta(transforms[first], transforms[second])
            max_translation = max(max_translation, translation)
            max_rotation = max(max_rotation, rotation)
    return max_translation, math.degrees(max_rotation)


def capture_quality(board_visible, board_score, robot_transform, robot_samples):
    """Score whether the current frame is worth capturing, out of 100."""
    if not board_visible:
        return {
            'total': 0.0, 'board': 0.0, 'novelty': 0.0, 'coverage': 0.0,
            'verdict': 'NO BOARD',
            'suggestions': ['Keep the ChArUco board visible'],
        }
    board = float(np.clip(board_score, 0.0, 40.0))
    if robot_transform is None:
        novelty = 0.0
        coverage = 0.0
        suggestions = ['Robot flange pose unavailable']
    elif not robot_samples:
        novelty = 40.0
        coverage = 20.0
        suggestions = []
    else:
        distances = [transform_delta(sample, robot_transform) for sample in robot_samples]
        novelty = min(
            20.0 * min(1.0, translation / 0.08)
            + 20.0 * min(1.0, math.degrees(rotation) / 25.0)
            for translation, rotation in distances)
        candidate = np.concatenate((
            robot_transform[:3, 3],
            Rotation.from_matrix(robot_transform[:3, :3]).as_rotvec()))
        history = np.asarray([
            np.concatenate((
                sample[:3, 3], Rotation.from_matrix(sample[:3, :3]).as_rotvec()))
            for sample in robot_samples])
        expanded_axes = np.count_nonzero(
            (candidate < history.min(axis=0)) | (candidate > history.max(axis=0)))
        coverage = 20.0 * expanded_axes / 6.0
        suggestions = []
    if board < 25.0:
        suggestions.append('Move closer and keep the board sharp')
    if robot_transform is not None and novelty < 20.0:
        suggestions.append('Move farther from previous poses')
    if robot_samples and coverage < 10.0:
        suggestions.append('Rotate around another axis')
    if not suggestions:
        suggestions.append('Good pose: hold still and capture')
    total = float(np.clip(board + novelty + coverage, 0.0, 100.0))
    verdict = 'GOOD' if total >= 80.0 else 'USABLE' if total >= 60.0 else 'LOW'
    return {
        'total': total, 'board': board, 'novelty': float(novelty),
        'coverage': float(coverage), 'verdict': verdict,
        'suggestions': suggestions[:2],
    }


class HandEyeSessionNode(Node):
    def __init__(self):
        super().__init__('handeye_session')

        self.declare_parameter('camera', 'unknown')
        self.declare_parameter('mode', 'eye_in_hand')
        self.declare_parameter('marker_topic', '/charuco/pose')
        self.declare_parameter('result_image_topic', '/charuco/result')
        self.declare_parameter('robot_topic', '/end_pose')
        self.declare_parameter('robot_base_frame', 'base')
        self.declare_parameter('robot_end_frame', 'gripper')
        self.declare_parameter('result_save_path', os.path.expanduser('~/handeye_results'))
        self.declare_parameter('min_samples', 15)
        self.declare_parameter('max_data_age_sec', 1.0)
        self.declare_parameter('max_pair_time_delta_sec', 0.20)
        self.declare_parameter('startup_timeout_sec', 20.0)
        self.declare_parameter('min_sample_translation_m', 0.01)
        self.declare_parameter('min_sample_rotation_deg', 5.0)
        self.declare_parameter('min_motion_span_m', 0.05)
        self.declare_parameter('min_motion_span_deg', 20.0)
        self.declare_parameter('max_translation_rms_m', 0.02)
        self.declare_parameter('max_rotation_rms_deg', 3.0)
        self.declare_parameter('max_method_translation_spread_m', 0.03)
        self.declare_parameter('max_method_rotation_spread_deg', 3.0)
        self.declare_parameter('board_columns', 6)
        self.declare_parameter('board_rows', 9)
        self.declare_parameter('board_dictionary', 'DICT_4X4_50')
        self.declare_parameter('board_square_length_m', 0.03)
        self.declare_parameter('board_marker_length_m', 0.0216)
        self.declare_parameter('ui_mode', 'rqt')
        self.declare_parameter('preview_topic', '/charuco/result')

        self.camera = self.get_parameter('camera').value
        self.mode = self.get_parameter('mode').value
        self.marker_topic = self.get_parameter('marker_topic').value
        self.image_topic = self.get_parameter('result_image_topic').value
        self.robot_topic = self.get_parameter('robot_topic').value
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.robot_end_frame = self.get_parameter('robot_end_frame').value
        self.result_save_path = os.path.expanduser(
            self.get_parameter('result_save_path').value)
        self.min_samples = self.get_parameter('min_samples').value
        self.max_data_age = self.get_parameter('max_data_age_sec').value
        self.max_pair_time_delta = self.get_parameter('max_pair_time_delta_sec').value
        self.startup_timeout = self.get_parameter('startup_timeout_sec').value
        self.min_sample_translation = self.get_parameter('min_sample_translation_m').value
        self.min_sample_rotation = math.radians(
            self.get_parameter('min_sample_rotation_deg').value)

        if self.mode not in ('eye_in_hand', 'eye_to_hand'):
            raise ValueError(f'Unsupported hand-eye mode: {self.mode}')
        if self.min_samples < 10:
            raise ValueError('min_samples must be at least 10')

        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_image_time = 0.0
        self.latest_camera_time = 0.0
        self.latest_marker = None
        self.camera_frame = ''
        self.latest_marker_time = 0.0
        self.latest_robot = None
        self.latest_robot_time = 0.0
        self.marker_history = deque(maxlen=60)
        self.robot_history = deque(maxlen=400)
        self.latest_pair_robot = None
        self.latest_pair_robot_time = 0.0
        self.latest_pair_delta = float('inf')
        self.latest_board_quality = {'visible': False, 'score': 0.0}
        self.latest_board_quality_time = 0.0
        self.robot_samples = []
        self.target_samples = []
        self.status_message = 'Waiting for camera, ChArUco board, and Piper pose'
        self.status_color = (0, 200, 255)
        self.finished = False
        self.started_at = time.monotonic()
        self.window_name = 'Hand-eye calibration'
        self.ui_mode = self.get_parameter('ui_mode').value

        if self.ui_mode != 'rqt':
            self.create_subscription(
                Image, self.image_topic, self.image_callback, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, self.marker_topic, self.marker_callback, 10)
        self.create_subscription(PoseStamped, self.robot_topic, self.robot_callback, 50)
        self.create_subscription(Empty, '/charuco/heartbeat', self.camera_callback, 10)
        self.create_subscription(
            String, '/charuco/quality', self.quality_callback, 10)
        self.create_subscription(Empty, '/handeye/capture', self.capture_command, 10)
        self.create_subscription(Empty, '/handeye/remove', self.remove_command, 10)
        self.create_subscription(Empty, '/handeye/calculate', self.calculate_command, 10)
        self.create_subscription(Empty, '/handeye/exit', self.exit_command, 10)
        self.status_publisher = self.create_publisher(String, '/handeye/status', 10)
        self.status_timer = self.create_timer(0.2, self.publish_status)
        self.create_timer(0.04, self.ui_timer)
        self.get_logger().info(
            f'Automatic session: camera={self.camera}, mode={self.mode}, '
            f'marker={self.marker_topic}, robot={self.robot_topic}')

    def image_callback(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.latest_image_time = time.monotonic()

    def camera_callback(self, _msg):
        self.latest_camera_time = time.monotonic()

    def quality_callback(self, msg):
        try:
            self.latest_board_quality = json.loads(msg.data)
            self.latest_board_quality_time = time.monotonic()
        except (TypeError, ValueError):
            self.latest_board_quality = {'visible': False, 'score': 0.0}

    def marker_callback(self, msg):
        try:
            transform = pose_to_matrix(msg.pose)
        except ValueError as error:
            self.get_logger().warn(str(error))
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # Use the camera message clock for motion history so it can be compared
        # directly with the Piper message clock. Keep monotonic time only for
        # transport-age/watchdog checks below.
        now = time.monotonic()
        history_stamp = stamp if stamp > 0.0 else now
        self.camera_frame = msg.header.frame_id
        self.latest_marker = transform
        self.latest_marker_time = now
        self.marker_history.append((history_stamp, transform))
        if self.robot_history:
            pair_time, pair_transform = min(
                self.robot_history, key=lambda item: abs(item[0] - stamp))
            self.latest_pair_robot = pair_transform
            self.latest_pair_robot_time = pair_time
            self.latest_pair_delta = abs(pair_time - stamp)

    def robot_callback(self, msg):
        try:
            transform = pose_to_matrix(msg.pose)
        except ValueError as error:
            self.get_logger().warn(str(error))
            return
        now = time.monotonic()
        self.latest_robot = transform
        self.latest_robot_time = now
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        history_stamp = stamp if stamp > 0.0 else now
        self.robot_history.append((history_stamp, transform))

    def readiness(self):
        now = time.monotonic()
        return {
            'camera': self.latest_camera_time > 0.0 and now - self.latest_camera_time <= self.max_data_age,
            'board': self.latest_marker is not None and now - self.latest_marker_time <= self.max_data_age,
            'robot': self.latest_robot is not None and now - self.latest_robot_time <= self.max_data_age,
        }

    @staticmethod
    def history_is_stable(
            history, translation_limit, rotation_limit, window=0.25,
            min_count=3, min_duration=0.15):
        if len(history) < min_count:
            return False
        newest_time, newest = history[-1]
        recent = [item for timestamp, item in history if newest_time - timestamp <= window]
        recent_times = [timestamp for timestamp, _ in history
                        if newest_time - timestamp <= window]
        if len(recent) < min_count or recent_times[-1] - recent_times[0] < min_duration:
            return False
        for item in recent:
            translation, rotation = transform_delta(item, newest)
            if translation > translation_limit or rotation > rotation_limit:
                return False
        return True

    def capture_sample(self):
        ready = self.readiness()
        missing = [name for name, value in ready.items() if not value]
        if missing:
            self.set_status('Cannot capture: waiting for ' + ', '.join(missing), False)
            return
        if self.latest_pair_robot is None or self.latest_pair_delta > self.max_pair_time_delta:
            self.set_status('Camera and robot data are not synchronized closely enough', False)
            return
        if not self.history_is_stable(self.robot_history, 0.003, math.radians(2.0)):
            self.set_status('Hold the robot still before capturing', False)
            return
        if not self.history_is_stable(
                self.marker_history, 0.010, math.radians(3.0), window=1.20):
            self.set_status('Hold the ChArUco board steady and fully visible', False)
            return

        if self.robot_samples:
            distances = [
                transform_delta(sample, self.latest_pair_robot)
                for sample in self.robot_samples]
            if any(
                    translation < self.min_sample_translation
                    and rotation < self.min_sample_rotation
                    for translation, rotation in distances):
                self.set_status('Move to a clearly different position and angle', False)
                return

        self.robot_samples.append(self.latest_pair_robot.copy())
        self.target_samples.append(self.latest_marker.copy())
        self.set_status(f'Captured sample {len(self.robot_samples)}', True)

    def remove_sample(self):
        if not self.robot_samples:
            self.set_status('No sample to remove', False)
            return
        self.robot_samples.pop()
        self.target_samples.pop()
        self.set_status(f'Removed sample; {len(self.robot_samples)} remain', True)

    def calculate(self):
        if len(self.robot_samples) < self.min_samples:
            self.set_status(
                f'Need at least {self.min_samples} samples; current {len(self.robot_samples)}',
                False)
            return

        translation_span, rotation_span = motion_span(self.robot_samples)
        if translation_span < self.get_parameter('min_motion_span_m').value:
            self.set_status('Motion range is too small; collect wider positions', False)
            return
        if rotation_span < self.get_parameter('min_motion_span_deg').value:
            self.set_status('Rotation range is too small; collect more camera angles', False)
            return

        solutions = solve_handeye(self.robot_samples, self.target_samples, self.mode)
        if len(solutions) < 2:
            self.set_status('Calibration is degenerate; collect more diverse poses', False)
            return

        best = solutions[0]
        residual = best['residual']
        if residual['translation_rms_m'] > self.get_parameter('max_translation_rms_m').value:
            self.set_status('Translation residual is too high; collect cleaner samples', False)
            return
        if residual['rotation_rms_deg'] > self.get_parameter('max_rotation_rms_deg').value:
            self.set_status('Rotation residual is too high; collect cleaner samples', False)
            return

        second = solutions[1]
        method_translation, method_rotation = transform_delta(
            best['transform'], second['transform'])
        if method_translation > self.get_parameter('max_method_translation_spread_m').value:
            self.set_status('Calibration methods disagree in translation; add samples', False)
            return
        if math.degrees(method_rotation) > self.get_parameter(
                'max_method_rotation_spread_deg').value:
            self.set_status('Calibration methods disagree in rotation; add samples', False)
            return

        path = self.save_result(
            best, solutions, translation_span, rotation_span,
            method_translation, math.degrees(method_rotation))
        self.set_status(f'Calibration passed and saved: {path}', True)
        self.get_logger().info(self.status_message)
        self.finished = True

    def save_result(self, best, solutions, translation_span, rotation_span,
                    method_translation, method_rotation):
        os.makedirs(self.result_save_path, exist_ok=True)
        transform = best['transform']
        quaternion = Rotation.from_matrix(transform[:3, :3]).as_quat()
        transform_name = (
            f'{self.robot_end_frame}_T_camera'
            if self.mode == 'eye_in_hand' else 'base_T_camera')
        result = {
            'quality_passed': True,
            'camera': self.camera,
            'mode': self.mode,
            'transform_name': transform_name,
            'method': best['method'],
            'translation_m': transform[:3, 3].tolist(),
            'quaternion_xyzw': quaternion.tolist(),
            'rpy_deg': Rotation.from_matrix(transform[:3, :3]).as_euler(
                'xyz', degrees=True).tolist(),
            'matrix': transform.tolist(),
            'sample_count': len(self.robot_samples),
            'motion_span': {
                'translation_m': translation_span,
                'rotation_deg': rotation_span,
            },
            'best_residual': best['residual'],
            'top_two_method_spread': {
                'translation_m': method_translation,
                'rotation_deg': method_rotation,
            },
            'all_methods': [
                {'method': item['method'], 'residual': item['residual']}
                for item in solutions
            ],
            'board': {
                'columns': self.get_parameter('board_columns').value,
                'rows': self.get_parameter('board_rows').value,
                'dictionary': self.get_parameter('board_dictionary').value,
                'square_length_m': self.get_parameter('board_square_length_m').value,
                'marker_length_m': self.get_parameter('board_marker_length_m').value,
            },
            'samples': [
                {
                    'robot_transform': robot.tolist(),
                    'target_to_camera_transform': target.tolist(),
                }
                for robot, target in zip(self.robot_samples, self.target_samples)
            ],
        }
        if self.mode == 'eye_in_hand':
            parent_frame = self.robot_end_frame
        else:
            parent_frame = self.robot_base_frame
        result['static_tf'] = {
            'parent_frame': parent_frame,
            'child_frame': self.camera_frame,
            'translation_m': transform[:3, 3].tolist(),
            'quaternion_xyzw': quaternion.tolist(),
        }

        filename = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        basename = f'{filename}_{self.mode}_{self.camera}'
        path = os.path.join(self.result_save_path, basename + '.json')
        yaml_path = os.path.join(self.result_save_path, basename + '.yaml')
        with open(path, 'w', encoding='utf-8') as output:
            json.dump(result, output, indent=2)
        with open(yaml_path, 'w', encoding='utf-8') as output:
            yaml.safe_dump(result, output, sort_keys=False, allow_unicode=True)
        return path

    def set_status(self, message, success):
        self.status_message = message
        self.status_color = (80, 220, 80) if success else (40, 80, 255)
        if success:
            self.get_logger().info(message)
        else:
            self.get_logger().warn(message)

    def capture_command(self, _msg):
        self.capture_sample()

    def remove_command(self, _msg):
        self.remove_sample()

    def calculate_command(self, _msg):
        self.calculate()

    def exit_command(self, _msg):
        self.finished = True

    def publish_status(self):
        ready = self.readiness()
        robot_stable = self.history_is_stable(
            self.robot_history, 0.003, math.radians(2.0))
        board_stable = self.history_is_stable(
            self.marker_history, 0.010, math.radians(3.0), window=1.20)
        nearest_translation = float('inf')
        nearest_rotation = float('inf')
        if self.robot_samples and self.latest_pair_robot is not None:
            distances = [
                transform_delta(sample, self.latest_pair_robot)
                for sample in self.robot_samples]
            nearest_translation, nearest_rotation = min(
                distances, key=lambda item: item[0] + item[1] * 0.05)
        unique = (
            not self.robot_samples
            or nearest_translation >= self.min_sample_translation
            or nearest_rotation >= self.min_sample_rotation)
        can_capture = (
            all(ready.values()) and robot_stable and board_stable and unique
            and self.latest_pair_delta <= self.max_pair_time_delta)
        quality_fresh = (
            self.latest_board_quality_time > 0.0
            and time.monotonic() - self.latest_board_quality_time <= self.max_data_age)
        quality = capture_quality(
            quality_fresh and self.latest_board_quality.get('visible', False),
            self.latest_board_quality.get('score', 0.0),
            self.latest_pair_robot,
            self.robot_samples)
        if can_capture:
            guidance = 'Ready: capture this new stable pose'
        elif not ready['camera']:
            guidance = 'Waiting for camera frames'
        elif not ready['board']:
            guidance = 'Keep the full ChArUco board visible'
        elif not ready['robot']:
            guidance = 'Waiting for Piper pose'
        elif self.latest_pair_delta > self.max_pair_time_delta:
            guidance = 'Waiting for synchronized camera and robot data'
        elif not robot_stable:
            guidance = 'Hold the robot still'
        elif not board_stable:
            guidance = 'Hold the ChArUco board steady'
        elif not unique:
            guidance = 'Old pose detected: move farther from previous samples'
        else:
            guidance = self.status_message
        payload = {
            'camera_name': self.camera,
            'mode': self.mode,
            'camera': ready['camera'],
            'board': ready['board'],
            'robot': ready['robot'],
            'pair_delta_sec': self.latest_pair_delta,
            'robot_stable': robot_stable,
            'board_stable': board_stable,
            'nearest_translation_m': nearest_translation,
            'nearest_rotation_deg': math.degrees(nearest_rotation),
            'unique': unique,
            'can_capture': can_capture,
            'samples': len(self.robot_samples),
            'min_samples': self.min_samples,
            'message': guidance,
            'capture_quality': quality,
            'finished': self.finished,
        }
        message = String()
        message.data = json.dumps(payload, ensure_ascii=True)
        self.status_publisher.publish(message)

    def ui_timer(self):
        if self.finished:
            cv2.destroyAllWindows()
            if rclpy.ok():
                rclpy.shutdown()
            return

        ready = self.readiness()
        infrastructure_received = {
            'camera': self.latest_camera_time > 0.0,
            'robot': self.latest_robot is not None,
        }
        infrastructure_missing = [
            name for name, received in infrastructure_received.items()
            if not received]
        if (infrastructure_missing
                and time.monotonic() - self.started_at > self.startup_timeout):
            self.get_logger().error(
                'Startup validation failed: no live ' + ', '.join(infrastructure_missing))
            self.finished = True
            return

        if self.ui_mode == 'rqt':
            return
        if self.latest_image is None:
            image = np.zeros((540, 960, 3), dtype=np.uint8)
        else:
            image = self.latest_image.copy()
        lines = [
            f'{self.camera} | {self.mode} | samples {len(self.robot_samples)}/{self.min_samples}',
            '  '.join(f'{name}: {"OK" if value else "WAIT"}' for name, value in ready.items()),
            'SPACE capture   D remove   Q validate/save   ESC exit',
            self.status_message,
        ]
        overlay = image.copy()
        cv2.rectangle(overlay, (0, 0), (image.shape[1], 122), (0, 0, 0), -1)
        image = cv2.addWeighted(overlay, 0.65, image, 0.35, 0)
        for index, line in enumerate(lines):
            color = self.status_color if index == 3 else (255, 255, 255)
            cv2.putText(
                image, line, (12, 26 + index * 29), cv2.FONT_HERSHEY_SIMPLEX,
                0.62, color, 2, cv2.LINE_AA)
        cv2.imshow(self.window_name, image)
        key = cv2.waitKey(1) & 0xFF
        if key == ord(' '):
            self.capture_sample()
        elif key in (ord('d'), ord('D')):
            self.remove_sample()
        elif key in (ord('q'), ord('Q')):
            self.calculate()
        elif key == 27:
            self.finished = True


def main(args=None):
    rclpy.init(args=args)
    node = HandEyeSessionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        if rclpy.ok():
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()

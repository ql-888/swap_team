#!/usr/bin/env python3

import json

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Empty, String
from tf2_ros import TransformBroadcaster


DICTIONARIES = {
    name: getattr(cv2.aruco, name)
    for name in dir(cv2.aruco)
    if name.startswith('DICT_') and isinstance(getattr(cv2.aruco, name), int)
}


def create_board(columns, rows, square_length, marker_length, dictionary_name,
                 marker_id_offset=0):
    if dictionary_name not in DICTIONARIES:
        supported = ', '.join(sorted(DICTIONARIES))
        raise ValueError(
            f'Unsupported ArUco dictionary {dictionary_name!r}. Supported: {supported}')
    if columns < 2 or rows < 2:
        raise ValueError('ChArUco columns and rows must both be at least 2')
    if square_length <= 0 or marker_length <= 0 or marker_length >= square_length:
        raise ValueError('Lengths must satisfy 0 < marker_length < square_length')

    dictionary = cv2.aruco.Dictionary_get(DICTIONARIES[dictionary_name])
    board = cv2.aruco.CharucoBoard_create(
        columns, rows, square_length, marker_length, dictionary)
    if marker_id_offset:
        board.ids = np.asarray(board.ids, dtype=np.int32) + marker_id_offset
    return dictionary, board


def board_image_score(image, corners, max_charuco_corners):
    """Score ChArUco completeness, image area, and sharpness out of 40."""
    points = np.asarray(corners, dtype=np.float32).reshape(-1, 2)
    if len(points) == 0 or max_charuco_corners <= 0:
        return {
            'score': 0.0, 'corner_count': 0,
            'completeness': 0.0, 'area_ratio': 0.0, 'sharpness': 0.0,
        }
    completeness = min(1.0, len(points) / max_charuco_corners)
    x0, y0 = np.floor(points.min(axis=0)).astype(int)
    x1, y1 = np.ceil(points.max(axis=0)).astype(int)
    x0 = int(np.clip(x0, 0, image.shape[1] - 1))
    x1 = int(np.clip(x1, x0 + 1, image.shape[1]))
    y0 = int(np.clip(y0, 0, image.shape[0] - 1))
    y1 = int(np.clip(y1, y0 + 1, image.shape[0]))
    area_ratio = ((x1 - x0) * (y1 - y0)) / (image.shape[0] * image.shape[1])
    area_quality = min(1.0, area_ratio / 0.12)
    gray_roi = cv2.cvtColor(image[y0:y1, x0:x1], cv2.COLOR_BGR2GRAY)
    sharpness = float(cv2.Laplacian(gray_roi, cv2.CV_64F).var())
    sharpness_quality = min(1.0, sharpness / 200.0)
    score = 20.0 * completeness + 10.0 * area_quality + 10.0 * sharpness_quality
    return {
        'score': float(np.clip(score, 0.0, 40.0)),
        'corner_count': len(points),
        'completeness': float(completeness),
        'area_ratio': float(area_ratio),
        'sharpness': sharpness,
    }


class CharucoDetectorNode(Node):
    def __init__(self):
        super().__init__('charuco_detector')

        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('pose_topic', '/charuco/pose')
        self.declare_parameter('result_image_topic', '/charuco/result')
        self.declare_parameter('quality_topic', '/charuco/quality')
        self.declare_parameter('board_frame', 'charuco_board')
        self.declare_parameter('columns', 6)
        self.declare_parameter('rows', 9)
        self.declare_parameter('square_length', 0.030)
        self.declare_parameter('marker_length', 0.0216)
        self.declare_parameter('dictionary', 'DICT_4X4_50')
        self.declare_parameter('marker_id_offset', 0)
        self.declare_parameter('min_charuco_corners', 6)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('draw_marker_ids', False)
        self.declare_parameter('max_processing_rate_hz', 15.0)
        self.declare_parameter('preview_width', 960)
        self.declare_parameter('detection_width', 960)

        self.image_topic = self.get_parameter('image_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        pose_topic = self.get_parameter('pose_topic').value
        result_image_topic = self.get_parameter('result_image_topic').value
        self.board_frame = self.get_parameter('board_frame').value
        self.min_charuco_corners = self.get_parameter('min_charuco_corners').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.draw_marker_ids = self.get_parameter('draw_marker_ids').value
        self.max_processing_rate = self.get_parameter('max_processing_rate_hz').value
        self.preview_width = self.get_parameter('preview_width').value
        self.detection_width = self.get_parameter('detection_width').value
        self.square_length = self.get_parameter('square_length').value
        columns = self.get_parameter('columns').value
        rows = self.get_parameter('rows').value
        self.max_charuco_corners = (columns - 1) * (rows - 1)

        self.dictionary, self.board = create_board(
            columns,
            rows,
            self.square_length,
            self.get_parameter('marker_length').value,
            self.get_parameter('dictionary').value,
            self.get_parameter('marker_id_offset').value,
        )
        if self.min_charuco_corners < 4:
            raise ValueError('min_charuco_corners must be at least 4')

        self.detector_parameters = cv2.aruco.DetectorParameters_create()
        self.detector_parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.detection_camera_matrix = None
        self.distortion = None
        self.camera_frame = ''
        self.latest_image_message = None
        self.last_process_time = 0.0

        self.pose_publisher = self.create_publisher(PoseStamped, pose_topic, 10)
        self.image_publisher = self.create_publisher(Image, result_image_topic, 1)
        self.quality_publisher = self.create_publisher(
            String, self.get_parameter('quality_topic').value, 10)
        self.heartbeat_publisher = self.create_publisher(Empty, '/charuco/heartbeat', 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.camera_info_subscription = self.create_subscription(
            CameraInfo, self.camera_info_topic, self.camera_info_callback,
            qos_profile_sensor_data)
        self.image_subscription = self.create_subscription(
            Image, self.image_topic, self.image_callback, qos_profile_sensor_data)
        self.process_timer = self.create_timer(
            1.0 / max(1.0, self.max_processing_rate), self.process_latest_image)

        self.get_logger().info(
            f'ChArUco detector: image={self.image_topic}, '
            f'camera_info={self.camera_info_topic}')

    def camera_info_callback(self, msg):
        camera_matrix = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)
        if camera_matrix[0, 0] <= 0 or camera_matrix[1, 1] <= 0:
            self.get_logger().error('CameraInfo contains invalid focal lengths')
            return
        self.camera_matrix = camera_matrix
        self.detection_camera_matrix = camera_matrix.copy()
        self.distortion = np.asarray(msg.d, dtype=np.float64)
        self.camera_frame = msg.header.frame_id

    def image_callback(self, msg):
        self.latest_image_message = msg

    def process_latest_image(self):
        msg = self.latest_image_message
        if msg is None:
            return
        self.heartbeat_publisher.publish(Empty())
        if self.camera_matrix is None:
            self.get_logger().warn(
                f'Waiting for CameraInfo on {self.camera_info_topic}',
                throttle_duration_sec=5.0)
            return

        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if 0 < self.detection_width < image.shape[1]:
            detection_height = round(image.shape[0] * self.detection_width / image.shape[1])
            scale_x = self.detection_width / image.shape[1]
            scale_y = detection_height / image.shape[0]
            image = cv2.resize(
                image, (self.detection_width, detection_height),
                interpolation=cv2.INTER_AREA)
            self.detection_camera_matrix = self.camera_matrix.copy()
            self.detection_camera_matrix[0, :] *= scale_x
            self.detection_camera_matrix[1, :] *= scale_y
        else:
            self.detection_camera_matrix = self.camera_matrix
        scoring_image = image.copy()
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(
            gray, self.dictionary, parameters=self.detector_parameters)

        charuco_corners = None
        charuco_ids = None
        quality = {
            'visible': False, 'score': 0.0, 'corner_count': 0,
            'max_corner_count': self.max_charuco_corners,
            'completeness': 0.0, 'area_ratio': 0.0, 'sharpness': 0.0,
        }
        if marker_ids is not None:
            if self.draw_marker_ids:
                cv2.aruco.drawDetectedMarkers(image, marker_corners, marker_ids)
            else:
                for corners in marker_corners:
                    points = np.asarray(corners, dtype=np.int32).reshape(-1, 1, 2)
                    cv2.polylines(image, [points], True, (0, 220, 0), 2)
            corner_count, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                marker_corners, marker_ids, gray, self.board,
                cameraMatrix=self.detection_camera_matrix, distCoeffs=self.distortion)
            if corner_count is not None and corner_count >= self.min_charuco_corners:
                quality.update(board_image_score(
                    scoring_image, charuco_corners, self.max_charuco_corners))
                quality['visible'] = True
                self.publish_pose(msg, image, charuco_corners, charuco_ids)

        quality_message = String()
        quality_message.data = json.dumps(quality, ensure_ascii=True)
        self.quality_publisher.publish(quality_message)

        if 0 < self.preview_width < image.shape[1]:
            preview_height = round(image.shape[0] * self.preview_width / image.shape[1])
            image = cv2.resize(
                image, (self.preview_width, preview_height),
                interpolation=cv2.INTER_AREA)
        result_msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        result_msg.header = msg.header
        self.image_publisher.publish(result_msg)
        self.latest_image_message = None

    def publish_pose(self, image_msg, image, charuco_corners, charuco_ids):
        cv2.aruco.drawDetectedCornersCharuco(image, charuco_corners, charuco_ids)
        valid, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
            charuco_corners, charuco_ids, self.board,
            self.detection_camera_matrix, self.distortion, None, None)
        if not valid:
            return

        cv2.drawFrameAxes(
            image, self.detection_camera_matrix, self.distortion, rvec, tvec,
            self.square_length * 2.0)
        quaternion = Rotation.from_rotvec(rvec.reshape(3)).as_quat()
        translation = tvec.reshape(3)
        frame_id = image_msg.header.frame_id or self.camera_frame

        pose = PoseStamped()
        pose.header = image_msg.header
        pose.header.frame_id = frame_id
        pose.pose.position.x = float(translation[0])
        pose.pose.position.y = float(translation[1])
        pose.pose.position.z = float(translation[2])
        pose.pose.orientation.x = float(quaternion[0])
        pose.pose.orientation.y = float(quaternion[1])
        pose.pose.orientation.z = float(quaternion[2])
        pose.pose.orientation.w = float(quaternion[3])
        self.pose_publisher.publish(pose)

        if self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header = pose.header
            transform.child_frame_id = self.board_frame
            transform.transform.translation.x = pose.pose.position.x
            transform.transform.translation.y = pose.pose.position.y
            transform.transform.translation.z = pose.pose.position.z
            transform.transform.rotation = pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = CharucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

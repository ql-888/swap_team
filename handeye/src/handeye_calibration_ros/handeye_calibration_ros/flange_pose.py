#!/usr/bin/env python3

import math
import xml.etree.ElementTree as ET

from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState


def transform_from_xyz_rpy(xyz, rpy):
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
    transform[:3, 3] = xyz
    return transform


def axis_angle_transform(axis, angle):
    transform = np.eye(4, dtype=np.float64)
    axis = np.asarray(axis, dtype=np.float64)
    transform[:3, :3] = Rotation.from_rotvec(
        axis / np.linalg.norm(axis) * angle).as_matrix()
    return transform


def parse_vector(element, attribute, default):
    if element is None or attribute not in element.attrib:
        return np.asarray(default, dtype=np.float64)
    return np.asarray(
        [float(value) for value in element.attrib[attribute].split()],
        dtype=np.float64)


def load_chain(urdf_path, base_frame, flange_frame):
    root = ET.parse(urdf_path).getroot()
    joints_by_child = {}
    for joint in root.findall('joint'):
        child = joint.find('child').attrib['link']
        joints_by_child[child] = joint

    reversed_chain = []
    link = flange_frame
    while link != base_frame:
        if link not in joints_by_child:
            raise ValueError(
                f'No URDF chain from {base_frame!r} to {flange_frame!r}')
        joint = joints_by_child[link]
        reversed_chain.append(joint)
        link = joint.find('parent').attrib['link']
    return list(reversed(reversed_chain))


class FlangePoseNode(Node):
    def __init__(self):
        super().__init__('flange_pose')
        self.declare_parameter('urdf_path', '')
        self.declare_parameter('joint_topic', '/joint_states_feedback')
        self.declare_parameter('pose_topic', '/flange_pose_stamped')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('flange_frame', 'link6')

        urdf_path = self.get_parameter('urdf_path').value
        if not urdf_path:
            raise ValueError('urdf_path must point to the Piper URDF')
        self.base_frame = self.get_parameter('base_frame').value
        self.flange_frame = self.get_parameter('flange_frame').value
        self.chain = load_chain(urdf_path, self.base_frame, self.flange_frame)
        self.moving_joint_names = [
            joint.attrib['name'] for joint in self.chain
            if joint.attrib.get('type') in ('revolute', 'continuous', 'prismatic')]
        if self.moving_joint_names != [f'joint{index}' for index in range(1, 7)]:
            raise ValueError(
                f'Unexpected Piper flange chain: {self.moving_joint_names}')

        self.publisher = self.create_publisher(
            PoseStamped, self.get_parameter('pose_topic').value, 10)
        self.subscription = self.create_subscription(
            JointState, self.get_parameter('joint_topic').value,
            self.joint_callback, 50)
        self.get_logger().info(
            f'Publishing TCP-independent flange FK: '
            f'{self.base_frame} -> {self.flange_frame}')

    def joint_callback(self, message):
        positions = dict(zip(message.name, message.position))
        missing = [name for name in self.moving_joint_names if name not in positions]
        if missing:
            self.get_logger().warn(
                'JointState is missing: ' + ', '.join(missing),
                throttle_duration_sec=5.0)
            return

        transform = np.eye(4, dtype=np.float64)
        for joint in self.chain:
            origin = joint.find('origin')
            xyz = parse_vector(origin, 'xyz', (0.0, 0.0, 0.0))
            rpy = parse_vector(origin, 'rpy', (0.0, 0.0, 0.0))
            transform = transform @ transform_from_xyz_rpy(xyz, rpy)
            joint_type = joint.attrib.get('type', 'fixed')
            if joint_type in ('revolute', 'continuous'):
                axis = parse_vector(joint.find('axis'), 'xyz', (1.0, 0.0, 0.0))
                transform = transform @ axis_angle_transform(
                    axis, positions[joint.attrib['name']])
            elif joint_type == 'prismatic':
                axis = parse_vector(joint.find('axis'), 'xyz', (1.0, 0.0, 0.0))
                motion = np.eye(4, dtype=np.float64)
                motion[:3, 3] = axis * positions[joint.attrib['name']]
                transform = transform @ motion

        quaternion = Rotation.from_matrix(transform[:3, :3]).as_quat()
        pose = PoseStamped()
        pose.header = message.header
        pose.header.frame_id = self.base_frame
        pose.pose.position.x = float(transform[0, 3])
        pose.pose.position.y = float(transform[1, 3])
        pose.pose.position.z = float(transform[2, 3])
        pose.pose.orientation.x = float(quaternion[0])
        pose.pose.orientation.y = float(quaternion[1])
        pose.pose.orientation.z = float(quaternion[2])
        pose.pose.orientation.w = float(quaternion[3])
        self.publisher.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = FlangePoseNode()
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

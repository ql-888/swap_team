import math
import xml.etree.ElementTree as ET

import numpy as np

from handeye_calibration_ros.flange_pose import (
    axis_angle_transform, load_chain, parse_vector, transform_from_xyz_rpy)


URDF_PATH = (
    '/home/guoyi/gy_ws/handeye/src/piper_ros/src/piper_description/'
    'urdf/piper_description.urdf')


def calculate_fk(chain, positions):
    transform = np.eye(4)
    for joint in chain:
        origin = joint.find('origin')
        transform = transform @ transform_from_xyz_rpy(
            parse_vector(origin, 'xyz', (0.0, 0.0, 0.0)),
            parse_vector(origin, 'rpy', (0.0, 0.0, 0.0)))
        if joint.attrib.get('type') == 'revolute':
            transform = transform @ axis_angle_transform(
                parse_vector(joint.find('axis'), 'xyz', (1.0, 0.0, 0.0)),
                positions[joint.attrib['name']])
    return transform


def test_piper_flange_chain_uses_only_six_arm_joints():
    chain = load_chain(URDF_PATH, 'base_link', 'link6')
    assert [joint.attrib['name'] for joint in chain] == [
        'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']


def test_flange_fk_is_finite_rigid_transform():
    chain = load_chain(URDF_PATH, 'base_link', 'link6')
    positions = {
        'joint1': 0.2, 'joint2': 1.0, 'joint3': -1.2,
        'joint4': 0.1, 'joint5': -0.2, 'joint6': 0.3,
    }
    transform = calculate_fk(chain, positions)
    assert np.all(np.isfinite(transform))
    assert np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0])
    assert math.isclose(np.linalg.det(transform[:3, :3]), 1.0, abs_tol=1e-9)

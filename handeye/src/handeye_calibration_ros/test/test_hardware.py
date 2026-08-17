from pathlib import Path
from types import SimpleNamespace

import pytest

from handeye_calibration_ros.hardware import (
    acquire_instance_lock, require_can_interface, select_camera, usb_devices)


def add_usb_device(root, name, vendor, product_id, product, speed):
    path = Path(root) / name
    path.mkdir()
    for field, value in (
        ('idVendor', vendor), ('idProduct', product_id),
        ('product', product), ('speed', speed),
    ):
        (path / field).write_text(str(value))


def test_selects_d435i_for_eye_in_hand(tmp_path):
    add_usb_device(tmp_path, 'a', '8086', '0b3a', 'Intel RealSense D435I', 5000)
    assert select_camera(usb_devices(tmp_path))[:2] == ('realsense', 'eye_in_hand')


def test_selects_gemini_336l_for_eye_to_hand(tmp_path):
    add_usb_device(tmp_path, 'a', '2bc5', '0807', 'Orbbec Gemini 336L', 5000)
    assert select_camera(usb_devices(tmp_path))[:2] == ('orbbec', 'eye_to_hand')


def test_rejects_usb2(tmp_path):
    add_usb_device(tmp_path, 'a', '2bc5', '0807', 'Orbbec Gemini 336L', 480)
    with pytest.raises(RuntimeError, match='USB 3'):
        select_camera(usb_devices(tmp_path))


def test_rejects_two_calibration_cameras(tmp_path):
    add_usb_device(tmp_path, 'a', '8086', '0b3a', 'Intel RealSense D435I', 5000)
    add_usb_device(tmp_path, 'b', '2bc5', '0807', 'Orbbec Gemini 336L', 5000)
    with pytest.raises(RuntimeError, match='Both'):
        select_camera(usb_devices(tmp_path))


def test_explicit_eye_in_hand_selects_d435i_when_both_are_connected(tmp_path):
    add_usb_device(tmp_path, 'a', '8086', '0b3a', 'Intel RealSense D435I', 5000)
    add_usb_device(tmp_path, 'b', '2bc5', '0807', 'Orbbec Gemini 336L', 5000)
    assert select_camera(
        usb_devices(tmp_path), requested_camera='realsense')[:2] == (
            'realsense', 'eye_in_hand')


def test_explicit_eye_to_hand_selects_orbbec_when_both_are_connected(tmp_path):
    add_usb_device(tmp_path, 'a', '8086', '0b3a', 'Intel RealSense D435I', 5000)
    add_usb_device(tmp_path, 'b', '2bc5', '0807', 'Orbbec Gemini 336L', 5000)
    assert select_camera(
        usb_devices(tmp_path), requested_camera='orbbec')[:2] == (
            'orbbec', 'eye_to_hand')


def test_instance_lock_rejects_second_launch(tmp_path):
    lock_path = tmp_path / 'handeye.lock'
    first = acquire_instance_lock(lock_path)
    try:
        with pytest.raises(RuntimeError, match='already running'):
            acquire_instance_lock(lock_path)
    finally:
        first.close()


def test_instance_lock_is_reusable_after_launch_exits(tmp_path):
    lock_path = tmp_path / 'handeye.lock'
    first = acquire_instance_lock(lock_path)
    first.close()
    second = acquire_instance_lock(lock_path)
    second.close()


def test_requires_up_can_interface():
    def runner(*args, **kwargs):
        return SimpleNamespace(
            returncode=0,
            stdout='2: can0: <NOARP,UP,LOWER_UP> mtu 16\n    can state ERROR-ACTIVE bitrate 1000000')
    require_can_interface('can0', runner)


def test_reports_disconnected_usb_can_adapter():
    def runner(*args, **kwargs):
        return SimpleNamespace(returncode=1, stdout='')
    with pytest.raises(RuntimeError, match='not connected'):
        require_can_interface('can0', runner, devices=[])


def test_reports_adapter_without_socketcan_interface():
    def runner(*args, **kwargs):
        return SimpleNamespace(returncode=1, stdout='')
    adapter = [{
        'vendor': '1d50', 'product_id': '606f',
        'product': 'candleLight', 'speed_mbps': 12, 'path': '/sys/fake',
    }]
    with pytest.raises(RuntimeError, match='was not created'):
        require_can_interface('can0', runner, devices=adapter)


def test_rejects_down_can_interface():
    def runner(*args, **kwargs):
        return SimpleNamespace(
            returncode=0,
            stdout='2: can0: <NOARP,ECHO> mtu 16\n    can state STOPPED bitrate 1000000')
    with pytest.raises(RuntimeError, match='DOWN'):
        require_can_interface('can0', runner)


def test_rejects_wrong_can_bitrate():
    def runner(*args, **kwargs):
        return SimpleNamespace(
            returncode=0,
            stdout='2: can0: <NOARP,UP,LOWER_UP> mtu 16\n    can state ERROR-ACTIVE bitrate 500000')
    with pytest.raises(RuntimeError, match='requires 1000000'):
        require_can_interface('can0', runner)


def test_auto_activates_down_can_interface():
    responses = iter([
        SimpleNamespace(
            returncode=0,
            stdout='2: can0: <NOARP,ECHO> mtu 16\n    can state STOPPED bitrate 500000'),
        SimpleNamespace(
            returncode=0,
            stdout='2: can0: <NOARP,UP,LOWER_UP> mtu 16\n    can state ERROR-ACTIVE bitrate 1000000'),
    ])
    commands = []

    def status_runner(*args, **kwargs):
        return next(responses)

    def activation_runner(command, **kwargs):
        commands.append(command)
        return SimpleNamespace(returncode=0)

    require_can_interface(
        'can0', status_runner, auto_activate=True,
        activation_runner=activation_runner)
    assert commands[0][0] == 'pkexec'
    assert commands[0][-2:] == ['can0', '1000000']


def test_reports_failed_can_authorization():
    def status_runner(*args, **kwargs):
        return SimpleNamespace(
            returncode=0,
            stdout='2: can0: <NOARP,ECHO> mtu 16\n    can state STOPPED bitrate 500000')

    def activation_runner(*args, **kwargs):
        return SimpleNamespace(returncode=126)

    with pytest.raises(RuntimeError, match='authorization'):
        require_can_interface(
            'can0', status_runner, auto_activate=True,
            activation_runner=activation_runner)

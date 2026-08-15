from pathlib import Path
import fcntl
import re
import subprocess


REALSENSE_D435I_PRODUCT_IDS = {'0b3a'}
ORBBEC_GEMINI_336L_PRODUCT_ID = '0807'
PIPER_CAN_ADAPTER_IDS = {('1d50', '606f')}
PIPER_CAN_BITRATE = 1_000_000
CAN_INTERFACE_PATTERN = re.compile(r'^[A-Za-z0-9_.:-]+$')


def acquire_instance_lock(lock_path='/tmp/handeye_calibration_ros.lock'):
    """Hold an advisory lock for the lifetime of one calibration launch."""
    lock_file = open(lock_path, 'w', encoding='ascii')
    try:
        fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as error:
        lock_file.close()
        raise RuntimeError(
            'A hand-eye calibration launch is already running. Close its rqt '
            'window or press Ctrl+C in the original terminal before starting again.') from error
    lock_file.write(str(Path('/proc/self').resolve().name))
    lock_file.flush()
    return lock_file


def usb_devices(sysfs_root='/sys/bus/usb/devices'):
    devices = []
    for device_path in Path(sysfs_root).glob('*'):
        try:
            vendor = (device_path / 'idVendor').read_text().strip().lower()
            product_id = (device_path / 'idProduct').read_text().strip().lower()
        except OSError:
            continue
        try:
            product = (device_path / 'product').read_text().strip()
        except OSError:
            product = ''
        try:
            speed = float((device_path / 'speed').read_text().strip())
        except (OSError, ValueError):
            speed = 0.0
        devices.append({
            'vendor': vendor,
            'product_id': product_id,
            'product': product,
            'speed_mbps': speed,
            'path': str(device_path),
        })
    return devices


def select_camera(devices, requested_camera='auto'):
    realsense = [
        item for item in devices
        if item['vendor'] == '8086' and (
            item['product_id'] in REALSENSE_D435I_PRODUCT_IDS
            or 'd435i' in item['product'].lower()
            or '435i' in item['product'].lower())
    ]
    orbbec = [
        item for item in devices
        if item['vendor'] == '2bc5'
        and item['product_id'] == ORBBEC_GEMINI_336L_PRODUCT_ID
    ]
    if requested_camera not in ('auto', 'realsense', 'orbbec'):
        raise ValueError(f'Unsupported requested camera: {requested_camera}')
    if requested_camera == 'auto' and realsense and orbbec:
        raise RuntimeError(
            'Both D435i and Gemini 336L are connected. Disconnect one camera so the '
            'calibration type cannot be selected incorrectly.')
    if len(realsense) > 1 or len(orbbec) > 1:
        raise RuntimeError('Multiple cameras of the same supported model were detected.')
    if requested_camera == 'realsense':
        if not realsense:
            raise RuntimeError('Eye-in-hand requires a connected D435i camera.')
        camera = 'realsense'
        device = realsense[0]
        mode = 'eye_in_hand'
    elif requested_camera == 'orbbec':
        if not orbbec:
            raise RuntimeError('Eye-to-hand requires a connected Gemini 336L camera.')
        camera = 'orbbec'
        device = orbbec[0]
        mode = 'eye_to_hand'
    elif realsense:
        camera = 'realsense'
        device = realsense[0]
        mode = 'eye_in_hand'
    elif orbbec:
        camera = 'orbbec'
        device = orbbec[0]
        mode = 'eye_to_hand'
    else:
        raise RuntimeError(
            'No D435i or Gemini 336L was detected. Connect exactly one calibration camera.')
    if device['speed_mbps'] < 5000:
        raise RuntimeError(
            f'{device["product"] or camera} is connected at {device["speed_mbps"]:.0f} Mbps. '
            'Use a USB 3.x port and cable (5000 Mbps or higher).')
    return camera, mode, device


def _has_piper_can_adapter(devices):
    return any(
        (item['vendor'], item['product_id']) in PIPER_CAN_ADAPTER_IDS
        for item in devices)


def _can_status(output):
    first_line = output.splitlines()[0] if output else ''
    flags = first_line.split('<', 1)[1].split('>', 1)[0].split(',') if '<' in first_line else []
    bitrate = None
    tokens = output.replace('\n', ' ').split()
    if 'bitrate' in tokens:
        try:
            bitrate = int(tokens[tokens.index('bitrate') + 1])
        except (ValueError, IndexError):
            pass
    return 'UP' in flags, bitrate


def _activate_can_interface(can_port, bitrate, command_runner):
    if not CAN_INTERFACE_PATTERN.fullmatch(can_port):
        raise RuntimeError(f'Invalid CAN interface name: {can_port!r}.')
    command = [
        'pkexec', '/bin/sh', '-c',
        '/usr/sbin/ip link set "$1" down && '
        '/usr/sbin/ip link set "$1" type can bitrate "$2" && '
        '/usr/sbin/ip link set "$1" up',
        'piper-can-setup', can_port, str(bitrate),
    ]
    return command_runner(command, check=False).returncode == 0


def require_can_interface(
        can_port, command_runner=subprocess.run, devices=None,
        expected_bitrate=PIPER_CAN_BITRATE, auto_activate=False,
        activation_runner=subprocess.run):
    result = command_runner(
        ['ip', '-details', 'link', 'show', can_port],
        capture_output=True, text=True, check=False)
    if result.returncode != 0:
        detected_devices = usb_devices() if devices is None else devices
        if not _has_piper_can_adapter(detected_devices):
            raise RuntimeError(
                'Piper USB-CAN adapter is not connected (expected USB ID '
                '1d50:606f). Reconnect the candleLight adapter, then run the '
                'launch command again.')
        raise RuntimeError(
            f'Piper USB-CAN adapter 1d50:606f is connected, but interface '
            f'{can_port} was not created. Replug the adapter and make sure the '
            'gs_usb kernel driver is loaded.')

    is_up, bitrate = _can_status(result.stdout)
    if auto_activate and (not is_up or bitrate != expected_bitrate):
        if not _activate_can_interface(can_port, expected_bitrate, activation_runner):
            raise RuntimeError(
                f'Administrator authorization to activate {can_port} failed. '
                'Run piper_ros/can_activate.sh manually, then start this launch again.')
        result = command_runner(
            ['ip', '-details', 'link', 'show', can_port],
            capture_output=True, text=True, check=False)
        if result.returncode != 0:
            raise RuntimeError(f'CAN interface {can_port} disappeared during activation.')
        is_up, bitrate = _can_status(result.stdout)

    if not is_up:
        raise RuntimeError(
            f'CAN interface {can_port} exists but is DOWN. Activate it at '
            f'{expected_bitrate} bit/s with piper_ros/can_activate.sh first.')

    if bitrate != expected_bitrate:
        displayed = 'unknown' if bitrate is None else str(bitrate)
        raise RuntimeError(
            f'CAN interface {can_port} bitrate is {displayed}; Piper requires '
            f'{expected_bitrate} bit/s. Reactivate the interface with '
            'piper_ros/can_activate.sh.')

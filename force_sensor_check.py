#!/usr/bin/env python3
"""Minimal RS485 checker for Zhongke Midian 6-axis force sensors."""

from __future__ import annotations

import argparse
import array
import fcntl
import math
import os
import select
import struct
import sys
import termios
import time
from pathlib import Path


HEADER = b"\xAA\x55"
TAIL = b"\x0D\x0A"
FRAME_LEN = 29
COMMAND_OFFSET = 2
PAYLOAD_OFFSET = 3
PAYLOAD_LEN = 24
TAIL_OFFSET = PAYLOAD_OFFSET + PAYLOAD_LEN
CRC_OFFSET = None
FIELDS = ("Fx", "Fy", "Fz", "Mx", "My", "Mz")

CMD_STOP = bytes.fromhex("AA 55 01 0D 0A")
CMD_READ_ONCE = bytes.fromhex("AA 55 03 0D 0A")
CMD_UNIT_KG = bytes.fromhex("AA 55 33 0D 0A")
CMD_UNIT_N = bytes.fromhex("AA 55 35 0D 0A")
CMD_QUERY_INFO = bytes.fromhex("AA 55 05 0D 0A")

DEFAULT_PORT = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG03384N-if00-port0"
BAUD_TO_TERMIOS = {
    460800: termios.B460800,
    921600: termios.B921600,
}
TCGETS2 = 0x802C542A
TCSETS2 = 0x402C542B
BOTHER = 0x1000
CBAUD = 0x100F
SAFE_PROBE_COMMANDS = (
    ("query-info 0x05", CMD_QUERY_INFO),
    ("unit-N 0x35", CMD_UNIT_N),
    ("read-once 0x03", CMD_READ_ONCE),
)


class ForceSensorError(RuntimeError):
    pass


def frame_hex(data: bytes) -> str:
    return data.hex(" ").upper()


def parse_force_response(frame: bytes) -> dict[str, object]:
    """Parse a 0x03 force response.

    Protocol V1.4 frame layout:
    AA 55 | command | 24-byte payload | 0D 0A
    There is no CRC field in this 29-byte response.
    """
    if len(frame) != FRAME_LEN:
        raise ForceSensorError(f"bad frame length: {len(frame)} bytes, expected {FRAME_LEN}")
    if not frame.startswith(HEADER):
        raise ForceSensorError(f"bad frame header: {frame_hex(frame[:2])}")
    if not frame.endswith(TAIL):
        raise ForceSensorError(f"bad frame tail: {frame_hex(frame[-2:])}")

    command = frame[COMMAND_OFFSET]
    payload = frame[PAYLOAD_OFFSET:TAIL_OFFSET]
    if len(payload) != PAYLOAD_LEN:
        raise ForceSensorError(f"bad payload length: {len(payload)} bytes, expected {PAYLOAD_LEN}")

    values = struct.unpack("<6f", payload)
    return {
        "raw": frame,
        "command": command,
        "payload": payload,
        "crc": None,
        "values": dict(zip(FIELDS, values)),
    }


def parse_force_frame(frame: bytes) -> dict[str, float]:
    parsed = parse_force_response(frame)
    return parsed["values"]


def extract_force_frames(buffer: bytearray) -> list[bytes]:
    frames: list[bytes] = []
    while True:
        start = buffer.find(HEADER)
        if start < 0:
            if buffer.endswith(HEADER[:1]):
                del buffer[:-1]
                return frames
            del buffer[:]
            return frames
        if start:
            del buffer[:start]
        if len(buffer) < FRAME_LEN:
            return frames

        candidate = bytes(buffer[:FRAME_LEN])
        if candidate[-2:] == TAIL:
            frames.append(candidate)
            del buffer[:FRAME_LEN]
            continue

        del buffer[0]


def configure_serial(fd: int, baud: int) -> None:
    speed = BAUD_TO_TERMIOS.get(baud)
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
    attrs[3] = 0
    if speed is not None:
        attrs[4] = speed
        attrs[5] = speed
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    if speed is None:
        set_custom_baud(fd, baud)
    termios.tcflush(fd, termios.TCIOFLUSH)


def set_custom_baud(fd: int, baud: int) -> None:
    buf = array.array("I", [0] * 64)
    try:
        fcntl.ioctl(fd, TCGETS2, buf, True)
        buf[2] &= ~CBAUD
        buf[2] |= BOTHER | termios.CLOCAL | termios.CREAD | termios.CS8
        buf[9] = baud
        buf[10] = baud
        fcntl.ioctl(fd, TCSETS2, buf)
    except OSError as exc:
        supported = ", ".join(str(item) for item in sorted(BAUD_TO_TERMIOS))
        raise ForceSensorError(
            f"unsupported baud {baud}; standard supported: {supported}; custom baud failed: {exc}"
        ) from exc


def open_serial(path: str, baud: int) -> int:
    if not os.path.exists(path):
        raise ForceSensorError(f"serial port not found: {path}")
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        configure_serial(fd, baud)
    except Exception:
        os.close(fd)
        raise
    return fd


def read_available(fd: int, timeout: float) -> bytes:
    end = time.monotonic() + timeout
    chunks: list[bytes] = []
    while True:
        remaining = end - time.monotonic()
        if remaining <= 0:
            break
        ready, _, _ = select.select([fd], [], [], remaining)
        if not ready:
            break
        chunk = os.read(fd, 4096)
        if chunk:
            chunks.append(chunk)
    return b"".join(chunks)


def write_command(fd: int, command: bytes, settle: float = 0.03) -> None:
    os.write(fd, command)
    time.sleep(settle)


def wait_for_force_frame(fd: int, timeout: float, buffer: bytearray) -> bytes | None:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        for frame in extract_force_frames(buffer):
            return frame
        chunk = read_available(fd, max(0.0, end - time.monotonic()))
        if chunk:
            buffer.extend(chunk)
    for frame in extract_force_frames(buffer):
        return frame
    return None


def print_force_response(parsed: dict[str, object], unit: str) -> None:
    values = parsed["values"]
    if not isinstance(values, dict):
        raise ForceSensorError("internal parse error: values is not a dict")

    print(f"raw hex: {frame_hex(parsed['raw'])}")
    print(f"payload hex: {frame_hex(parsed['payload'])}")
    parts = []
    for name in FIELDS:
        value = values[name]
        if not math.isfinite(value):
            parts.append(f"{name}=nan")
        else:
            parts.append(f"{name}={value: .6f}")
    print("  ".join(parts) + f"  unit={unit}")


def print_values(values: dict[str, float], unit: str) -> None:
    parts = []
    for name in FIELDS:
        value = values[name]
        if not math.isfinite(value):
            parts.append(f"{name}=nan")
        else:
            parts.append(f"{name}={value: .6f}")
    print("  ".join(parts) + f"  unit={unit}")


def list_ports() -> None:
    paths = sorted(Path("/dev").glob("ttyUSB*")) + sorted(Path("/dev").glob("ttyACM*"))
    by_id = Path("/dev/serial/by-id")
    print("Serial ports:")
    if not paths:
        print("  none found")
    for path in paths:
        try:
            stat = path.stat()
            mode = oct(stat.st_mode & 0o777)
        except OSError:
            mode = "?"
        print(f"  {path}  mode={mode}")

    print("Stable names:")
    if not by_id.exists():
        print("  none found")
        return
    names = sorted(by_id.iterdir())
    if not names:
        print("  none found")
    for name in names:
        try:
            target = os.readlink(name)
        except OSError:
            target = "?"
        print(f"  {name} -> {target}")


def probe_port(path: str, bauds: list[int], timeout: float) -> int:
    any_response = False
    for baud in bauds:
        print(f"== {path} @ {baud} 8N1 ==")
        try:
            fd = open_serial(path, baud)
        except Exception as exc:
            print(f"open failed: {exc}")
            continue

        try:
            for label, command in SAFE_PROBE_COMMANDS:
                termios.tcflush(fd, termios.TCIOFLUSH)
                write_command(fd, command, settle=0.05)
                data = read_available(fd, timeout)
                if data:
                    any_response = True
                print(f"{label}: {len(data)} bytes {frame_hex(data)}")
        finally:
            os.close(fd)
    return 0 if any_response else 2


def run(args: argparse.Namespace) -> int:
    if args.list_ports:
        list_ports()
        return 0
    if args.probe:
        return probe_port(args.port, args.probe_baud, args.timeout)

    fd = open_serial(args.port, args.baud)
    buffer = bytearray()
    unit = "N/Nm" if args.unit == "n" else "kg/kgm"
    try:
        if args.stop_first:
            write_command(fd, CMD_STOP)
            read_available(fd, 0.1)

        write_command(fd, CMD_UNIT_N if args.unit == "n" else CMD_UNIT_KG)
        ack = read_available(fd, 0.15)
        if args.verbose and ack:
            print(f"unit response: {frame_hex(ack)}", file=sys.stderr)

        if args.info:
            write_command(fd, CMD_QUERY_INFO)
            info = read_available(fd, args.timeout)
            print(f"info response ({len(info)} bytes): {frame_hex(info)}")

        for index in range(args.count):
            write_command(fd, CMD_READ_ONCE)
            frame = wait_for_force_frame(fd, args.timeout, buffer)
            if frame is None:
                print(
                    f"read {index + 1}: timeout, no valid {FRAME_LEN}-byte force frame received",
                    file=sys.stderr,
                )
                if args.verbose and buffer:
                    print(f"buffer: {frame_hex(bytes(buffer))}", file=sys.stderr)
                return 2
            parsed = parse_force_response(frame)
            print_force_response(parsed, unit)
            if index + 1 < args.count:
                time.sleep(args.interval)
    finally:
        os.close(fd)
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Read six-axis force data from the RS485 force sensor."
    )
    parser.add_argument("--port", default=DEFAULT_PORT, help=f"serial port, default: {DEFAULT_PORT}")
    parser.add_argument("--baud", type=int, default=460800, help="baud rate, default: 460800")
    parser.add_argument("--count", type=int, default=10, help="number of samples to read")
    parser.add_argument("--timeout", type=float, default=0.5, help="seconds to wait for each frame")
    parser.add_argument("--interval", type=float, default=0.1, help="seconds between reads")
    parser.add_argument("--unit", choices=("n", "kg"), default="n", help="output unit command")
    parser.add_argument("--stop-first", action="store_true", help="send stop command before reading")
    parser.add_argument("--info", action="store_true", help="also query sensor info once")
    parser.add_argument("--probe", action="store_true", help="send safe probe commands and print raw replies")
    parser.add_argument(
        "--probe-baud",
        type=int,
        action="append",
        default=None,
        help="baud rate to use with --probe; can be repeated",
    )
    parser.add_argument("--list-ports", action="store_true", help="print detected serial ports and exit")
    parser.add_argument("--verbose", action="store_true", help="print raw frames to stderr")
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    if args.probe_baud is None:
        args.probe_baud = [460800, 691200, 921600]
    if args.count < 1:
        parser.error("--count must be >= 1")
    try:
        return run(args)
    except PermissionError as exc:
        print(f"permission denied: {exc}", file=sys.stderr)
        print("Try: sudo usermod -aG dialout $USER, then log out and back in.", file=sys.stderr)
        return 1
    except ForceSensorError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    except OSError as exc:
        print(f"serial error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

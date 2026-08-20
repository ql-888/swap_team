import math

from piper_pink.units import protocol_to_radians, radians_to_protocol


def test_protocol_round_trip():
    raw = [0, 90_000, -90_000, 12_345, -54_321, 120_000]
    assert radians_to_protocol(protocol_to_radians(raw)) == raw


def test_one_radian_conversion():
    encoded = radians_to_protocol([1.0])[0]
    assert encoded == round(math.degrees(1.0) * 1000.0)


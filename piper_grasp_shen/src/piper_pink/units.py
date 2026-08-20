"""Unit conversion at the Pinocchio/PiPER SDK boundary."""

from __future__ import annotations

import math
from collections.abc import Iterable


PROTOCOL_UNITS_PER_DEGREE = 1000.0


def protocol_to_radians(values: Iterable[int | float]) -> list[float]:
    """Convert SDK joint feedback in 0.001 degrees to radians."""

    return [math.radians(float(value) / PROTOCOL_UNITS_PER_DEGREE) for value in values]


def radians_to_protocol(values: Iterable[int | float]) -> list[int]:
    """Convert radians to the SDK's signed 0.001-degree joint command."""

    return [round(math.degrees(float(value)) * PROTOCOL_UNITS_PER_DEGREE) for value in values]

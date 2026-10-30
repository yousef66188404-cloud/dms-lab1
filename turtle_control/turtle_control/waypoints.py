"""Waypoints of the shapes the turtle can draw.

Design pattern: Factory Method - ``WaypointFactory.create(shape, ...)`` hides how each shape is
built, so a new shape is one new function plus one registry entry (the square server does not change).
"""
import math
from typing import Callable, Dict, List

from turtle_control.motion import Target


def _square(x0: float, y0: float, side: float) -> List[Target]:
    """4 corners, counter-clockwise, starting to the right. The yaw of each goal is the
    direction of the NEXT side, so the turtle is already facing it. The last goal is the start."""
    return [
        Target(x0 + side, y0, math.pi / 2),
        Target(x0 + side, y0 + side, math.pi),
        Target(x0, y0 + side, -math.pi / 2),
        Target(x0, y0, 0.0),
    ]


class WaypointFactory:
    _builders: Dict[str, Callable[[float, float, float], List[Target]]] = {
        'square': _square,
    }

    @classmethod
    def create(cls, shape: str, origin_x: float, origin_y: float, size: float) -> List[Target]:
        try:
            builder = cls._builders[shape]
        except KeyError:
            raise ValueError(f"unknown shape '{shape}', available: {sorted(cls._builders)}")
        return builder(origin_x, origin_y, size)

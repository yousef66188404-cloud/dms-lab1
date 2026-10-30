"""Motion logic of the goToPose action. Pure Python (no ROS imports) so it can be unit-tested anywhere.

Design patterns used here:
  * Strategy - ``Controller`` decides the speed for a given error; ``ProportionalController``
    is one strategy and can be swapped without touching the states.
  * State    - ``RotateToTarget`` -> ``DriveToTarget`` -> ``RotateToYaw`` are the phases of a
    goToPose motion; each state knows its own behaviour and which state comes next.
"""
import math
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional, Tuple

from turtle_control.constants import (
    DEFAULT_ANGLE_TOLERANCE,
    DEFAULT_POSITION_TOLERANCE,
    MAX_ALLOWED_SPEED,
)


def wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    theta: float


@dataclass(frozen=True)
class Target:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Limits:
    max_linear: float
    max_angular: float


@dataclass(frozen=True)
class Tolerances:
    position: float = DEFAULT_POSITION_TOLERANCE
    angle: float = DEFAULT_ANGLE_TOLERANCE
    heading: float = 0.05  # how well we must face the target before driving


@dataclass(frozen=True)
class Command:
    linear: float = 0.0
    angular: float = 0.0


STOP = Command()


def validate_positive(value, upper: float = MAX_ALLOWED_SPEED) -> Tuple[bool, str]:
    """Check a speed/tolerance value: a finite number with 0 < value <= upper."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False, 'must be a number'
    if not math.isfinite(value):
        return False, 'must be finite'
    if value <= 0.0:
        return False, 'must be greater than 0'
    if value > upper:
        return False, f'must be at most {upper}'
    return True, ''


# ----------------------------------------------------------------------------------------
# Strategy pattern: how fast do we move for a given error?
# ----------------------------------------------------------------------------------------
class Controller(ABC):
    @abstractmethod
    def linear(self, distance: float, limits: Limits) -> float:
        """Linear speed (>= 0) for a remaining distance."""

    @abstractmethod
    def angular(self, error: float, limits: Limits) -> float:
        """Angular speed (signed) for an angle error in [-pi, pi]."""


class ProportionalController(Controller):
    """P controller, saturated by the speed limits (which are read live from the parameters)."""

    def __init__(self, kp_linear: float = 2.0, kp_angular: float = 4.0):
        self.kp_linear = kp_linear
        self.kp_angular = kp_angular

    def linear(self, distance, limits):
        return min(self.kp_linear * distance, limits.max_linear)

    def angular(self, error, limits):
        return max(-limits.max_angular, min(self.kp_angular * error, limits.max_angular))


# ----------------------------------------------------------------------------------------
# State pattern: the three phases of a goToPose motion.
# ----------------------------------------------------------------------------------------
@dataclass
class MotionContext:
    target: Target
    controller: Controller
    tolerances: Tolerances


def _distance_and_heading_error(pose: Pose, target: Target) -> Tuple[float, float]:
    dx, dy = target.x - pose.x, target.y - pose.y
    return math.hypot(dx, dy), wrap_angle(math.atan2(dy, dx) - pose.theta)


class MotionState(ABC):
    name = ''

    @abstractmethod
    def update(self, pose: Pose, limits: Limits,
               ctx: MotionContext) -> Tuple[Command, Optional['MotionState']]:
        """Return the command to publish and the next state (``None`` = motion finished)."""


class RotateToTarget(MotionState):
    """Turn on the spot until the turtle faces the target point."""
    name = 'rotate_to_target'

    def update(self, pose, limits, ctx):
        distance, error = _distance_and_heading_error(pose, ctx.target)
        if distance < ctx.tolerances.position:
            return STOP, RotateToYaw()          # already at the point: only the yaw is left
        if abs(error) < ctx.tolerances.heading:
            return STOP, DriveToTarget()
        return Command(0.0, ctx.controller.angular(error, limits)), self


class DriveToTarget(MotionState):
    """Drive to the point, correcting the heading on the way."""
    name = 'drive_to_target'

    def update(self, pose, limits, ctx):
        distance, error = _distance_and_heading_error(pose, ctx.target)
        if distance < ctx.tolerances.position:
            return STOP, RotateToYaw()
        # slow down the forward speed when the heading is off (cos < 0 -> do not drive backwards)
        linear = ctx.controller.linear(distance, limits) * max(0.0, math.cos(error))
        return Command(linear, ctx.controller.angular(error, limits)), self


class RotateToYaw(MotionState):
    """Turn on the spot to the final yaw."""
    name = 'rotate_to_yaw'

    def update(self, pose, limits, ctx):
        error = wrap_angle(ctx.target.yaw - pose.theta)
        if abs(error) < ctx.tolerances.angle:
            return STOP, None
        return Command(0.0, ctx.controller.angular(error, limits)), self


class GoToPoseMotion:
    """Runs the state machine. Call ``update`` at a fixed rate until ``finished`` is True."""

    def __init__(self, target: Target, controller: Optional[Controller] = None,
                 tolerances: Optional[Tolerances] = None):
        self._ctx = MotionContext(target, controller or ProportionalController(),
                                  tolerances or Tolerances())
        self._state: Optional[MotionState] = RotateToTarget()

    @property
    def finished(self) -> bool:
        return self._state is None

    @property
    def phase(self) -> str:
        return 'done' if self._state is None else self._state.name

    def update(self, pose: Pose, limits: Limits) -> Command:
        if self._state is None:
            return STOP
        command, self._state = self._state.update(pose, limits, self._ctx)
        return command

    def remaining(self, pose: Pose) -> Tuple[float, float]:
        """(distance to the target point, yaw error to the target yaw)."""
        target = self._ctx.target
        return (math.hypot(target.x - pose.x, target.y - pose.y),
                wrap_angle(target.yaw - pose.theta))

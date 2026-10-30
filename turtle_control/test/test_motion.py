"""Unit tests of the motion logic against a small simulation of turtlesim's kinematics.

Run with:  python -m pytest turtle_control/test   (or `colcon test --packages-select turtle_control`)
"""
import math

import pytest

from turtle_control.constants import (
    LOOP_PERIOD, MAX_SIDE, START_X, START_Y, WORLD_MAX, WORLD_MIN)
from turtle_control.motion import (
    GoToPoseMotion, Limits, Pose, Target, validate_positive, wrap_angle)
from turtle_control.waypoints import WaypointFactory

POSITION_TOL = 0.1   # tolerances asked by the assignment
ANGLE_TOL = 0.05


class FakeTurtle:
    """Same update rule as turtlesim: rotate first, then move along the new heading; walls clamp."""

    def __init__(self, x=START_X, y=START_Y, theta=0.0):
        self.pose = Pose(x, y, theta)

    def apply(self, command, dt=LOOP_PERIOD):
        theta = wrap_angle(self.pose.theta + command.angular * dt)
        x = self.pose.x + math.cos(theta) * command.linear * dt
        y = self.pose.y + math.sin(theta) * command.linear * dt
        x = min(max(x, WORLD_MIN), WORLD_MAX)
        y = min(max(y, WORLD_MIN), WORLD_MAX)
        self.pose = Pose(x, y, theta)


def run_goal(turtle, target, limits=Limits(2.0, 2.0), limits_at=None, max_time=60.0):
    """Drive the turtle to ``target``. Returns the list of (pose, command) of every tick."""
    motion = GoToPoseMotion(target)
    trace = []
    for tick in range(int(max_time / LOOP_PERIOD)):
        current = limits_at(tick) if limits_at else limits
        command = motion.update(turtle.pose, current)
        trace.append((turtle.pose, command, current))
        if motion.finished:
            return trace
        turtle.apply(command)
    raise AssertionError(f'goal {target} not reached in {max_time}s, pose={turtle.pose}')


def assert_reached(turtle, target):
    pose = turtle.pose
    assert math.hypot(pose.x - target.x, pose.y - target.y) < POSITION_TOL
    assert abs(wrap_angle(pose.theta - target.yaw)) < ANGLE_TOL


@pytest.mark.parametrize('x,y', [
    (8.0, 5.5), (2.0, 5.5), (5.5, 9.0), (5.5, 2.0),   # right, left, up, down
    (1.0, 1.0), (10.0, 10.0), (1.0, 10.0), (10.0, 1.0),
    (0.0, 0.0), (11.0, 11.0),                            # the corners of the window
])
@pytest.mark.parametrize('yaw', [0.0, math.pi / 2, math.pi, -math.pi / 2, 2.5, -3.0])
def test_reaches_pose_in_all_directions(x, y, yaw):
    turtle = FakeTurtle()
    target = Target(x, y, yaw)
    run_goal(turtle, target)
    assert_reached(turtle, target)


def test_goal_behind_the_turtle():
    turtle = FakeTurtle(theta=0.0)           # facing +x, goal is at -x
    target = Target(2.0, START_Y, 0.0)
    trace = run_goal(turtle, target)
    assert_reached(turtle, target)
    assert all(command.linear >= 0.0 for _, command, _ in trace)   # never drives backwards


def test_goal_where_the_turtle_already_is():
    turtle = FakeTurtle()
    target = Target(START_X, START_Y, 1.0)   # only a rotation
    run_goal(turtle, target)
    assert_reached(turtle, target)


def test_speed_limits_are_respected_and_changes_apply_live():
    turtle = FakeTurtle()
    target = Target(10.0, 10.0, 0.0)
    # the limits drop to 0.5 after 1 second (tick 50)
    limits_at = lambda tick: Limits(2.0, 2.0) if tick < 50 else Limits(0.5, 0.5)
    trace = run_goal(turtle, target, limits_at=limits_at)
    assert_reached(turtle, target)
    for _, command, limits in trace:
        assert abs(command.linear) <= limits.max_linear + 1e-9
        assert abs(command.angular) <= limits.max_angular + 1e-9
    assert max(c.linear for _, c, _ in trace[60:]) <= 0.5 + 1e-9


def test_lower_speed_takes_longer():
    fast = run_goal(FakeTurtle(), Target(9.0, 9.0, 0.0), Limits(3.0, 3.0))
    slow = run_goal(FakeTurtle(), Target(9.0, 9.0, 0.0), Limits(0.5, 0.5))
    assert len(slow) > 2 * len(fast)


def test_square_four_goals_equal_sides_and_inside_window():
    side = 3.0
    waypoints = WaypointFactory.create('square', START_X, START_Y, side)
    assert len(waypoints) == 4
    turtle = FakeTurtle()
    visited = [(turtle.pose.x, turtle.pose.y)]
    for waypoint in waypoints:
        trace = run_goal(turtle, waypoint)
        assert_reached(turtle, waypoint)
        visited += [(pose.x, pose.y) for pose, _, _ in trace]
        # each side is a straight line of length `side`
    corners = [(START_X, START_Y)] + [(w.x, w.y) for w in waypoints]
    sides = [math.dist(corners[i], corners[i + 1]) for i in range(4)]
    assert all(s == pytest.approx(side) for s in sides)
    assert all(WORLD_MIN <= x <= WORLD_MAX and WORLD_MIN <= y <= WORLD_MAX for x, y in visited)


def test_biggest_square_stays_inside_window():
    waypoints = WaypointFactory.create('square', START_X, START_Y, MAX_SIDE)
    assert all(WORLD_MIN <= w.x <= WORLD_MAX and WORLD_MIN <= w.y <= WORLD_MAX for w in waypoints)


def test_square_path_is_straight_between_corners():
    """While driving a side, the turtle must stay on the line between the two corners."""
    waypoints = WaypointFactory.create('square', START_X, START_Y, 3.0)
    turtle = FakeTurtle()
    start = (START_X, START_Y)
    for waypoint in waypoints:
        trace = run_goal(turtle, waypoint)
        end = (waypoint.x, waypoint.y)
        for pose, command, _ in trace:
            if command.linear > 0.0:
                # distance from the point to the segment start-end
                px, py = pose.x - start[0], pose.y - start[1]
                ex, ey = end[0] - start[0], end[1] - start[1]
                cross = abs(px * ey - py * ex) / math.hypot(ex, ey)
                assert cross < 0.1
        start = end


def test_factory_rejects_unknown_shape():
    with pytest.raises(ValueError):
        WaypointFactory.create('hexagon', START_X, START_Y, 1.0)


@pytest.mark.parametrize('value,ok', [
    (1.0, True), (2, True), (0.0, False), (0, False), (-1.0, False),
    (float('nan'), False), (float('inf'), False), (100.0, False), ('fast', False), (True, False),
])
def test_validate_positive(value, ok):
    assert validate_positive(value)[0] is ok

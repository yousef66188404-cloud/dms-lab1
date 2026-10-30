"""goToPose action server + speed parameters + set_speed_limits service (the node that moves the turtle)."""
import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.parameter import Parameter

from turtle_control import constants as C
from turtle_control.motion import (
    STOP, Command, GoToPoseMotion, Limits, ProportionalController, Target, Tolerances,
    validate_positive)
from turtle_control.pose_tracker import PoseTracker
from turtle_control.ros_utils import spin_node
from turtle_interfaces.action import GoToPose
from turtle_interfaces.srv import SetSpeedLimits

POSE_TIMEOUT = 2.0      # seconds without a pose message before we abort
FEEDBACK_EVERY = 5      # control ticks between feedback messages (10 Hz)
SPEED_PARAMS = ('max_linear_speed', 'max_angular_speed')
TOLERANCE_PARAMS = ('position_tolerance', 'angle_tolerance')


class GoToPoseServer(Node):
    def __init__(self):
        super().__init__('goto_pose_server')
        group = ReentrantCallbackGroup()
        self._controller = ProportionalController()
        self._lock = threading.Lock()
        self._busy = False
        self._goal_count = 0
        self._tracker = PoseTracker('goto_pose_listener')

        # dynamic_typing: `ros2 param set ... 1` (an integer) is accepted too.
        def descriptor(text):
            return ParameterDescriptor(description=text, dynamic_typing=True)

        self.declare_parameter('max_linear_speed', C.DEFAULT_MAX_LINEAR_SPEED,
                               descriptor('Maximum linear speed in units/s (> 0).'))
        self.declare_parameter('max_angular_speed', C.DEFAULT_MAX_ANGULAR_SPEED,
                               descriptor('Maximum angular speed in rad/s (> 0).'))
        self.declare_parameter('position_tolerance', C.DEFAULT_POSITION_TOLERANCE,
                               descriptor('Distance (units) at which the target counts as reached.'))
        self.declare_parameter('angle_tolerance', C.DEFAULT_ANGLE_TOLERANCE,
                               descriptor('Yaw error (rad) at which the final yaw counts as reached.'))
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self._cmd_pub = self.create_publisher(Twist, C.CMD_VEL_TOPIC, 10)
        self.create_service(SetSpeedLimits, 'set_speed_limits', self._on_set_speed_limits,
                            callback_group=group)
        self._action_server = ActionServer(
            self, GoToPose, 'go_to_pose',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=group)
        self.get_logger().info('goToPose server ready (action: go_to_pose, service: set_speed_limits)')

    # ------------------------------------------------------------------ parameters
    def _on_set_parameters(self, parameters):
        """Single place that validates every change: `ros2 param set` and our service both end here."""
        for parameter in parameters:
            if parameter.name in SPEED_PARAMS:
                ok, reason = validate_positive(parameter.value)
            elif parameter.name in TOLERANCE_PARAMS:
                ok, reason = validate_positive(parameter.value, upper=1.0)
            else:
                continue
            if not ok:
                return SetParametersResult(
                    successful=False, reason=f"'{parameter.name}' {reason} (got {parameter.value!r})")
        return SetParametersResult(successful=True)

    def _read_limits(self) -> Limits:
        """Read the speed parameters. Called at every control tick so changes apply live."""
        values = []
        for name, default in (('max_linear_speed', C.DEFAULT_MAX_LINEAR_SPEED),
                              ('max_angular_speed', C.DEFAULT_MAX_ANGULAR_SPEED)):
            value = self.get_parameter(name).value
            if not validate_positive(value)[0]:  # e.g. a bad value coming from a YAML file
                self.get_logger().warn(f"invalid '{name}' = {value!r}, using {default}",
                                       throttle_duration_sec=5.0)
                value = default
            values.append(float(value))
        return Limits(*values)

    def _read_tolerances(self) -> Tolerances:
        position = self.get_parameter('position_tolerance').value
        angle = self.get_parameter('angle_tolerance').value
        if not (validate_positive(position, 1.0)[0] and validate_positive(angle, 1.0)[0]):
            return Tolerances()
        return Tolerances(position=float(position), angle=float(angle))

    def _on_set_speed_limits(self, request, response):
        result = self.set_parameters_atomically([
            Parameter('max_linear_speed', Parameter.Type.DOUBLE, float(request.max_linear_speed)),
            Parameter('max_angular_speed', Parameter.Type.DOUBLE, float(request.max_angular_speed)),
        ])
        response.success = result.successful
        if result.successful:
            response.message = (f'Speed limits set: linear={request.max_linear_speed:.3f}, '
                                f'angular={request.max_angular_speed:.3f}')
            self.get_logger().info(response.message)
        else:
            response.message = f'Rejected: {result.reason}'
            self.get_logger().warn(response.message)
        return response

    # ------------------------------------------------------------------ turtle I/O
    def _publish(self, command: Command):
        msg = Twist()
        msg.linear.x = command.linear
        msg.angular.z = command.angular
        self._cmd_pub.publish(msg)

    # ------------------------------------------------------------------ action server
    def _on_goal(self, goal_request):
        target = goal_request.target
        values = (target.x, target.y, target.yaw)
        if not all(math.isfinite(v) for v in values):
            self.get_logger().warn('Goal rejected: x, y and yaw must be finite numbers')
            return GoalResponse.REJECT
        if not (C.WORLD_MIN <= target.x <= C.WORLD_MAX and C.WORLD_MIN <= target.y <= C.WORLD_MAX):
            self.get_logger().warn(
                f'Goal rejected: ({target.x:.2f}, {target.y:.2f}) is outside the window '
                f'[{C.WORLD_MIN}, {C.WORLD_MAX}]')
            return GoalResponse.REJECT
        with self._lock:
            if self._busy:
                self.get_logger().warn('Goal rejected: the turtle is already executing a goal')
                return GoalResponse.REJECT
            self._busy = True
            self._goal_count += 1
            count = self._goal_count
        self.get_logger().info(
            f'Goal #{count} accepted: x={target.x:.2f} y={target.y:.2f} yaw={target.yaw:.2f}')
        return GoalResponse.ACCEPT

    def _execute(self, goal_handle):
        try:
            return self._drive(goal_handle)
        finally:
            self._publish(STOP)
            with self._lock:
                self._busy = False

    def _drive(self, goal_handle):
        request = goal_handle.request.target
        target = Target(float(request.x), float(request.y), float(request.yaw))
        motion = GoToPoseMotion(target, self._controller, self._read_tolerances())
        feedback = GoToPose.Feedback()
        result = GoToPose.Result()
        started = time.monotonic()
        pose = self._tracker.pose
        tick = 0
        next_tick = time.monotonic()

        while rclpy.ok():
            pose = self._tracker.pose
            if pose is None or self._tracker.age > POSE_TIMEOUT:
                return self._finish(goal_handle, result, motion, pose, 'abort',
                                    'No pose from turtlesim (is turtlesim_node running?)')
            if goal_handle.is_cancel_requested:
                return self._finish(goal_handle, result, motion, pose, 'cancel', 'Goal canceled')
            if time.monotonic() - started > C.GOAL_TIMEOUT:
                return self._finish(goal_handle, result, motion, pose, 'abort',
                                    f'Timeout: not reached after {C.GOAL_TIMEOUT:.0f} s')

            self._publish(motion.update(pose, self._read_limits()))
            if motion.finished:
                return self._finish(goal_handle, result, motion, pose, 'succeed', 'Pose reached')

            if tick % FEEDBACK_EVERY == 0:
                feedback.distance_remaining, feedback.yaw_remaining = motion.remaining(pose)
                feedback.phase = motion.phase
                goal_handle.publish_feedback(feedback)
            tick += 1
            next_tick += C.LOOP_PERIOD   # fixed rate: no drift, no jitter from the loop body
            time.sleep(max(0.0, next_tick - time.monotonic()))

        return self._finish(goal_handle, result, motion, pose, 'abort', 'Node shutting down')

    def _finish(self, goal_handle, result, motion, pose, outcome, message):
        if pose is not None:
            result.distance_error, result.yaw_error = (float(v) for v in motion.remaining(pose))
        result.success = outcome == 'succeed'
        result.message = message
        {'succeed': goal_handle.succeed, 'cancel': goal_handle.canceled,
         'abort': goal_handle.abort}[outcome]()
        log = self.get_logger().info if result.success else self.get_logger().warn
        log(f'Goal finished: {message}')
        return result

    def destroy_node(self):
        self._tracker.shutdown()
        super().destroy_node()


def main(args=None):
    spin_node(GoToPoseServer)


if __name__ == '__main__':
    main()

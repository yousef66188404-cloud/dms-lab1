"""draw_square action server: reset service first, then exactly 4 goals to the goToPose action server."""
import math
import threading
import time

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_srvs.srv import Trigger

from turtle_control import constants as C
from turtle_control.ros_utils import call_service, spin_node, wait_for_future
from turtle_control.waypoints import WaypointFactory
from turtle_interfaces.action import DrawSquare, GoToPose
from turtle_interfaces.msg import TargetPose

SERVER_WAIT = 10.0   # seconds to wait for the other servers
FEEDBACK_PERIOD = 0.1


class SquareServer(Node):
    def __init__(self):
        super().__init__('square_server')
        group = ReentrantCallbackGroup()
        self._goto_client = ActionClient(self, GoToPose, 'go_to_pose', callback_group=group)
        self._reset_client = self.create_client(Trigger, 'reset_turtle', callback_group=group)
        self._lock = threading.Lock()
        self._busy = False
        self._remaining = 0.0   # distance_remaining of the goToPose goal in progress
        self._server = ActionServer(
            self, DrawSquare, 'draw_square',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=group)
        self.get_logger().info('square server ready (action: draw_square)')

    def _on_goal(self, goal_request):
        side = goal_request.side_length
        if not math.isfinite(side) or side < 0.0 or side > C.MAX_SIDE:
            self.get_logger().warn(
                f'Goal rejected: side_length must be between 0 and {C.MAX_SIDE:.2f} (got {side})')
            return GoalResponse.REJECT
        with self._lock:
            if self._busy:
                self.get_logger().warn('Goal rejected: already drawing a square')
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    def _execute(self, goal_handle):
        try:
            return self._draw(goal_handle)
        finally:
            with self._lock:
                self._busy = False

    # ------------------------------------------------------------------ the square
    def _draw(self, goal_handle):
        side = goal_handle.request.side_length or C.DEFAULT_SIDE
        result = DrawSquare.Result()
        feedback = DrawSquare.Feedback()
        self.get_logger().info(f'Drawing a square with side {side:.2f}')

        # Step 1: clean screen and known start pose.
        if not self._reset_client.wait_for_service(timeout_sec=SERVER_WAIT):
            return self._end(goal_handle, result, 'abort', 0, 'reset_turtle service not available')
        reset = call_service(self._reset_client, Trigger.Request(), timeout=15.0)
        if reset is None or not reset.success:
            return self._end(goal_handle, result, 'abort', 0,
                             f'Reset failed: {reset.message if reset else "no answer"}')
        if not self._goto_client.wait_for_server(timeout_sec=SERVER_WAIT):
            return self._end(goal_handle, result, 'abort', 0, 'go_to_pose server not available')

        # Step 2: 4 goals, one after the other.
        corners = WaypointFactory.create('square', C.START_X, C.START_Y, side)
        for index, corner in enumerate(corners, start=1):
            feedback.current_side = index
            feedback.distance_remaining = 0.0
            goal_handle.publish_feedback(feedback)
            status, message = self._go_to(goal_handle, corner, index, feedback)
            if status == 'canceled':
                return self._end(goal_handle, result, 'cancel', index - 1, 'Square canceled')
            if status == 'failed':
                return self._end(goal_handle, result, 'abort', index - 1,
                                 f'Side {index} failed: {message}')
        return self._end(goal_handle, result, 'succeed', len(corners), 'Square drawn')

    def _go_to(self, goal_handle, corner, side_number, feedback):
        """Send ONE goToPose goal and wait for it. Returns (status, message)."""
        goal = GoToPose.Goal()
        goal.target = TargetPose(x=float(corner.x), y=float(corner.y), yaw=float(corner.yaw))
        send_future = self._goto_client.send_goal_async(goal, feedback_callback=self._on_feedback)
        if not wait_for_future(send_future, SERVER_WAIT):
            return 'failed', 'go_to_pose did not answer'
        goto_handle = send_future.result()
        if not goto_handle.accepted:
            return 'failed', 'go_to_pose rejected the goal'

        result_future = goto_handle.get_result_async()
        while not result_future.done() and rclpy.ok():
            if goal_handle.is_cancel_requested:
                goto_handle.cancel_goal_async()
                wait_for_future(result_future, 5.0)
                return 'canceled', ''
            feedback.current_side = side_number
            feedback.distance_remaining = float(self._remaining)
            goal_handle.publish_feedback(feedback)
            time.sleep(FEEDBACK_PERIOD)
        if not result_future.done():
            return 'failed', 'node shutting down'
        goto_result = result_future.result().result
        return ('ok', '') if goto_result.success else ('failed', goto_result.message)

    def _on_feedback(self, feedback_msg):
        self._remaining = feedback_msg.feedback.distance_remaining

    def _end(self, goal_handle, result, outcome, sides, message):
        result.success = outcome == 'succeed'
        result.sides_completed = sides
        result.message = message
        {'succeed': goal_handle.succeed, 'cancel': goal_handle.canceled,
         'abort': goal_handle.abort}[outcome]()
        log = self.get_logger().info if result.success else self.get_logger().warn
        log(message)
        return result


def main(args=None):
    spin_node(SquareServer)


if __name__ == '__main__':
    main()

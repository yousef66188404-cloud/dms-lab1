"""reset_turtle service: clear the screen and teleport the turtle back, using ONLY turtlesim's services."""
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_srvs.srv import Empty, Trigger

from turtle_control import constants as C
from turtle_control.ros_utils import call_service, spin_node
from turtle_control.turtlesim_compat import SetPen, TeleportAbsolute


class ResetService(Node):
    def __init__(self):
        super().__init__('reset_service')
        group = ReentrantCallbackGroup()
        self._clear = self.create_client(Empty, C.CLEAR_SERVICE, callback_group=group)
        self._teleport = self.create_client(TeleportAbsolute, C.TELEPORT_SERVICE, callback_group=group)
        self._pen = self.create_client(SetPen, C.SET_PEN_SERVICE, callback_group=group)
        self.create_service(Trigger, 'reset_turtle', self._on_reset, callback_group=group)
        self.get_logger().info('reset service ready (service: reset_turtle)')

    def _set_pen(self, off: bool):
        request = SetPen.Request()
        request.r, request.g, request.b, request.width = C.PEN_R, C.PEN_G, C.PEN_B, C.PEN_WIDTH
        request.off = 1 if off else 0
        return call_service(self._pen, request)

    def _on_reset(self, _request, response):
        clients = (self._clear, self._teleport, self._pen)
        if not all(client.wait_for_service(timeout_sec=2.0) for client in clients):
            response.success = False
            response.message = 'turtlesim services not available (is turtlesim_node running?)'
            self.get_logger().error(response.message)
            return response

        pen_is_off = False
        try:
            # 1. pen up, so teleporting does not draw a line
            pen_is_off = self._set_pen(off=True) is not None
            # 2. clear the screen
            if call_service(self._clear, Empty.Request()) is None:
                raise TimeoutError('clear service did not answer')
            # 3. back to the starting pose
            teleport = TeleportAbsolute.Request()
            teleport.x, teleport.y, teleport.theta = C.START_X, C.START_Y, C.START_YAW
            if call_service(self._teleport, teleport) is None:
                raise TimeoutError('teleport_absolute service did not answer')
            response.success = True
            response.message = 'Screen cleared, turtle back at the start pose'
        except TimeoutError as error:
            response.success = False
            response.message = str(error)
        finally:
            if pen_is_off:  # 4. pen down again, even if something failed
                self._set_pen(off=False)

        log = self.get_logger().info if response.success else self.get_logger().error
        log(response.message)
        return response


def main(args=None):
    spin_node(ResetService)


if __name__ == '__main__':
    main()

"""Import turtlesim's interfaces on any ROS 2 distro.

Humble/Iron: ``turtlesim.msg`` / ``turtlesim.srv``.  Jazzy and newer: ``turtlesim_msgs``.
"""
try:
    from turtlesim_msgs.msg import Pose
    from turtlesim_msgs.srv import SetPen, TeleportAbsolute
except ImportError:  # older distros
    from turtlesim.msg import Pose
    from turtlesim.srv import SetPen, TeleportAbsolute

__all__ = ['Pose', 'SetPen', 'TeleportAbsolute']

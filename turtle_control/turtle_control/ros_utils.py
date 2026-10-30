"""Small helpers shared by the nodes."""
import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor

try:
    from rclpy.executors import ExternalShutdownException
except ImportError:  # older rclpy
    ExternalShutdownException = KeyboardInterrupt


def wait_for_future(future, timeout=None) -> bool:
    """Block the calling thread until ``future`` is done (the executor must run in other threads)."""
    done = threading.Event()
    future.add_done_callback(lambda _: done.set())
    return done.wait(timeout)


def call_service(client, request, timeout=5.0):
    """Call a service from inside a callback without blocking the executor. None on timeout."""
    future = client.call_async(request)
    if not wait_for_future(future, timeout):
        return None
    return future.result()


def spin_node(node_factory, num_threads=4):
    """Create the node and spin it with several threads, so callbacks can wait for each other
    (e.g. an action server that calls a service) without deadlocking."""
    rclpy.init()
    node = None
    try:
        node = node_factory()
        executor = MultiThreadedExecutor(num_threads=num_threads)
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

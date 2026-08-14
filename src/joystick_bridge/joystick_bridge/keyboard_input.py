"""Terminal keyboard fallback -> /control_input, for testing without a
physical gamepad plugged in. Keys: '1' = stand up, '2' = sit down,
'0' = e-stop (torque off), Ctrl+C to quit.
"""
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from controller.msg import Inputs

CMD_NONE = 0
CMD_STAND = 1
CMD_SIT = 2
CMD_ESTOP = 9

_KEY_TO_COMMAND = {
    '1': CMD_STAND,
    '2': CMD_SIT,
    '0': CMD_ESTOP,
}


class KeyboardInput(Node):
    def __init__(self):
        super().__init__('keyboard_input_node')
        self._publisher = self.create_publisher(Inputs, '/control_input', 10)
        self.get_logger().info('keyboard_input started.')
        self.get_logger().info("Press '1' to stand up, '2' to sit down, '0' to e-stop.")
        self.get_logger().info('Ctrl+C to quit.')

    def publish(self, command: int):
        msg = Inputs()
        msg.command = command
        self._publisher.publish(msg)


def _read_key_nonblocking(timeout_s: float):
    ready, _, _ = select.select([sys.stdin], [], [], timeout_s)
    if ready:
        return sys.stdin.read(1)
    return None


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardInput()

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            key = _read_key_nonblocking(0.1)
            if key in _KEY_TO_COMMAND:
                node.publish(_KEY_TO_COMMAND[key])
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

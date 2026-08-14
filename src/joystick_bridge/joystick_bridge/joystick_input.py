"""Physical gamepad -> /control_input (controller/msg/Inputs).

Reads sensor_msgs/Joy (published by the standard `joy` package's joy_node,
which talks to whatever /dev/input/js* or evdev device the OS enumerates -
same node works unmodified on a dev PC running the simulation or on the RDK
X3 wired to real hardware). Only edge-triggers on button press (not every
/joy message, which repeats at ~50Hz while held) so a single press sends a
single command - repeating it every cycle is harmless (the FSM ignores a
request for the state it's already in) but noisy.

3 buttons:
- `toggle_button` - Sit <-> Stand, flips every press. First press sends Sit
  (assumes the robot starts at rest/passive, not mid-air standing), second
  press Stand, alternating from there. State is tracked locally in this node
  (no feedback topic from the real FSM state to read back).
- `home_button` - always sends Sit ("home" - smooth tanh interpolation like
  any Sit command, NOT an abrupt Estop/Passive drop) regardless of current
  posture - a single press is enough whether the robot was standing or
  already sitting (repeating Sit while already sitting is a harmless no-op,
  the FSM ignores a request for the state it's already in).
- `estop_button` - always separate from the other two, always instant.
  Disabled (never checked) if set to a negative index - useful while bench-
  testing with only 2 buttons wired up; DO NOT leave it disabled once this
  runs anything that can actually hurt itself or someone nearby.

Both `home_button` and `estop_button` resync `toggle_button`'s internal
next-press state back to Stand when pressed, so the toggle never gets out of
sync with reality just because one of the other two buttons moved the robot
without going through the toggle itself.

Default button indices match a typical Xbox-layout gamepad as seen through
`joy_node` (A=0, B=1, back/select=6). Check your own pad with:
    ros2 run joy joy_enumerate_devices
    ros2 topic echo /joy
and override via ROS2 params (toggle_button/home_button/estop_button) if it
differs - not guaranteed across every gamepad/driver combination.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from controller.msg import Inputs

CMD_NONE = 0
CMD_STAND = 1
CMD_SIT = 2
CMD_ESTOP = 9


class JoystickInput(Node):
    def __init__(self):
        super().__init__('joystick_input_node')

        self.declare_parameter('toggle_button', 0)
        self.declare_parameter('home_button', 1)
        self.declare_parameter('estop_button', 6)

        self._toggle_button = self.get_parameter('toggle_button').value
        self._home_button = self.get_parameter('home_button').value
        self._estop_button = self.get_parameter('estop_button').value

        self._prev_buttons = []
        # What the next toggle_button press should send - flips after every
        # toggle press, resynced to Stand whenever home_button or
        # estop_button fires (see module docstring). Starts at Sit so the
        # very first press means "stand up from rest".
        self._next_toggle_command = CMD_SIT
        self._publisher = self.create_publisher(Inputs, '/control_input', 10)
        self._subscription = self.create_subscription(Joy, '/joy', self._joy_callback, 10)

        estop_desc = f'estop_button={self._estop_button}' if self._estop_button >= 0 \
            else 'estop_button=disabled (negative index)'
        self.get_logger().info(
            f'joystick_input started - toggle_button={self._toggle_button} '
            f'(next press -> SIT), home_button={self._home_button} (-> SIT), {estop_desc}')

    def _pressed(self, buttons, index):
        """True only on the press edge (was 0/absent, now 1). index < 0 never matches
        (used to disable estop_button without Python's negative-index wraparound
        silently reading the wrong/last button)."""
        if index < 0:
            return False
        was_down = index < len(self._prev_buttons) and self._prev_buttons[index] == 1
        is_down = index < len(buttons) and buttons[index] == 1
        return is_down and not was_down

    def _joy_callback(self, msg: Joy):
        command = CMD_NONE
        if self._pressed(msg.buttons, self._estop_button):
            command = CMD_ESTOP
            self._next_toggle_command = CMD_STAND
        elif self._pressed(msg.buttons, self._home_button):
            command = CMD_SIT
            self._next_toggle_command = CMD_STAND
        elif self._pressed(msg.buttons, self._toggle_button):
            command = self._next_toggle_command
            self._next_toggle_command = (
                CMD_STAND if command == CMD_SIT else CMD_SIT)

        self._prev_buttons = list(msg.buttons)

        if command != CMD_NONE:
            out = Inputs()
            out.command = command
            self._publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = JoystickInput()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

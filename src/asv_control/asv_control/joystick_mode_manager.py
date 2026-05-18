#!/usr/bin/env python3
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String


class JoystickModeManager(Node):
    """Convert joystick buttons into ASV mode and emergency-stop topics."""

    def __init__(self):
        super().__init__("joystick_mode_manager")
        self.declare_parameter("manual_button", 4)
        self.declare_parameter("auto_button", 5)
        self.declare_parameter("estop_button", 1)
        self.declare_parameter("clear_estop_button", 0)

        self.manual_button = int(self.get_parameter("manual_button").value)
        self.auto_button = int(self.get_parameter("auto_button").value)
        self.estop_button = int(self.get_parameter("estop_button").value)
        self.clear_estop_button = int(self.get_parameter("clear_estop_button").value)

        self.estop = False
        self.previous_buttons = []
        self.mode_pub = self.create_publisher(String, "/asv/mode", 10)
        self.estop_pub = self.create_publisher(Bool, "/asv/emergency_stop", 10)
        self.create_subscription(Joy, "/joy", self.on_joy, 10)

    def on_joy(self, msg: Joy):
        # Button edges are used so one press sends one mode/stop command.
        if self._pressed(msg, self.manual_button):
            self.mode_pub.publish(String(data="manual"))
        if self._pressed(msg, self.auto_button):
            self.mode_pub.publish(String(data="auto"))
        if self._pressed(msg, self.estop_button):
            self.estop = True
        if self._pressed(msg, self.clear_estop_button):
            self.estop = False
        self.estop_pub.publish(Bool(data=self.estop))
        self.previous_buttons = list(msg.buttons)

    def _pressed(self, msg: Joy, index: int) -> bool:
        if not 0 <= index < len(msg.buttons):
            return False
        previous = self.previous_buttons[index] if index < len(self.previous_buttons) else 0
        return previous == 0 and msg.buttons[index] == 1


def main(args=None):
    rclpy.init(args=args)
    node = JoystickModeManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()

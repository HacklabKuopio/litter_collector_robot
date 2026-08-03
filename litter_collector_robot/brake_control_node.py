# encoding: utf-8

# Copyright 2023-2026 Mikael Lammentausta
#
# Licensed under the Apache License, Version 2.0.
# See the LICENSE file in the project root for license information.

import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity

from std_msgs.msg import Float32, Bool

try:
    import RPi.GPIO as GPIO
except:
    import Mock.GPIO as GPIO


class BrakeControlNode(Node):
    def __init__(self):
        super().__init__('brake_control_node')
        # Configure logger
        self.logger = self.get_logger()
        self.logger.set_level(LoggingSeverity.DEBUG)
        self.logger.info("BrakeControlNode init")

        # Read GPIO pin ports from config
        self.declare_parameter("brake_pin", rclpy.Parameter.Type.INTEGER)
        self.brake_pin = self.get_parameter("brake_pin").value
        self.declare_parameter("brake_feedback_pin", rclpy.Parameter.Type.INTEGER)
        self.brake_feedback_pin = self.get_parameter("brake_feedback_pin").value

        self.declare_parameter("brake_apply_delay", rclpy.Parameter.Type.DOUBLE)
        self.brake_apply_delay = self.get_parameter("brake_apply_delay").value

        # Init Raspberry Pi GPIO pins
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.brake_pin, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(self.brake_feedback_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        self.brake_state = True
        self.emergency_stop_triggered = False
        self.robot_movement_commanded_timer = None

        self.create_subscription(
            Bool,
            'lcr/robot_movement_commanded',
            self.robot_movement_commanded_callback,
            10
        )
        self.create_subscription(
            Bool,
            'lcr/emergency_stop_alert',
            self.emergency_stop_alert_callback,
            1
        )


    def __del__(self):
        GPIO.cleanup()


    def robot_movement_commanded_callback(self, msg):
        self.robot_movement_commanded = msg.data
        if not self.robot_movement_commanded:
            # apply brakes after timeout
            if self.robot_movement_commanded_timer and self.robot_movement_commanded_timer.is_canceled():
                # self.logger.debug("reset timer")
                self.robot_movement_commanded_timer.reset()
            elif not self.robot_movement_commanded_timer:
                # self.logger.debug("Create new timer")
                self.logger.info(f'Setting breaks on after {self.brake_apply_delay:0.1f} secs')
                self.robot_movement_commanded_timer = self.create_timer(self.brake_apply_delay, self.movement_rundown_timeout)

        elif self.brake_state:
            self.release_brakes()
            if self.robot_movement_commanded_timer and not self.robot_movement_commanded_timer.is_canceled():
                # self.logger.debug("cancel timer")
                self.robot_movement_commanded_timer.cancel()


    def movement_rundown_timeout(self):
        # movement ends this timeout ago and now brakes clatch on
        # self.logger.debug("movement rundown timeout")
        self.apply_brakes()


    def emergency_stop_alert_callback(self, msg):
        self.apply_brakes()


    def release_brakes(self):
        if self.brake_state:
            self.logger.info(f"Release brakes; output GPIO{self.brake_pin} HIGH", throttle_duration_sec=1)
            GPIO.output(self.brake_pin, GPIO.HIGH)
            self.brake_state = False


    def apply_brakes(self):
        if not self.brake_state:
            self.logger.info(f"Apply brakes; output GPIO{self.brake_pin} LOW", throttle_duration_sec=1)
            GPIO.output(self.brake_pin, GPIO.LOW)
            self.brake_state = True


    def check_brake_feedback(self):
        """Return True when brake_feedback_pin is HIGH"""
        return GPIO.input(self.brake_feedback_pin) == GPIO.HIGH


def main(args=None):
    rclpy.init(args=args)
    brake_control_node = BrakeControlNode()
    rclpy.spin(brake_control_node)
    brake_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

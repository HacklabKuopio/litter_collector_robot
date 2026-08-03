# encoding: utf-8

# Copyright 2023-2026 Mikael Lammentausta
#
# Licensed under the Apache License, Version 2.0.
# See the LICENSE file in the project root for license information.

from time import sleep

import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity

from std_msgs.msg import Bool
from example_interfaces.msg import String # for audio

try:
    import RPi.GPIO as GPIO
except:
    import Mock.GPIO as GPIO


class EmergencyStopNode(Node):
    """Emergency stop node.

    If any of the following is true, emergency stop triggers and issues alert topic
    and sets the emergency_stop_output pin LOW until reset message is received.

        (1) physical emergency stop (input) is triggered (GPIO state LOW)
            (the triggered state remains on even after the physical pin is set to HIGH)

        (2) software trigger via /lcr/emergency_stop_ds4_button topic

    The feature can be reset by /lcr/emergency_stop_reset topic, while robot is not being commanded to move.

    """
    def __init__(self):
        super().__init__('emergency_stop_node')
        # Configure logger
        self.logger = self.get_logger()
        self.logger.set_level(LoggingSeverity.INFO)
        self.logger.info("EmergencyStopNode init")

        # Read GPIO pin ports from config
        self.declare_parameter("emergency_stop_pin", rclpy.Parameter.Type.INTEGER)
        self.emergency_stop_pin = self.get_parameter("emergency_stop_pin").value
        self.declare_parameter("emergency_stop_output", rclpy.Parameter.Type.INTEGER)
        self.emergency_stop_output = self.get_parameter("emergency_stop_output").value

        # Init Raspberry Pi GPIO pins
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.emergency_stop_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) # during normal operation this is 1
        GPIO.setup(self.emergency_stop_output, GPIO.OUT, initial=GPIO.LOW) # during normal operation this is 1

        # Subscriptions
        self.create_subscription(
            Bool,
            'lcr/emergency_stop_ds4_button',
            self.emergency_stop_ds4_button_callback,
            1
        )
        self.create_subscription(
            Bool,
            'lcr/emergency_stop_reset',
            self.emergency_stop_reset_callback,
            1
        )
        # emergency stop alert publisher - this signal will trigger x times/sec
        self.emergency_stop_alert_pub = self.create_publisher(
            Bool,
            'lcr/emergency_stop_alert',
            1
        )
        self.audio_pub = self.create_publisher(
            String,
            'lcr/audio/play',
            100
        )

        self.emergency_stop_triggered = False

        # start timer to periodically run safety checks
        check_interval = 0.25
        self.create_timer(check_interval, self.main_safety_checks)


    def main_safety_checks(self):
        """Emergency stop safety checks.

            Emergency stop can be triggered from physical pin, or by ds4 button topic.
            Both signals set `self.emergency_stop_triggered` variable `True` when either of these is set.

            Once the triggered state is issued, returning the physical pin will not reset triggered state.

            Triggered state can be reset by `emergency_stop_reset` topic.
        """
        self.check_emergency_stop_pin()

        if self.emergency_stop_triggered:
            # emergency stop triggered
            self.logger.debug("Emergency alert", throttle_duration_sec=3)
            # publish alert: true message on /lcr/emergency_stop_alert topic
            self.emergency_stop_alert_pub.publish(Bool(data=True))
            # publish audio alert
            # self.audio_pub.publish(String(data="emergency_stop_state"))
        else:
            # normal operation
            #self.logger.debug("normal operation", throttle_duration_sec=1)
            # TODO: move blip elsewhere
            #self.audio_pub.publish(String(data="normal_state"))
            pass


    def check_emergency_stop_pin(self):
        """Trigger emergency stop from hardware GPIO pin.

        Check emergency_stop_pin, during normal operation this is HIGH.
        """
        # self.logger.debug(f"Check emergency_stop_pin; input GPIO{self.emergency_stop_pin}", throttle_duration_sec=1)
        if not GPIO.input(self.emergency_stop_pin):
            self.logger.debug("Emergency stop hardware input pin LOW", throttle_duration_sec=5)
            # play sound once when the state triggers
            if not self.emergency_stop_triggered:
                self.audio_pub.publish(String(data="emergency_stop_trigger"))
            self.emergency_stop_triggered = True
            self.deactivate_emergency_stop_output()


    def emergency_stop_ds4_button_callback(self, msg):
        """Trigger emergency stop from software."""
        self.logger.info("Emergency stop ds4 button", throttle_duration_sec=5)
        # play sound once when the state triggers
        if not self.emergency_stop_triggered:
            self.audio_pub.publish(String(data="emergency_stop_trigger"))
        self.emergency_stop_triggered = True
        self.deactivate_emergency_stop_output()


    def emergency_stop_reset_callback(self, msg):
        """Reset triggered emergency stop."""
        self.logger.info("Emergency stop reset", throttle_duration_sec=1)
        self.emergency_stop_triggered = False
        self.activate_emergency_stop_output()
        # publish alert: false message on /lcr/emergency_stop_alert topic to reset listeners
        self.emergency_stop_alert_pub.publish(Bool(data=False))
        # play sound
        self.audio_pub.publish(String(data="emergency_stop_reset"))


    def activate_emergency_stop_output(self):
        """Robot runs"""
        self.logger.info(f"Run; Emergency stop detriggered", throttle_duration_sec=5)
        GPIO.output(self.emergency_stop_output, GPIO.HIGH)
        sleep(0.25)


    def deactivate_emergency_stop_output(self):
        """Robot does not run"""
        self.logger.warning(f"Halt; Emergency stop triggered", throttle_duration_sec=5)
        GPIO.output(self.emergency_stop_output, GPIO.LOW)
        sleep(0.25)


def main(args=None):
    rclpy.init(args=args)
    emergency_stop_node = EmergencyStopNode()
    rclpy.spin(emergency_stop_node)
    emergency_stop_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

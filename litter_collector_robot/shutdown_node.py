# encoding: utf-8

# Copyright 2023-2026 Mikael Lammentausta
#
# Licensed under the Apache License, Version 2.0.
# See the LICENSE file in the project root for license information.

from os import system
from time import sleep

import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity

from std_msgs.msg import Bool
from example_interfaces.msg import String # for audio

class ShutdownNode(Node):
    def __init__(self):
        super().__init__('shutdown_node')
        # Configure logger
        self.logger = self.get_logger()
        self.logger.set_level(LoggingSeverity.DEBUG)
        self.logger.info("ShutdownNode init")

        # Read config
        self.declare_parameter("shutdown_timeout_sec", rclpy.Parameter.Type.DOUBLE)
        self.shutdown_timeout_sec = self.get_parameter("shutdown_timeout_sec").value

        # self.shutdown_clock = Clock()
        self.shutdown_issued_at = None

        # shutdown topic subscription
        self.create_subscription(
            Bool,
            'lcr/shutdown',
            self.shutdown_callback,
            10
        )

        # emergency stop publisher - ds4 button signal will trigger stop in brake control node
        self.emergency_stop_pub = self.create_publisher(
            Bool,
            'lcr/emergency_stop_ds4_button',
            1
        )
        # publish shutdown signal for comms (firewall)
        self.comms_shutdown_pub = self.create_publisher(
            Bool,
            'lcr/comms_shutdown',
            1
        )
        # play shutdown sound
        self.audio_pub = self.create_publisher(
            String,
            'lcr/audio/play',
            100
        )


    def shutdown_callback(self, msg):
        time_now = self.get_clock().now()
        shutdown_timeout_nsec = self.shutdown_timeout_sec * 1e9

        if not self.shutdown_issued_at:
            self.shutdown_issued_at = time_now

        elif (time_now - self.shutdown_issued_at).nanoseconds > shutdown_timeout_nsec * 1.1:
            # this should trigger only during testing, when shutdown does not halt normal operation
            self.cancel_shutdown()

        elif (time_now - self.shutdown_issued_at).nanoseconds > shutdown_timeout_nsec: # nsec
            # issue shutdown
            self.perform_shutdown()

        else:
            self.logger.debug(f'Shutdown wait ... {((time_now - self.shutdown_issued_at).nanoseconds / 1e9):.2f} secs', throttle_duration_sec=0.5)


    def perform_shutdown(self):
        """Perform actual shutdown"""
        # publish emergency stop signal
        self.emergency_stop_pub.publish(Bool(data=True))
        self.comms_shutdown_pub.publish(Bool(data=True))
        self.audio_pub.publish(String(data="unit_shutdown"))

        self.logger.warning('Shutdown ETA in 1 sec')
        sleep(1)
        self.logger.warning('Shutdown NOW')
        system("sudo systemctl poweroff")


    def cancel_shutdown(self):
        self.logger.info('Shutdown canceled')
        self.shutdown_issued_at = None


def main(args=None):
    rclpy.init(args=args)
    shutdown_node = ShutdownNode()
    rclpy.spin(shutdown_node)
    shutdown_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

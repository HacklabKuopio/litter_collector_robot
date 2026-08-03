# encoding: utf-8

# Copyright 2023-2026 Mikael Lammentausta
#
# Licensed under the Apache License, Version 2.0.
# See the LICENSE file in the project root for license information.

from os import system
from time import sleep

from smbus2 import SMBus
from ticlib import TicI2C, SMBus2Backend

import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity

from std_msgs.msg import Bool, Float32MultiArray
# from example_interfaces.msg import String # for audio

class NoseNode(Node):

    tic_z = None
    tic_y = None

    def __init__(self):
        super().__init__('nose_node')
        # Configure logger
        self.logger = self.get_logger()
        self.logger.set_level(LoggingSeverity.DEBUG)
        self.logger.info("NoseNode init")

        # declare parameters
        self.declare_parameter("i2c_bus", rclpy.Parameter.Type.INTEGER)
        self.i2c_bus = self.get_parameter("i2c_bus").value
        self.declare_parameter("i2c_z_address", rclpy.Parameter.Type.INTEGER)
        self.i2c_z_address = self.get_parameter("i2c_z_address").value
        self.declare_parameter("i2c_y_address", rclpy.Parameter.Type.INTEGER)
        self.i2c_y_address = self.get_parameter("i2c_y_address").value

        self.create_subscription(
            Float32MultiArray,
            'lcr/nose/cmd_vels',
            self.nose_command_callback,
            10
        )
        self.emergency_stop_subscription = self.create_subscription(
            Bool,
            'lcr/emergency_stop_alert',
            self.emergency_stop_alert_callback,
            1
        )

        self.init_smbus()


    def __del__(self):
        if self.tic_z:
            self.tic_z.enter_safe_start()
        if self.tic_y:
            self.tic_y.enter_safe_start()


    def init_smbus(self):
        try:
            bus = SMBus(self.i2c_bus)
            backend_z = SMBus2Backend(bus, self.i2c_z_address)
            backend_y = SMBus2Backend(bus, self.i2c_y_address)

            self.tic_z = TicI2C(backend_z)
            self.tic_y = TicI2C(backend_y)

            self.tic_z.energize()
            self.tic_y.energize()

            self.tic_z.energize()
            self.tic_y.energize()

            self.tic_z.exit_safe_start()
            self.tic_y.exit_safe_start()

        except Exception as err:
            self.logger.error(f"{err}")


    def nose_command_callback(self, values):
        # self.logger.info("nose movement commanded")
        nose_y_vel = int(values.data[0] * -100000000)
        nose_z_vel = int(values.data[1] * 200000000)
        if nose_y_vel != 0 or nose_z_vel != 0:
            self.logger.info(f'Nose move cmd {nose_y_vel} y, {nose_z_vel} z')
        try:
            self.tic_z.set_target_velocity(nose_z_vel)
            self.tic_y.set_target_velocity(nose_y_vel)
        except OSError as err:
            # self.logger.warn(err)
            pass


    def emergency_stop_alert_callback(self, value):
        pass
        # FIXME: This does not receive trigger after emergency state is cleared,
        #        so emergency trigger permanently stops the nose action
        # if value:
        #     self.tic_z.deenergize()
        #     self.tic_y.deenergize()
        #     self.logger.warn("Stepper motors de-energized")
        # else:
        #     self.tic_z.energize()
        #     self.tic_y.energize()
        #     self.logger.warn("Stepper motors energized")


def main(args=None):
    rclpy.init(args=args)
    nose_node = NoseNode()
    rclpy.spin(nose_node)
    nose_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

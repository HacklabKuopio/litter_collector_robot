# encoding: utf-8

# Copyright 2023-2026 Mikael Lammentausta
#
# Licensed under the Apache License, Version 2.0.
# See the LICENSE file in the project root for license information.

from os import system
from time import sleep

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.logging import LoggingSeverity

from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist
from example_interfaces.msg import String # for audio

try:
    import RPi.GPIO as GPIO
except:
    import Mock.GPIO as GPIO


class LightsNode(Node):

    # Operation mode; four modes.
    #   0 Indicator + main lights active
    #   1 Indicator + main + flood lights active
    #   2 Indicator + main + flood lights + buzzer active
    #   3 Indicator + main lights + buzzer active
    #   4 No lights
    operation_mode = 0

    # Indicator light states
    state_standby = True
    state_emergency_stop_alert = False
    state_movement_commanded = False
    state_movement_turn_left = False
    state_movement_turn_right = False
    state_hazard = False
    state_low_voltage_yellow = False
    state_low_voltage_red = False

    def __init__(self):
        super().__init__('lights_node')
        # Configure logger
        self.logger = self.get_logger()
        self.logger.set_level(LoggingSeverity.INFO)
        self.logger.info("LightsNode init")

        # declare parameters
        self.declare_parameter("indicator_light_yellow_pin", rclpy.Parameter.Type.INTEGER)
        self.indicator_light_yellow_pin = self.get_parameter("indicator_light_yellow_pin").value

        self.declare_parameter("indicator_light_green_pin", rclpy.Parameter.Type.INTEGER)
        self.indicator_light_green_pin = self.get_parameter("indicator_light_green_pin").value

        self.declare_parameter("indicator_light_red_pin", rclpy.Parameter.Type.INTEGER)
        self.indicator_light_red_pin = self.get_parameter("indicator_light_red_pin").value

        self.declare_parameter("indicator_light_white_pin", rclpy.Parameter.Type.INTEGER)
        self.indicator_light_white_pin = self.get_parameter("indicator_light_white_pin").value

        self.declare_parameter("indicator_buzzer_pin", rclpy.Parameter.Type.INTEGER)
        self.indicator_buzzer_pin = self.get_parameter("indicator_buzzer_pin").value

        self.declare_parameter("flood_light_pin", rclpy.Parameter.Type.INTEGER)
        self.flood_light_pin = self.get_parameter("flood_light_pin").value

        self.declare_parameter("main_light_left_pin", rclpy.Parameter.Type.INTEGER)
        self.main_light_left_pin = self.get_parameter("main_light_left_pin").value

        self.declare_parameter("main_light_right_pin", rclpy.Parameter.Type.INTEGER)
        self.main_light_right_pin = self.get_parameter("main_light_right_pin").value

        # Init Raspberry Pi GPIO pins
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.indicator_light_yellow_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.indicator_light_green_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.indicator_light_red_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.indicator_light_white_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.indicator_buzzer_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.flood_light_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.main_light_left_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.main_light_right_pin, GPIO.OUT, initial=GPIO.LOW)

        self.audio_pub = self.create_publisher(
            String,
            'lcr/audio/play',
            10
        )

        # angularity (left/right direction) from cmd_vel
        self.create_subscription(
            Twist,
            'lcr/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        # hazard light (triangle keypress)
        self.create_subscription(
            Bool,
            'lcr/lights/hazard',
            self.hazard_light_callback,
            10
        )
        # lights operation mode toggle
        self.create_subscription(
            Bool,
            'lcr/lights/mode',
            self.light_mode_callback,
            10
        )
        # voltage level subscription
        self.create_subscription(
            Bool,
            'lcr/health/low_voltage_yellow_alert',
            self.low_voltage_yellow_alert_callback,
            100
        )
        self.create_subscription(
            Bool,
            'lcr/health/low_voltage_red_alert',
            self.low_voltage_red_alert_callback,
            100
        )
        # emergency stop
        self.create_subscription(
            Bool,
            'lcr/emergency_stop_alert',
            self.emergency_stop_alert_callback,
            10
        )

        # self.movement_trigger = False

        self.duty_cycle_counter = 0.0

        self.create_timer(1.0, self.logical_loop)
        self.create_timer(0.05, self.gpio_loop)


    def __del__(self):
        GPIO.cleanup()


    def logical_loop(self):
        if self.state_emergency_stop_alert or self.state_movement_commanded:
            self.state_standby = False
        else:
            self.state_standby = True

        # if self.state_standby:
        #     self.logger.info("Standby")
        if self.state_hazard:
            self.logger.debug("State: Hazard", throttle_duration_sec=5)
        if self.state_emergency_stop_alert:
            self.logger.debug("State: Emergency stop", throttle_duration_sec=5)
        if self.state_low_voltage_yellow:
            self.logger.debug("State: Low voltage; yellow", throttle_duration_sec=5)
        if self.state_low_voltage_red:
            self.logger.debug("State: Low voltage; red", throttle_duration_sec=5)
        # if self.state_movement_commanded:
        #     self.logger.info("Moving")


    def gpio_loop(self):
        GPIO.output(self.indicator_light_red_pin, self.get_state_of_red_indicator())
        GPIO.output(self.indicator_light_green_pin, self.get_state_of_green_indicator())
        GPIO.output(self.indicator_light_yellow_pin, self.get_state_of_yellow_indicator())
        GPIO.output(self.indicator_light_white_pin, self.get_state_of_white_indicator())
        GPIO.output(self.flood_light_pin, self.get_state_of_flood_light())
        GPIO.output(self.main_light_left_pin, self.get_state_of_main_left_light())
        GPIO.output(self.main_light_right_pin, self.get_state_of_main_right_light())
        GPIO.output(self.indicator_buzzer_pin, self.get_state_of_buzzer())

        self.duty_cycle_counter += 3.25
        if self.duty_cycle_counter > 100.0:
            self.duty_cycle_counter = 0.0


    def get_state_of_red_indicator(self):
        if self.operation_mode == 4:
            return GPIO.LOW # lights off

        if self.state_low_voltage_red:
            if (self.duty_cycle_counter >= 50 and self.duty_cycle_counter < 60) or \
                (self.duty_cycle_counter >= 70 and self.duty_cycle_counter < 80) or \
                (self.duty_cycle_counter >= 90 and self.duty_cycle_counter < 100):
                return GPIO.HIGH

        elif self.state_emergency_stop_alert:
            return GPIO.HIGH

        return GPIO.LOW


    def get_state_of_green_indicator(self):
        if self.operation_mode == 4:
            return GPIO.LOW # lights off

        if not self.state_emergency_stop_alert:
            return GPIO.HIGH

        return GPIO.LOW


    def get_state_of_yellow_indicator(self):
        if self.operation_mode == 4:
            return GPIO.LOW # lights off

        if self.state_movement_commanded and not self.state_emergency_stop_alert:
            if self.duty_cycle_counter <= 5:
                return GPIO.HIGH

        if self.state_hazard:
            if (self.duty_cycle_counter >= 5 and self.duty_cycle_counter < 10) or \
                (self.duty_cycle_counter >= 15 and self.duty_cycle_counter < 20) or \
                (self.duty_cycle_counter >= 25 and self.duty_cycle_counter < 30):
                return GPIO.HIGH

        if self.state_low_voltage_yellow:
            if (self.duty_cycle_counter >= 50 and self.duty_cycle_counter < 60) or \
                (self.duty_cycle_counter >= 70 and self.duty_cycle_counter < 80) or \
                (self.duty_cycle_counter >= 90 and self.duty_cycle_counter < 100):
                return GPIO.HIGH

        return GPIO.LOW


    def get_state_of_white_indicator(self):
        if self.operation_mode == 4:
            return GPIO.LOW # lights off

        if self.state_movement_commanded and not self.state_emergency_stop_alert:
            if (self.duty_cycle_counter >= 50 and self.duty_cycle_counter < 60):
                return GPIO.HIGH

        if not self.state_emergency_stop_alert and not self.state_movement_commanded:
            return GPIO.HIGH

        if self.state_emergency_stop_alert:
            if (self.duty_cycle_counter >= 95 and self.duty_cycle_counter < 100):
                return GPIO.HIGH

        return GPIO.LOW


    def get_state_of_flood_light(self):
        if self.operation_mode == 1 or self.operation_mode == 2:
            return GPIO.HIGH

        return GPIO.LOW


    def get_state_of_main_left_light(self):
        if self.operation_mode == 4:
            return GPIO.LOW # lights off

        if self.state_movement_turn_left and not self.state_emergency_stop_alert:
            if (self.duty_cycle_counter >= 25 and self.duty_cycle_counter < 50) or \
                (self.duty_cycle_counter >= 75 and self.duty_cycle_counter < 100):
                return GPIO.LOW

        return GPIO.HIGH


    def get_state_of_main_right_light(self):
        if self.operation_mode == 4:
            return GPIO.LOW # lights off

        if self.state_movement_turn_right and not self.state_emergency_stop_alert:
            if (self.duty_cycle_counter >= 25 and self.duty_cycle_counter < 50) or \
                (self.duty_cycle_counter >= 75 and self.duty_cycle_counter < 100):
                return GPIO.LOW

        return GPIO.HIGH


    def get_state_of_buzzer(self):
        if not self.operation_mode == 2 and not self.operation_mode == 3:
            return GPIO.LOW

        if self.state_movement_turn_left or self.state_movement_turn_right:
            return GPIO.HIGH

        # short buzz when forward movement started
        # if self.movement_trigger:
        #     self.movement_trigger = False
        #     return GPIO.HIGH

        return GPIO.LOW


    def cmd_vel_callback(self, twist_msg):
        linear_x = twist_msg.linear.x
        angular_z = -twist_msg.angular.z

        # self.logger.debug(f"cmd_vel {linear_x} {angular_z}", throttle_duration_sec=1.0)

        # implement a "dead zone" so miniscule movement does not trigger the lights
        if linear_x < -0.01 or linear_x > 0.01:
            self.state_movement_commanded = True
            # self.movement_trigger = True
        else:
            self.state_movement_commanded = False
            # self.movement_trigger = False

        if angular_z < -0.1:
            self.state_movement_commanded = True
            # set turning indicator lights; during backwards movement they are reversed
            if linear_x > 0:
                self.state_movement_turn_left = True
                self.state_movement_turn_right = False
            else:
                self.state_movement_turn_left = False
                self.state_movement_turn_right = True
        elif angular_z > 0.1:
            self.state_movement_commanded = True
            # set turning indicator lights; during backwards movement they are reversed
            if linear_x > 0:
                self.state_movement_turn_left = False
                self.state_movement_turn_right = True
            else:
                self.state_movement_turn_left = True
                self.state_movement_turn_right = False
        else:
            self.state_movement_turn_left = False
            self.state_movement_turn_right = False


    def hazard_light_callback(self, msg):
        self.state_hazard = msg.data
        if self.state_hazard:
            self.audio_pub.publish(String(data="hazard_toggle"))


    def low_voltage_yellow_alert_callback(self, msg):
        self.low_voltage_yellow = msg.data


    def low_voltage_red_alert_callback(self, msg):
        self.low_voltage_red = msg.data


    def emergency_stop_alert_callback(self, msg):
        self.state_emergency_stop_alert = msg.data


    def light_mode_callback(self, msg):
        if msg.data:
            self.operation_mode += 1
            if self.operation_mode > 4:
                self.operation_mode = 0
        self.logger.info(f"Change operation mode to {self.operation_mode}")
        self.audio_pub.publish(String(data="light_mode_change"))


def main(args=None):
    rclpy.init(args=args)
    lights_node = LightsNode()

    executor = MultiThreadedExecutor()
    rclpy.spin(lights_node, executor=executor)
    # rclpy.spin(lights_node)

    lights_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

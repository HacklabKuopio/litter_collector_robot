# encoding: utf-8

# Copyright 2023-2026 Mikael Lammentausta
#
# Licensed under the Apache License, Version 2.0.
# See the LICENSE file in the project root for license information.

from time import sleep
from math import floor, fabs

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.logging import LoggingSeverity

from std_msgs.msg import Bool, Float32MultiArray, Int16
from geometry_msgs.msg import Twist, TwistStamped
from ds4_driver_msgs.msg import Status
from example_interfaces.msg import String # for audio


class RobotControlNode(Node):
    """
    This Node:
        - subscribes to ds4/status message
            - translates linear and angular velocity to lcr/cmd_vel twist message
                - publishes lcr/cmd_vel
            - ds4 buttons:
                - publishes lcr/vacuum/toggle
                            lcr/vacuum/timeout_change
                - publishes lcr/emergency_stop_ds4_button
                - publishes lcr/emergency_stop_reset
                - publishes lcr/shutdown

    """
    def __init__(self):
        super().__init__('robot_control_node')
        # Configure logger
        self.logger = self.get_logger()
        self.logger.set_level(LoggingSeverity.DEBUG)
        self.logger.info("RobotControlNode init")

        # declare parameters
        self.declare_parameter("linear_velocity_scale", rclpy.Parameter.Type.DOUBLE)
        self.linear_velocity_scale = self.get_parameter("linear_velocity_scale").value
        self.declare_parameter("angular_velocity_scale", rclpy.Parameter.Type.DOUBLE)
        self.angular_velocity_scale = self.get_parameter("angular_velocity_scale").value

        # create ds4_driver /ds/status subscription
        self.create_subscription(
            Status,
            'ds4/status',
            self.ds4_status_callback,
            1
        )

        # publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'lcr/cmd_vel',
            1
        )
        self.movement_command_pub = self.create_publisher(
            Bool,
            'lcr/robot_movement_commanded',
            10
        )
        self.hazard_light_pub = self.create_publisher(
            Bool,
            'lcr/lights/hazard',
            10
        )
        self.light_mode_toggle_pub = self.create_publisher(
            Bool,
            'lcr/lights/mode',
            100
        )
        self.nose_command_pub = self.create_publisher(
            Float32MultiArray,
            'lcr/nose/cmd_vels',
            100
        )
        self.vacuum_toggle_pub = self.create_publisher(
            Bool,
            'lcr/vacuum/toggle',
            10
        )
        self.vacuum_timeout_change_pub = self.create_publisher(
            Int16,
            'lcr/vacuum/timeout_change',
            10
        )
        self.emergency_stop_reset_pub = self.create_publisher(
            Bool,
            'lcr/emergency_stop_reset',
            10
        )
        self.emergency_stop_ds4_button_pub = self.create_publisher(
            Bool,
            'lcr/emergency_stop_ds4_button',
            10
        )
        self.shutdown_pub = self.create_publisher(
            Bool,
            'lcr/shutdown',
            10
        )
        self.audio_pub = self.create_publisher(
            String,
            'lcr/audio/play',
            100
        )

        self.velocity_modifier = 2.0 # speed factor, controlled with dpad up/down buttons
        self.velocity_down_throttle_mod = 0
        self.velocity_up_throttle_mod = 0

        # Nose variables
        self.nose_cmd_throttle_mod = 0 # modulo for throttling nose commands

        # vacuum variables
        self.vacuum_toggle_mod = 0
        self.vacuum_min_throttle_mod = 0 # modulo for throttling vacuum duration -- button
        self.vacuum_max_throttle_mod = 0 # modulo for throttling vacuum duration ++ button

        # Hazard light toggle & lights operation mode
        self.hazard_light_toggle = False
        self.hazard_light_throttle_mod = 0
        self.light_mode_throttle_mod = 0

        # Movement blip throttle
        self.movement_blip_throttle_mod = 0

        # Horn sound throttle
        self.horn_throttle_mod = 0

        # play startup sound
        sleep(2.5)
        self.audio_pub.publish(String(data="unit_online"))


    def __del__(self):
        pass


    def ds4_status_callback(self, msg):
        """Subscribes to ds4 controller /ds4/status message."""

        # Check if controller was disconnected
        if msg.disconnected:
            self.logger.warn("Controller disconnected, trigger emergency stop")
            self.audio_pub.publish(String(data="control_device_disconnected"))
            self.emergency_stop_ds4_button()
            return

        # Triangle button with PS button, when movement is not issued
        if msg.button_triangle and not msg.button_ps:
            # slow button reactivity
            if self.hazard_light_throttle_mod % 50 == 0:
                self.hazard_light_toggle = not self.hazard_light_toggle
                self.hazard_light_pub.publish(Bool(data=self.hazard_light_toggle))
            self.hazard_light_throttle_mod += 1

        elif msg.button_triangle and msg.button_ps and msg.axis_left_x == 0 and msg.axis_left_y == 0:
            self.emergency_stop_reset()

        # Circle button to trigger emergency stop from software
        elif msg.button_circle:
            self.emergency_stop_ds4_button()

        # Shutdown signal - ShutdownNode checks that the buttons are pressed for some time before issuing shutdown
        if msg.button_options and msg.button_share:
            # self._shutdown_call_loop()
            self.shutdown_call()
            return

        # Options - toggle lights mode (throttled)
        elif msg.button_options:
            if self.light_mode_throttle_mod % 50 == 0:
                self.light_mode_toggle_pub.publish(Bool(data=True))
            self.light_mode_throttle_mod += 1

        # Dpad up/down - control max velocity
        d_pad_up = msg.button_dpad_up
        d_pad_down = msg.button_dpad_down

        if d_pad_up:
            if self.velocity_up_throttle_mod % 50 == 0:
                # increase max velocity
                vm = min(2.0, self.velocity_modifier + 0.4)
                if vm != self.velocity_modifier:
                    self.velocity_modifier = vm
                    self.logger.debug(f"increase max velocity modifier to {self.velocity_modifier:.1f}")
                    # play sound unit_speed_x
                    self.audio_pub.publish(String(data=f"unit_speed_{int(self.velocity_modifier)}"))
            self.velocity_up_throttle_mod += 1
        if d_pad_down:
            if self.velocity_down_throttle_mod % 50 == 0:
                # decrease max velocity
                vm = max(0.6, self.velocity_modifier - 0.4)
                if vm != self.velocity_modifier:
                    self.velocity_modifier = vm
                    self.logger.debug(f"decrease max velocity modifier to {self.velocity_modifier:.1f}")
                    # play sound unit_speed_x
                    self.audio_pub.publish(String(data=f"unit_speed_{int(self.velocity_modifier)}"))
            self.velocity_down_throttle_mod += 1

        # Adjust vacuum cleaner timer with D-pad left and right buttons (throttled)
        d_pad_left = msg.button_dpad_left
        d_pad_right = msg.button_dpad_right

        if d_pad_left:
            if self.vacuum_min_throttle_mod % 50 == 0:
                self.vacuum_timeout_change_pub.publish(Int16(data=-1))
            self.vacuum_min_throttle_mod += 1
        if d_pad_right:
            if self.vacuum_max_throttle_mod % 50 == 0:
                self.vacuum_timeout_change_pub.publish(Int16(data=1))
            self.vacuum_max_throttle_mod += 1

        # Cross button: start the vacuum cleaner (throttled)
        if msg.button_cross:
            if self.vacuum_toggle_mod % 50 == 0:
                self.vacuum_toggle_pub.publish(Bool(data=True))
            self.vacuum_toggle_mod += 1

        # Publish Twist message for Roboclaw
        self.publish_twist_message(msg)

        # Publish status to control nose
        if self.nose_cmd_throttle_mod % 15 == 0:
            self.publish_nose_message(msg)
        self.nose_cmd_throttle_mod += 1

        # play doot-doot on ps button press
        if self.horn_throttle_mod % 60 == 0:
            if msg.button_ps and not msg.button_triangle:
                self.audio_pub.publish(String(data="horn"))
        self.horn_throttle_mod += 1

        # play normal operation sound
        # self.audio_pub.publish(String(data="normal_state"))


    def publish_twist_message(self, status_msg):
        # Safety feature; require either button l1 or l2 to be pressed for movement command
        if status_msg.button_l1 or status_msg.button_l2:
            # Calculate linear and angular velocity based on Left-stick inputs
            linear_vel = self.linear_velocity_scale * status_msg.axis_left_y * ((status_msg.button_l2 * self.velocity_modifier) + 1)
            angular_vel = self.angular_velocity_scale * status_msg.axis_left_x * ((status_msg.button_l2 * self.velocity_modifier) + 1)

            # Publish Twist message on cmd_vel topic
            twist_msg = Twist()
            twist_msg.linear.x = linear_vel
            twist_msg.angular.z = angular_vel
            self.cmd_vel_pub.publish(twist_msg)

            # Publish whether movement command was issued (linear or angular velocity was > 0) (for brake control main callback)
            movement_commanded = Bool(data=(linear_vel != 0 or angular_vel != 0))
            self.movement_command_pub.publish(movement_commanded)

            # Blip sound
            self.publish_movement_blip(linear_vel)

        else:
            # ensure that the previous cmd_vel does not remain published; stop motors
            twist_msg = Twist()
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            self.cmd_vel_pub.publish(twist_msg)
            # inform subscribers movement was not commanded
            self.movement_command_pub.publish(Bool(data=False))


    def publish_movement_blip(self, linear_vel):
        """Blip sound based on linear velocity.

        Radar blip has 5 stages. Velocity 0 -> radar1, and velocity "max" (0.25) -> radar5.
        """
        if self.movement_blip_throttle_mod % 100 == 0:
            radar_level = floor(fabs(linear_vel) * 8.0 + 1 + 0.1)
            radar_sound = None
            if radar_level >= 5:
                radar_sound = 'unit_movement_5'
            elif radar_level >= 4:
                radar_sound = 'unit_movement_4'
            elif radar_level >= 3:
                radar_sound = 'unit_movement_3'
            elif radar_level >= 2:
                radar_sound = 'unit_movement_2'
            elif radar_level >= 1:
                radar_sound = 'unit_movement_1'
            else:
                self.logger.warning(f"Sound blip; unknown velocity {linear_vel}")
                return

            self.logger.debug(f"Sound blip; Radar level {radar_level}: {radar_sound} (velocity {linear_vel}")
            self.audio_pub.publish(String(data=radar_sound))

        self.movement_blip_throttle_mod += 1


    def publish_nose_message(self, status_msg):
        nose_y_vel = status_msg.axis_right_x
        nose_z_vel = status_msg.axis_right_y
        self.nose_command_pub.publish(Float32MultiArray(data=[nose_y_vel, nose_z_vel]))


    def emergency_stop_reset(self):
        self.emergency_stop_reset_pub.publish(Bool(data=True))


    def emergency_stop_ds4_button(self):
        self.emergency_stop_ds4_button_pub.publish(Bool(data=True))


    def shutdown_call(self):
        self.shutdown_pub.publish(Bool(data=True))


def main(args=None):
    rclpy.init(args=args)
    robot_control_node = RobotControlNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(robot_control_node, executor=executor)
    robot_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

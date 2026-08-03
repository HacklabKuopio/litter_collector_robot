# encoding: utf-8

# Copyright 2023-2026 Mikael Lammentausta
#
# Licensed under the Apache License, Version 2.0.
# See the LICENSE file in the project root for license information.

import os

import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity

from std_msgs.msg import Bool, Float32, Float32MultiArray
from geometry_msgs.msg import Twist, TwistStamped

from roboclaw_3 import Roboclaw
from roboclaw_error_logger import RoboclawErrorLogger


class RoboclawTwistNode(Node):
    roboclaw = None
    emergency_stop_triggered = False
    error_logger = None

    def __init__(self):
        super().__init__('roboclaw_twist_node')
        # Configure logger
        self.logger = self.get_logger()
        self.logger.set_level(LoggingSeverity.DEBUG)
        self.logger.info("RoboclawTwistNode init")

        # error hex decoder
        self.error_logger = RoboclawErrorLogger(self.logger)

        # Define config params
        self.declare_parameter("accel_quad_pulses_per_second", rclpy.Parameter.Type.INTEGER)
        self.accel_quad_pulses_per_second = int(self.get_parameter("accel_quad_pulses_per_second").value)
        self.declare_parameter("device_name", rclpy.Parameter.Type.STRING)
        self.device_name = self.get_parameter("device_name").value
        self.declare_parameter("device_baudrate", rclpy.Parameter.Type.INTEGER)
        self.device_baudrate = int(self.get_parameter("device_baudrate").value)
        self.declare_parameter("device_address", rclpy.Parameter.Type.INTEGER)
        self.device_address = self.get_parameter("device_address").value
        self.declare_parameter("m1_p", rclpy.Parameter.Type.DOUBLE)
        self.m1_p = float(self.get_parameter("m1_p").value)
        self.declare_parameter("m1_i", rclpy.Parameter.Type.DOUBLE)
        self.m1_i = float(self.get_parameter("m1_i").value)
        self.declare_parameter("m1_d", rclpy.Parameter.Type.DOUBLE)
        self.m1_d = float(self.get_parameter("m1_d").value)
        self.declare_parameter("m1_qpps", rclpy.Parameter.Type.INTEGER)
        self.m1_qpps = int(self.get_parameter("m1_qpps").value)
        self.declare_parameter("m2_p", rclpy.Parameter.Type.DOUBLE)
        self.m2_p = float(self.get_parameter("m2_p").value)
        self.declare_parameter("m2_i", rclpy.Parameter.Type.DOUBLE)
        self.m2_i = float(self.get_parameter("m2_i").value)
        self.declare_parameter("m2_d", rclpy.Parameter.Type.DOUBLE)
        self.m2_d = float(self.get_parameter("m2_d").value)
        self.declare_parameter("m2_qpps", rclpy.Parameter.Type.INTEGER)
        self.m2_qpps = int(self.get_parameter("m2_qpps").value)
        self.declare_parameter("m1_max_current", rclpy.Parameter.Type.DOUBLE)
        self.m1_max_current = float(self.get_parameter("m1_max_current").value)
        self.declare_parameter("m2_max_current", rclpy.Parameter.Type.DOUBLE)
        self.m2_max_current = float(self.get_parameter("m2_max_current").value)
        self.declare_parameter("max_angular_velocity", rclpy.Parameter.Type.DOUBLE)
        self.max_angular_velocity = float(self.get_parameter("max_angular_velocity").value)
        self.declare_parameter("max_linear_velocity", rclpy.Parameter.Type.DOUBLE)
        self.max_linear_velocity = float(self.get_parameter("max_linear_velocity").value)
        self.declare_parameter("max_seconds_uncommanded_travel", rclpy.Parameter.Type.DOUBLE)
        self.max_seconds_uncommanded_travel = float(self.get_parameter("max_seconds_uncommanded_travel").value)
        self.declare_parameter("quad_pulses_per_meter", rclpy.Parameter.Type.INTEGER)
        self.quad_pulses_per_meter = int(self.get_parameter("quad_pulses_per_meter").value)
        self.declare_parameter("quad_pulses_per_revolution", rclpy.Parameter.Type.INTEGER)
        self.quad_pulses_per_revolution = int(self.get_parameter("quad_pulses_per_revolution").value)
        self.declare_parameter("wheel_radius", rclpy.Parameter.Type.DOUBLE)
        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.declare_parameter("wheel_separation", rclpy.Parameter.Type.DOUBLE)
        self.wheel_separation = float(self.get_parameter("wheel_separation").value)
        self.declare_parameter("roboclaw_status_topic", "roboclaw_status")
        self.roboclaw_status_topic = self.get_parameter("roboclaw_status_topic").value
        self.declare_parameter("vmin", rclpy.Parameter.Type.INTEGER)
        self.vmin = int(self.get_parameter("vmin").value)
        self.declare_parameter("vtime", rclpy.Parameter.Type.INTEGER)
        self.vtime = int(self.get_parameter("vtime").value)

        # Init emergency stop subsciber
        self.emergency_stop_subscription = self.create_subscription(
            Bool,
            'lcr/emergency_stop',
            self.emergency_stop_callback,
            1
        )

        # Init subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'lcr/cmd_vel',
            self.cmd_vel_callback,
            1
        )

        # Publish main voltage (to health node)
        self.main_voltage_pub = self.create_publisher(
            Float32,
            'lcr/roboclaw/main_voltage',
            100
        )
        # Publish current travelled distance in meters (to health node)
        self.odom_travelled_distance_pub = self.create_publisher(
            Float32,
            'lcr/roboclaw/odom_travelled_distance',
            100
        )
        # Publish motor temperatures
        self.temperatures_pub = self.create_publisher(
            Float32MultiArray,
            'lcr/roboclaw/temperatures',
            100
        )
        # Publish PWM data
        self.pwms_pub = self.create_publisher(
            Float32MultiArray,
            'lcr/roboclaw/pwms',
            100
        )
        # Publish currents
        self.currents_pub = self.create_publisher(
            Float32MultiArray,
            'lcr/roboclaw/currents',
            100
        )

        # Open RoboClaw device bus
        if os.path.exists(self.device_name):
            self.logger.info(f'Open RoboClaw on {self.device_name} @ {self.device_baudrate} baud')
            self.roboclaw = Roboclaw(self.device_name, self.device_baudrate)
            self.roboclaw.Open()

            # Timer to check and publish main battery voltage
            self.create_timer(2.5, self.read_main_battery_voltage)
            # Timer to read encoder data and publish travelled distance
            self.create_timer(5.0, self.read_encoders)
            # Timer to read and publish temperature data
            self.create_timer(10.0, self.read_temperatures)
            # PWMs
            self.create_timer(10.0, self.read_pwms)
            # Currents
            self.create_timer(10.0, self.read_currents)
            # Timer to check possible errors
            self.create_timer(10.0, self.read_error)
        else:
            self.logger.error(f'RoboClaw device {self.device_name} does not exist!')


    def __del__(self):
        self.stop_motors()


    def cmd_vel_callback(self, twist_msg):
        """RoboClaw motor control, callback for cmd_vel Twist message.

        Based on https://github.com/sonyccd/roboclaw_ros/blob/master/roboclaw_node/nodes/roboclaw_node.py#L238
        """
        linear_x = twist_msg.linear.x
        angular_z = -twist_msg.angular.z

        # Check for max_linear_velocity
        if linear_x > self.max_linear_velocity:
            self.logger.warning(f'max lin +x exceeded {linear_x:.3f}', throttle_duration_sec=0.5)
            linear_x = self.max_linear_velocity
        elif linear_x < -self.max_linear_velocity:
            self.logger.warning(f'max lin -x exceeded {linear_x:.3f}', throttle_duration_sec=0.5)
            linear_x =  -self.max_linear_velocity

        # Check for max_angular_velocity
        if angular_z > self.max_angular_velocity:
            self.logger.warning(f'max ang +z exceeded {angular_z:.3f}', throttle_duration_sec=0.5)
            angular_z = self.max_angular_velocity
        elif angular_z < -self.max_angular_velocity:
            self.logger.warning(f'max ang -z exceeded {angular_z:.3f}', throttle_duration_sec=0.5)
            angular_z =  -self.max_angular_velocity

        # Calculate vr/vl
        vr = linear_x + angular_z * self.wheel_separation / 2.0 # m/s
        vl = linear_x - angular_z * self.wheel_separation / 2.0

        if linear_x != 0 or angular_z != 0:
            self.logger.info(f'Velocity linear x {linear_x:.3f}, angular z {angular_z:.3f}', throttle_duration_sec=0.5)
            self.logger.info(f'Velocity R: {(vr * 3.6):.2f} km/h, L: {(vl * 3.6):.2f} km/h', throttle_duration_sec=0.5)

        vr_ticks = int(vr * self.quad_pulses_per_meter) # ticks/s
        vl_ticks = int(vl * self.quad_pulses_per_meter)

        try:
            # This is a hack way to keep a poorly tuned PID from making noise at speed 0
            if vr_ticks == 0 and vl_ticks == 0:
                self.stop_motors()
            elif self.roboclaw:
                # self.logger.debug(f'SpeedM1M2 vr_ticks: {vr_ticks}, vl_ticks: {vl_ticks}', throttle_duration_sec=0.25)
                self.roboclaw.SpeedM1M2(self.device_address, vr_ticks, vl_ticks)

            # self.last_set_speed_time = self.get_clock().now()
            # self.logger.debug(f'last_set_speed_time {self.last_set_speed_time}', throttle_duration_sec=0.5)

        except OSError as e:
            self.logger.warning(f"SpeedM1M2 OSError: {e.errno}")
            self.logger.debug(e)


    def stop_motors(self):
        if self.roboclaw:
            self.logger.debug('Stop roboclaw motors', throttle_duration_sec=5)
            self.roboclaw.ForwardM1(self.device_address, 0)
            self.roboclaw.ForwardM2(self.device_address, 0)


    def emergency_stop_callback(self, msg):
        """Check emergency stop status, should be 1 for correct operation"""
        if msg.data and not self.emergency_stop_triggered:
            self.logger.info(f'Emergency stop triggered', throttle_duration_sec=1)
            self.stop_motors()
        self.emergency_stop_triggered = msg.data


    def read_error(self):
        """Read and log possible error status"""
        if not self.roboclaw:
            return
        try:
            error = self.roboclaw.ReadError(self.device_address)
            # self.logger.debug(f'Error: {error}')
            if error[1]:
                self.error_logger.decode_error(error[1])
        except Exception as err:
            self.logger.warning(str(err))


    def read_main_battery_voltage(self):
        """Read and publish main battery voltage"""
        if not self.roboclaw:
            return
        try:
            main_battery_reading = self.roboclaw.ReadMainBatteryVoltage(self.device_address)
            powered_on = main_battery_reading[0]
            if powered_on == 0:
                self.logger.info("RoboClaw is off")
            else:
                main_battery_voltage = (main_battery_reading[1] / 10.0) - 0.4
                # self.logger.debug(f"Main battery voltage: {main_battery_voltage} V")
                self.main_voltage_pub.publish(Float32(data=main_battery_voltage))
        except Exception as err:
            self.logger.warning(str(err))


    def read_encoders(self):
        """Read encoder data and publish travelled distance (in meters)"""
        if not self.roboclaw:
            return
        try:
            enc1 = self.roboclaw.ReadEncM1(self.device_address)
            enc2 = self.roboclaw.ReadEncM2(self.device_address)
            # modes = self.roboclaw.ReadEncoderModes(self.device_address)
            # self.logger.debug(f'Encoders M1: {enc1}, M2: {enc2}, modes: {modes}')
            enc1_m = enc1[1] / self.quad_pulses_per_meter
            enc2_m = enc2[1] / self.quad_pulses_per_meter
            linear_distance_m = (enc1_m + enc2_m) / 2.0
            self.logger.debug(f'Encoders odom M1: {enc1_m:.2f} m, M2: {enc2_m:.2f} m (avg {linear_distance_m:.2f} m)')
            self.odom_travelled_distance_pub.publish(Float32(data=linear_distance_m))
        except Exception as err:
            self.logger.warning(str(err))


    def read_temperatures(self):
        """Read and publish motor temperatures (in Celcius)"""
        if not self.roboclaw:
            return
        try:
            temp1 = self.roboclaw.ReadTemp(self.device_address)
            temp2 = self.roboclaw.ReadTemp2(self.device_address)
            _temp1 = temp1[1] / 10.0
            _temp2 = temp2[1] / 10.0
            if _temp1 > 40.0 or _temp2 > 40.0:
                self.logger.info(f'Temp M1: {_temp1:.1f}°C, M2: {_temp2:.1f}°C')
            else:
                self.logger.debug(f'Temp M1: {_temp1:.1f}°C, M2: {_temp2:.1f}°C')
            self.temperatures_pub.publish(Float32MultiArray(data=[_temp1, _temp2]))
        except Exception as err:
            self.logger.warning(str(err))


    def read_pwms(self):
        """Read and publish PWM data"""
        if not self.roboclaw:
            return
        try:
            pwms = self.roboclaw.ReadPWMs(self.device_address)
            if pwms and pwms[0] == 1:
                pwm1 = pwms[1] / 327.67
                pwm2 = pwms[2] / 327.67
                self.logger.debug(f'PWM 1 {pwm1}, PWM 2 {pwm2}')
                self.pwms_pub.publish(Float32MultiArray(data=[pwm1, pwm2]))
        except Exception as err:
            self.logger.warning(str(err))


    def read_currents(self):
        """Read and publish currents"""
        if not self.roboclaw:
            return
        try:
            currents = self.roboclaw.ReadCurrents(self.device_address)
            if currents and currents[0] == 1:
                current1 = currents[1] / 100.0
                current2 = currents[2] / 100.0
                self.logger.debug(f'Current 1 {current1}, current 2 {current2}')
                self.currents_pub.publish(Float32MultiArray(data=[current1, current2]))
        except Exception as err:
            self.logger.warning(str(err))


    def read_sensors_debug(self):
        """Read RoboClaw sensor data"""
        self.logger.debug('--- RoboClaw sensor data ---')

        # speed
        try:
            speed1 = self.roboclaw.ReadSpeedM1(self.device_address)
            speed2 = self.roboclaw.ReadSpeedM2(self.device_address)
            # self.logger.debug(f'Speed M1: {speed1}, M2: {speed2}')
            _speed1 = speed1[1]
            _speed2 = speed2[1]
            self.logger.debug(f'Speed M1: {_speed1}, M2: {_speed2}')
            # ispeed1 = self.roboclaw.ReadISpeedM1(self.device_address)
            # ispeed2 = self.roboclaw.ReadISpeedM2(self.device_address)
            # self.logger.debug(f'ISpeed M1: {speed1}, M2: {speed2}')
        except Exception as err:
            self.logger.warning(str(err))

        # # velocity PID
        # try:
        #     vel1 = self.roboclaw.ReadM1VelocityPID(self.device_address)
        #     vel2 = self.roboclaw.ReadM2VelocityPID(self.device_address)
        #     self.logger.debug(f'Velocity PID M1: {vel1}, M2: {vel2}')
        # except Exception as err:
        #     self.logger.warning(str(err))

        # # position PID
        # try:
        #     pos1 = self.roboclaw.ReadM1PositionPID(self.device_address)
        #     pos2 = self.roboclaw.ReadM2PositionPID(self.device_address)
        #     self.logger.debug(f'Position PID M1: {pos1}, M2: {pos2}')
        # except Exception as err:
        #     self.logger.warning(str(err))

        # # max current
        # try:
        #     maxcurr1 = self.roboclaw.ReadM1MaxCurrent(self.device_address)
        #     maxcurr2 = self.roboclaw.ReadM2MaxCurrent(self.device_address)
        #     self.logger.debug(f'Max current M1: {maxcurr1}, M2: {maxcurr2}')
        # except Exception as err:
        #     self.logger.warning(str(err))

        # # pwm mode
        # try:
        #     pwm_mode = self.roboclaw.ReadPWMMode(self.device_address)
        #     self.logger.debug(f'PWM mode: {pwm_mode}')
        # except Exception as err:
        #     self.logger.warning(str(err))

        # # eeprom
        # try:
        #     eeprom = self.roboclaw.ReadEeprom(self.device_address)
        #     self.logger.debug(f'Eeprom: {eeprom}')
        # except Exception as err:
        #     self.logger.warning(str(err))

        # # config
        # try:
        #     config = self.roboclaw.GetConfig(self.device_address)
        #     self.logger.debug(f'Config: {config}')
        # except Exception as err:
        #     self.logger.warning(str(err))

        # # dead band
        # try:
        #     db = self.roboclaw.GetDeadBand(self.device_address)
        #     self.logger.debug(f'Dead band: {db}')
        # except Exception as err:
        #     self.logger.warning(str(err))


def main(args=None):
    rclpy.init(args=args)
    roboclaw_twist_node = RoboclawTwistNode()
    # executor = MultiThreadedExecutor()
    rclpy.spin(roboclaw_twist_node) #, executor=executor)
    roboclaw_twist_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

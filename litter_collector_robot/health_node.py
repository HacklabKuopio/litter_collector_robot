# encoding: utf-8

# Copyright 2023-2026 Mikael Lammentausta
#
# Licensed under the Apache License, Version 2.0.
# See the LICENSE file in the project root for license information.

import os
from pathlib import Path
from datetime import datetime, timedelta
from math import fabs

import rclpy
# from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.logging import LoggingSeverity

from std_msgs.msg import Bool, Float32, Float32MultiArray
from example_interfaces.msg import String # for audio

import numpy as np


class HealthNode(Node):

    main_battery_voltages = []
    main_battery_voltages_window_size = 30 # readings to keep in memory
    main_battery_average_voltage = 0.0

    red_alert_cooloff = 15 # sec
    red_alert_cooloff_counter = 0
    yellow_alert_cooloff = 30 # sec
    yellow_alert_cooloff_counter = 0

    odom_logfilepath = Path("~/LCR_odom").expanduser()
    odom_log_interval = 60.0 # seconds
    odom_last_logwrite = None
    odom_last_raw_travelled_distance = 0.0 # meters
    odom_total_travelled_distance = 0.0 # meters
    odom_startup_offset = 0.0 # meters

    stats_csv_logfiletemplate = "~/LCR_health_%Y-%m-%d.csv"
    stats_csv_log_interval = 60.0 # seconds
    stats_vacuum_total_runtime = 0.0 # seconds
    stats_vacuum_total_onoff_cycles = 0
    stats_roboclaw_temp1 = 0.0 # °C
    stats_roboclaw_temp2 = 0.0 # °C

    def __init__(self):
        super().__init__('health_node')
        # Configure logger
        self.logger = self.get_logger()
        self.logger.set_level(LoggingSeverity.INFO)
        self.logger.info("HealthNode init")

        # Read config
        self.declare_parameter("charging_voltage", rclpy.Parameter.Type.DOUBLE)
        self.charging_voltage = self.get_parameter("charging_voltage").value

        self.declare_parameter("yellow_alert_voltage", rclpy.Parameter.Type.DOUBLE)
        self.yellow_alert_voltage = self.get_parameter("yellow_alert_voltage").value

        self.declare_parameter("red_alert_voltage", rclpy.Parameter.Type.DOUBLE)
        self.red_alert_voltage = self.get_parameter("red_alert_voltage").value

        self.declare_parameter("soc_steps", rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.soc_steps = np.array(self.get_parameter("soc_steps").value)

        # voltage level subscription
        self.create_subscription(
            Float32,
            'lcr/roboclaw/main_voltage',
            self.main_voltage_callback,
            100
        )
        # travelled distance (from roboclaw encoders) subscription
        # NOTE: this value is raw data from the encoders, and resets when roboclaw is powered off
        self.create_subscription(
            Float32,
            'lcr/roboclaw/odom_travelled_distance',
            self.odom_travelled_distance_callback,
            100
        )
        # roboclaw motor temperatures
        self.create_subscription(
            Float32MultiArray,
            'lcr/roboclaw/temperatures',
            self.motor_temperatures_callback,
            100
        )
        # roboclaw PWMs
        self.create_subscription(
            Float32MultiArray,
            'lcr/roboclaw/pwms',
            self.motor_pwms_callback,
            100
        )
        # roboclaw currents
        self.create_subscription(
            Float32MultiArray,
            'lcr/roboclaw/currents',
            self.motor_currents_callback,
            100
        )
        # vacuum stats
        self.create_subscription(
            Float32MultiArray,
            'lcr/vacuum/session_stats',
            self.vacuum_session_stats_callback,
            100
        )

        # low voltage alert publishers
        self.low_voltage_yellow_alert_pub = self.create_publisher(
            Bool,
            'lcr/health/low_voltage_yellow_alert',
            100
        )
        self.low_voltage_red_alert_pub = self.create_publisher(
            Bool,
            'lcr/health/low_voltage_red_alert',
            100
        )
        # audio alert publisher
        self.audio_pub = self.create_publisher(
            String,
            'lcr/audio/play',
            10
        )

        # read odometry log file
        if os.path.exists(self.odom_logfilepath):
            with open(self.odom_logfilepath, 'r') as file:
                try:
                    # store this offset to memory for csv stats
                    self.odom_startup_offset = float(file.read())
                except ValueError as err:
                    self.logger.warning(str(err))
                self.logger.info(f"Travelled distance offset from log file: {self.odom_startup_offset:.2f} m")

        # periodically write stats csv log
        self.create_timer(self.stats_csv_log_interval, self.stats_csv_write_log)


    def __del__(self):
        # store travelled distance to odometry logfile at shutdown
        #self.odom_write_log()
        pass


    def main_voltage_callback(self, msg):
        main_battery_voltage = msg.data
        # self.logger.debug(f"Main battery voltage: {main_battery_voltage:.2f} V")

        # push reading to array
        self.main_battery_voltages.append(main_battery_voltage)
        # keep array size manageable
        if len(self.main_battery_voltages) > self.main_battery_voltages_window_size:
            self.main_battery_voltages.pop(0) # remove oldest element

        # calculate average
        self.main_battery_average_voltage = np.sum(self.main_battery_voltages) / len(self.main_battery_voltages)
        # get SoC (State of Charge) approx level
        est_soc = self.get_soc_level(self.main_battery_average_voltage)
        # log voltage info
        self.logger.info(f"Current voltage: {self.get_termcolor_for_voltage(main_battery_voltage)}{main_battery_voltage:.2f} V\033[0m; " +
            f"avg {self.get_termcolor_for_voltage(self.main_battery_average_voltage)}{self.main_battery_average_voltage:.2f} V\033[0m " +
            # f"(n={len(self.main_battery_voltages)}); " +
            f"SoC ~{self.get_termcolor_for_soc(est_soc)}{est_soc}%")

        # react to voltages
        if main_battery_voltage >= self.charging_voltage:
            # charging
            self.logger.info(f"{self.get_termcolor_for_voltage(main_battery_voltage)}Battery charging", throttle_duration_sec=10.0)

        # RED ALERT VOLTAGE
        if self.main_battery_average_voltage <= self.red_alert_voltage:
            self.logger.warn(f"\033[91mBattery voltage critically low! {self.main_battery_average_voltage:.2f} V")

            # play sound every 15 sec
            if self.red_alert_cooloff_counter % self.red_alert_cooloff == 0:
                self.audio_pub.publish(String(data="red_low_power"))
            self.red_alert_cooloff_counter += 1

            # publish warning
            self.low_voltage_red_alert_pub.publish(Bool(data=True))

        # YELLOW ALERT VOLTAGE
        elif self.main_battery_average_voltage <= self.yellow_alert_voltage:
            self.logger.warn(f"\033[93mBattery voltage getting low! {self.main_battery_average_voltage:.2f} V")

            # play sound every 30 sec
            if self.yellow_alert_cooloff_counter % self.yellow_alert_cooloff == 0:
                self.audio_pub.publish(String(data="yellow_low_power"))
            self.yellow_alert_cooloff_counter += 1

            # publish warning
            self.low_voltage_yellow_alert_pub.publish(Bool(data=True))

        else:
            self.red_alert_cooloff_counter = 0
            self.yellow_alert_cooloff_counter = 0
            self.low_voltage_red_alert_pub.publish(Bool(data=False))
            self.low_voltage_yellow_alert_pub.publish(Bool(data=False))


    def get_soc_level(self, voltage):
        """Return State of Charge percentage approximation (integer) per voltage level. See main.yaml. """
        # use numpy to figure out the closest value
        difference_array = np.absolute(self.soc_steps - voltage)
        index = difference_array.argmin()
        return index * 10


    def get_termcolor_for_voltage(self, voltage):
        if voltage >= self.charging_voltage:
            return "\033[94m"
        elif voltage <= self.red_alert_voltage:
            return "\033[91m"
        elif voltage <= self.yellow_alert_voltage:
            return "\033[93m"
        else:
            return "\033[92m"


    def get_termcolor_for_soc(self, soc):
        if soc <= 20:
            return "\033[91m"
        elif soc <= 40:
            return "\033[93m"
        else:
            return "\033[92m"


    def odom_travelled_distance_callback(self, msg):
        """Callback to get travelled distance raw value (in meters) from Roboclaw encoders.

           NOTE: this value is raw data from the encoders, and resets when roboclaw is powered off.
           When the robot moves backwards, this reading decreases.

           Movement delta (difference) since the last reading is calculated at every reading.

           The total travelled distance is calculated by adding the delta value up with logged reading,
           and once a minute is saved to a file which is read at startup.
        """
        raw_travelled_distance = fabs(msg.data)
        # self.logger.debug(f"Travelled distance (abs raw): {raw_travelled_distance:.3f} m")

        # if there is no change, noop
        if raw_travelled_distance == self.odom_last_raw_travelled_distance:
            return

        # detect if roboclaw was reset since the last reading
        if raw_travelled_distance < 0.01 and raw_travelled_distance < self.odom_last_raw_travelled_distance:
            self.logger.debug("Roboclaw was reset?")
            # reset last reading
            self.odom_last_raw_travelled_distance = 0.0

        # roboclaw is going backwards
        if raw_travelled_distance < self.odom_last_raw_travelled_distance:
            self.logger.debug(f"Roboclaw travel reading is decreasing")
            raw_movement_delta = self.odom_last_raw_travelled_distance - raw_travelled_distance

        # going forwards
        else:
            raw_movement_delta = raw_travelled_distance - self.odom_last_raw_travelled_distance

        self.logger.debug(f"Roboclaw movement diff {raw_movement_delta:.3f} m")

        # remember last raw value
        self.odom_last_raw_travelled_distance = raw_travelled_distance

        # calculate total travelled distance
        session_travelled_distance = self.odom_total_travelled_distance + raw_movement_delta
        if session_travelled_distance > self.odom_total_travelled_distance:
            # robot moved since the last report
            self.odom_total_travelled_distance = session_travelled_distance

            # print info log:
            # Movement since session start xxx,x m (x,xx km, odo xx,x km) at hh:mm:ss (dd:mm:yyyy)
            odom_info = f"Movement since session start {session_travelled_distance:.2f} m"
            if session_travelled_distance >= 1000.0:
                odom_info += f" ({(session_travelled_distance / 1000.0):.3f} km)"
            odom_info += f" (total travel {(self.odom_total_travelled_distance + self.odom_startup_offset):.2f} m)"
            odom_info += f" at {datetime.now().strftime('%H:%M (%d.%m.%Y)')}"
            self.logger.info(odom_info)

            # save total distance to odom log file (once a minute)
            if not self.odom_last_logwrite or \
                (datetime.now() - (self.odom_last_logwrite + timedelta(seconds=self.odom_log_interval))).total_seconds() >= 0:
                self.odom_write_log()

        elif session_travelled_distance < self.odom_total_travelled_distance:
            self.logger.warning(f"Total travelled distance decreased, should not be possible! {session_travelled_distance:.3f} < {self.odom_total_travelled_distance:.3f}")


    def odom_write_log(self):
        """Write total distance travelled (odometry) data to text file"""
        if self.odom_total_travelled_distance:
            try:
                with open(self.odom_logfilepath, 'w') as file:
                    file.write(str(self.odom_total_travelled_distance + self.odom_startup_offset))
                    self.odom_last_logwrite = datetime.now()
                    self.logger.debug(f"Saved odom data to {self.odom_logfilepath}")
            except Exception as err:
                self.logger.warning(str(err))


    def stats_csv_write_log(self):
        """Write health stats to CSV file"""
        try:
            ts = datetime.now()

            # format csv data
            csv_data = ", ".join([
                # timestamp
                ts.isoformat('T', 'seconds'),
                # main battery voltage
                f"{self.main_battery_average_voltage:.2f}",
                # estimated SoC level
                f"{self.get_soc_level(self.main_battery_average_voltage):.2f}",
                # vacuum total runtime
                f"{self.stats_vacuum_total_runtime:.1f}",
                # vacuum on/off cycles
                str(self.stats_vacuum_total_onoff_cycles),
                # odom session travelled distance
                f"{(self.odom_total_travelled_distance - self.odom_startup_offset):.2f}",
                # odom total travelled distance
                f"{self.odom_total_travelled_distance:.2f}",
                # roboclaw M1 temp
                f"{self.stats_roboclaw_temp1:.1f}",
                # roboclaw M2 temp
                f"{self.stats_roboclaw_temp2:.1f}"
            ])

            # open csv file by timestamp
            csv_filepath = Path(ts.strftime(self.stats_csv_logfiletemplate)).expanduser()

            # write comment to the top of new file
            if not os.path.exists(csv_filepath):
                with open(csv_filepath, 'w') as file:
                    file.write("; timestamp, main battery voltage, SoC, vacuum runtime, vacuum cycles, odom session, odom total, roboclaw temp1, roboclaw temp2\n")

            # write data
            with open(csv_filepath, 'a') as file:
                file.write(csv_data + '\n')
                self.logger.debug(f"Saved stats csv data to {csv_filepath}")

        except Exception as err:
            self.logger.warning(str(err))


    def motor_temperatures_callback(self, msg):
        self.stats_roboclaw_temp1 = msg.data[0]
        self.stats_roboclaw_temp2 = msg.data[1]
        self.logger.debug(f'Roboclaw temperatures M1: {self.stats_roboclaw_temp1:.1f}°C, M2: {self.stats_roboclaw_temp2:.1f}°C')


    def motor_pwms_callback(self, msg):
        pwm1 = msg.data[0]
        pwm2 = msg.data[1]
        self.logger.info(f'Roboclaw PWM 1: {pwm1:.1f}%, 2: {pwm2:.1f}%')


    def motor_currents_callback(self, msg):
        current1 = msg.data[0]
        current2 = msg.data[1]
        self.logger.info(f'Roboclaw current 1: {current1:.1f} A, 2: {current2:.1f} A')


    def vacuum_session_stats_callback(self, msg):
        self.stats_vacuum_total_runtime = msg.data[0]
        self.stats_vacuum_total_onoff_cycles = int(msg.data[1])
        self.logger.debug(f"Vacuum runtime {self.stats_vacuum_total_runtime:.1f} s, {self.stats_vacuum_total_onoff_cycles} on/off cycles")


def main(args=None):
    rclpy.init(args=args)
    health_node = HealthNode()

    # rclpy.spin(health_node, executor=MultiThreadedExecutor())
    rclpy.spin(health_node)

    health_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

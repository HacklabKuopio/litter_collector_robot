# encoding: utf-8

# Copyright 2023-2026 Mikael Lammentausta
#
# Licensed under the Apache License, Version 2.0.
# See the LICENSE file in the project root for license information.

import logging
from datetime import datetime, timedelta

import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity

from std_msgs.msg import Bool, Int16, Float32MultiArray
from example_interfaces.msg import String # for audio

try:
    import RPi.GPIO as GPIO
except:
    import Mock.GPIO as GPIO


class VacuumNode(Node):
    def __init__(self):
        super().__init__('vacuum_node')
        # Configure logger
        self.logger = self.get_logger()
        self.logger.set_level(LoggingSeverity.DEBUG)
        self.logger.info("VacuumNode init")

        # stats logger
        logging.basicConfig(
            filename='/dev/null',
            # stream=sys.stdout,
            encoding="utf-8",
            level=logging.DEBUG,
            # format="%(asctime)s - %(levelname)s - %(message)s",
            datefmt="%d.%m.%Y %H:%M:%S")

        self.statistics_logger = logging.getLogger('statistics')
        stats_handler = logging.FileHandler('stats.log', 'a')
        stats_handler.setFormatter(logging.Formatter('%(asctime)s - %(levelname)s - %(message)s'))
        self.statistics_logger.addHandler(stats_handler)

        # subscriptions
        self.create_subscription(
            Bool,
            'lcr/vacuum/toggle',
            self.vacuum_toggle_callback,
            10
        )
        self.create_subscription(
            Int16,
            'lcr/vacuum/timeout_change',
            self.vacuum_timeout_change_callback,
            10
        )
        self.create_subscription(
            Bool,
            'lcr/emergency_stop_alert',
            self.emergency_stop_alert_callback,
            1
        )

        # publishers
        self.vacuum_session_stats_pub = self.create_publisher(
            Float32MultiArray,
            'lcr/vacuum/session_stats',
            100
        )
        self.audio_pub = self.create_publisher(
            String,
            'lcr/audio/play',
            100
        )

        # vacuum variables
        self.vacuum_latched_on = False
        self.vacuum_started_at = 0
        self.vacuum_last_buttonpress_at = 0
        self.vacuum_latch_buttonpress_duration = 3.0
        self.vacuum_standard_timeout = 3.0 # Initial vacuum timer value in seconds
        self.vacuum_latched_timeout = 60.0 # Latched vacuum timeout
        self.vacuum_timeout = self.vacuum_standard_timeout
        self.vacuum_current_runtime = 0
        self.vacuum_cooldown_until = datetime.now()
        self.vacuum_timer = None
        self.vacuum_total_runtime = 0
        self.vacuum_total_onoff_cycles = 0
        self.vacuum_min_standard_timeout = 3.0
        self.vacuum_max_standard_timeout = 9.0

        self.state_emergency_stop_alert = False

        # Read GPIO pin ports from config
        self.declare_parameter("vacuum_pin", rclpy.Parameter.Type.INTEGER)
        self.vacuum_pin = self.get_parameter("vacuum_pin").value

        # Init Raspberry Pi GPIO pins
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.vacuum_pin, GPIO.OUT, initial=GPIO.LOW)


    def __del__(self):
        GPIO.cleanup()


    def vacuum_timeout_change_callback(self, msg):
        delta = msg.data

        # Adjust vacuum cleaner timer
        if delta < 0:
            if not self.vacuum_latched_on:
                self.vacuum_standard_timeout = max(self.vacuum_min_standard_timeout, self.vacuum_standard_timeout - 1.0)
                self.logger.info(f'Vacuum timeout set to {self.vacuum_standard_timeout:.2f} sec', throttle_duration_sec=1)
                self.audio_pub.publish(String(data="vacuum_off_delay_decrease"))
            else:
                # adjust latch time
                self.vacuum_latched_timeout = max(30.0, self.vacuum_latched_timeout - 15.0)
                self.logger.info(f'Vacuum latch timeout set to {self.vacuum_latched_timeout} s')
                self.audio_pub.publish(String(data="vacuum_latch_time_decrease"))
        elif delta > 0:
            if not self.vacuum_latched_on:
                self.vacuum_standard_timeout = min(self.vacuum_max_standard_timeout, self.vacuum_standard_timeout + 1.0)
                self.logger.info(f'Vacuum timeout set to {self.vacuum_standard_timeout:.2f} sec', throttle_duration_sec=1)
                self.audio_pub.publish(String(data="vacuum_off_delay_increase"))
            else:
                # adjust latch time
                self.vacuum_latched_timeout = min(120.0, self.vacuum_latched_timeout + 15.0)
                self.logger.info(f'Vacuum latch timeout set to {self.vacuum_latched_timeout} s')
                self.audio_pub.publish(String(data="vacuum_latch_time_increase"))


    def vacuum_toggle_callback(self, msg):
        """Start vacuum cleaner for a few seconds."""
        if self.state_emergency_stop_alert:
            return

        # check for cooldown period
        if (datetime.now() - self.vacuum_cooldown_until).total_seconds() < 0:
            self.logger.debug("Vacuum cooldown period", throttle_duration_sec=1.0)
            return

        # vacuum not yet started
        if self.vacuum_started_at == 0:
            self.vacuum_total_onoff_cycles += 1
            self.vacuum_started_at = datetime.now()
            log_msg = f"Vacuum ON at {self.vacuum_started_at.strftime('%H:%M:%S')}"
            self.logger.info(log_msg)
            self.statistics_logger.info(log_msg)

        # set vacuum latch on after certain period with buttonpress on
        # (this method is called only while the button is being pressed)
        if not self.vacuum_latched_on and self.vacuum_current_runtime > self.vacuum_latch_buttonpress_duration:
            self.logger.info(f"Vacuum latched on; set timeout to {self.vacuum_latched_timeout:.2f} sec")
            self.vacuum_latched_on = True
            self.vacuum_timer.cancel()
            self.audio_pub.publish(String(data="vacuum_latch"))

        # cancel latch
        # NOTE: button should be released within 2.5 sec, or else the latch will cancel!
        elif self.vacuum_latched_on and self.vacuum_current_runtime > self.vacuum_latch_buttonpress_duration + 2.5:
            self.logger.info(f"Vacuum latch canceled; stop vacuum")
            self.vacuum_latched_on = False
            self.vacuum_timer.cancel()
            self.vacuum_callback()
            return

        # turn vacuum pin on
        GPIO.output(self.vacuum_pin, GPIO.HIGH)

        # set vacuum timeout
        if self.vacuum_latched_on:
            self.vacuum_timeout = self.vacuum_latched_timeout
        else:
            self.vacuum_timeout = self.vacuum_standard_timeout

        # set callback timer to 1 sec
        if self.vacuum_timer and not self.vacuum_timer.is_canceled():
            self.vacuum_current_runtime = (datetime.now() - self.vacuum_started_at).total_seconds()
            log_msg = f'Current runtime {self.vacuum_current_runtime:.0f} s (timeout {self.vacuum_timeout})'
            self.logger.info(log_msg, throttle_duration_sec=1.0)
            # reset timer
            self.vacuum_timer.reset()
        else:
            # set callback timer
            self.vacuum_timer = self.create_timer(1.0, self.vacuum_callback)


    def vacuum_callback(self):
        """Vacuum stop timer callback."""
        self.vacuum_timer.cancel()

        # set vacuum timeout
        if self.vacuum_latched_on:
            self.vacuum_timeout = self.vacuum_latched_timeout
        else:
            self.vacuum_timeout = self.vacuum_standard_timeout

        self.vacuum_current_runtime = (datetime.now() - self.vacuum_started_at).total_seconds()

        log_msg = f'Current runtime {self.vacuum_current_runtime:.0f} s (timeout {self.vacuum_timeout})'
        self.logger.info(log_msg)
        self.statistics_logger.info(log_msg)

        if self.vacuum_current_runtime < self.vacuum_timeout:
            # set callback timer to 1 sec
            self.vacuum_timer = self.create_timer(1.0, self.vacuum_callback)
        else:
            # elif not self.vacuum_latched_on:
            self.stop_vacuum()


    def stop_vacuum(self):
        """Stop vacuum."""
        if self.vacuum_started_at:
            vacuum_current_runtime = (datetime.now() - self.vacuum_started_at).total_seconds()
            self.vacuum_total_runtime += vacuum_current_runtime
            log_msg = f'Vacuum OFF;' +\
                f' {vacuum_current_runtime:.1f} s' +\
                f' ({self.vacuum_total_onoff_cycles} cycles,' +\
                f' total runtime {self.vacuum_total_runtime:.1f} s)'
            self.logger.info(log_msg)
            self.statistics_logger.info(log_msg)
        self.vacuum_latched_on = False
        self.vacuum_started_at = 0
        self.vacuum_current_runtime = 0

        # publish the data for health node
        self.vacuum_session_stats_pub.publish(Float32MultiArray(data=[
            self.vacuum_total_runtime,
            float(self.vacuum_total_onoff_cycles)
        ]))

        # set vacuum GPIO pin off
        self.logger.debug(f'Vacuum OFF; output GPIO{self.vacuum_pin} LOW', throttle_duration_sec=2)
        GPIO.output(self.vacuum_pin, GPIO.LOW)

        # set vacuum cooloff (so that it does not start immediately after stopping)
        # self.logger.debug("Set vacuum cooldown flag on")
        self.vacuum_cooldown_until = datetime.now() + timedelta(seconds=3.0)


    def emergency_stop_alert_callback(self, msg):
        self.state_emergency_stop_alert = msg.data
        if self.vacuum_timer:
            self.vacuum_timer.cancel()
        self.stop_vacuum()


def main(args=None):
    rclpy.init(args=args)
    vacuum_node = VacuumNode()
    rclpy.spin(vacuum_node)
    vacuum_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

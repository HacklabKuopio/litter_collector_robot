# encoding: utf-8

# Copyright 2023-2026 Mikael Lammentausta
#
# Licensed under the Apache License, Version 2.0.
# See the LICENSE file in the project root for license information.

# Roboclaw errors in hex, from the manual
INFO_NORMAL                     = 0x00000000
INFO_ESTOP                      = 0x00000001
ERROR_TEMPERATURE               = 0x00000002
ERROR_TEMPERATURE2              = 0x00000004
ERROR_MAIN_VOLTAGE_HIGH         = 0x00000008
ERROR_LOGIC_VOLTAGE_HIGH        = 0x00000010
ERROR_LOGIC_VOLTAGE_LOW         = 0x00000020
ERROR_M1_DRIVER_FAULT           = 0x00000040
ERROR_M2_DRIVER_FAULT           = 0x00000080
ERROR_M1_SPEED                  = 0x00000100
ERROR_M2_SPEED                  = 0x00000200
ERROR_M1_POSITION               = 0x00000400
ERROR_M2_POSITION               = 0x00000800
ERROR_M1_CURRENT                = 0x00001000
ERROR_M2_CURRENT                = 0x00002000
WARNING_M1_OVER_CURRENT         = 0x00010000
WARNING_M2_OVER_CURRENT         = 0x00020000
WARNING_MAIN_VOLTAGE_HIGH       = 0x00040000
WARNING_MAIN_VOLTAGE_LOW        = 0x00080000
WARNING_TEMPERATURE             = 0x00100000
WARNING_TEMPERATURE2            = 0x00200000
WARNING_S4_SIGNAL_TRIGGERED     = 0x00400000
WARNING_S5_SIGNAL_TRIGGERED     = 0x00800000
WARNING_SPEED_ERROR_LIMIT       = 0x01000000
WARNING_POSITION_ERROR_LIMIT    = 0x02000000


class RoboclawErrorLogger(object):

    def __init__(self, _logger):
        self.logger = _logger


    def decode_error(self, error_code: int) -> None:
        errors = []

        self.logger.debug(f'Roboclaw Error: {"0x%0.2X" % error_code} ({error_code})')

        if error_code == INFO_NORMAL:
            self.logger.info("Normal operation")
            return

        if error_code & INFO_ESTOP:
            self.logger.info("E-Stop")
            errors.append(INFO_ESTOP)

        if error_code & ERROR_TEMPERATURE:
            self.logger.error("Temperature Error")
            errors.append(ERROR_TEMPERATURE)

        if error_code & ERROR_TEMPERATURE2:
            self.logger.error("Temperature 2 Error")
            errors.append(ERROR_TEMPERATURE2)

        if error_code & ERROR_MAIN_VOLTAGE_HIGH:
            self.logger.error("Main Voltage High Error")
            errors.append(ERROR_MAIN_VOLTAGE_HIGH)

        if error_code & ERROR_LOGIC_VOLTAGE_HIGH:
            self.logger.error("Logic Voltage High Error")
            errors.append(ERROR_LOGIC_VOLTAGE_HIGH)

        if error_code & ERROR_LOGIC_VOLTAGE_LOW:
            self.logger.error("Logic Voltage Low Error")
            errors.append(ERROR_LOGIC_VOLTAGE_LOW)

        if error_code & ERROR_M1_DRIVER_FAULT:
            self.logger.error("M1 Driver Fault Error")
            errors.append(ERROR_M1_DRIVER_FAULT)

        if error_code & ERROR_M2_DRIVER_FAULT:
            self.logger.error("M2 Driver Fault Error")
            errors.append(ERROR_M2_DRIVER_FAULT)

        if error_code & ERROR_M1_SPEED:
            self.logger.error("M1 Speed Error")
            errors.append(ERROR_M1_SPEED)

        if error_code & ERROR_M2_SPEED:
            self.logger.error("M2 Speed Error")
            errors.append(ERROR_M2_SPEED)

        if error_code & ERROR_M1_POSITION:
            self.logger.error("M1 Position Error")
            errors.append(ERROR_M1_POSITION)

        if error_code & ERROR_M2_POSITION:
            self.logger.error("M2 Position Error")
            errors.append(ERROR_M2_POSITION)

        if error_code & ERROR_M1_CURRENT:
            self.logger.error("M1 Current Error")
            errors.append(ERROR_M1_CURRENT)

        if error_code & ERROR_M2_CURRENT:
            self.logger.error("M2 Current Error")
            errors.append(ERROR_M2_CURRENT)

        if error_code & WARNING_M1_OVER_CURRENT:
            self.logger.warning("M1 Over Current Warning")
            errors.append(WARNING_M1_OVER_CURRENT)

        if error_code & WARNING_M2_OVER_CURRENT:
            self.logger.warning("M2 Over Current Warning")
            errors.append(WARNING_M2_OVER_CURRENT)

        if error_code & WARNING_MAIN_VOLTAGE_HIGH:
            self.logger.warning("Main Voltage High Warning")
            errors.append(WARNING_MAIN_VOLTAGE_HIGH)

        if error_code & WARNING_MAIN_VOLTAGE_LOW:
            self.logger.warning("Main Voltage Low Warning")
            errors.append(WARNING_MAIN_VOLTAGE_LOW)

        if error_code & WARNING_TEMPERATURE:
            self.logger.warning("Temperature Warning")
            errors.append(WARNING_TEMPERATURE)

        if error_code & WARNING_TEMPERATURE2:
            self.logger.warning("Temperature 2 Warning")
            errors.append(WARNING_TEMPERATURE2)

        if error_code & WARNING_S4_SIGNAL_TRIGGERED:
            self.logger.warning("S4 Signal Triggered")
            errors.append(WARNING_S4_SIGNAL_TRIGGERED)

        if error_code & WARNING_S5_SIGNAL_TRIGGERED:
            self.logger.warning("S5 Signal Triggered")
            errors.append(WARNING_S5_SIGNAL_TRIGGERED)

        if error_code & WARNING_SPEED_ERROR_LIMIT:
            self.logger.warning("Speed Error Limit Warning")
            errors.append(WARNING_SPEED_ERROR_LIMIT)

        if error_code & WARNING_POSITION_ERROR_LIMIT:
            self.logger.warning("Position Error Limit Warning")
            errors.append(WARNING_POSITION_ERROR_LIMIT)

        # check for remaining hex code
        sanity_check = error_code
        for error in errors:
            sanity_check -= error

        if sanity_check != 0:
            self.logger.warning(f"Unknown error code leftover: 0x{format(sanity_check, '08x')}")



if __name__ == '__main__':
    import logging
    logging.basicConfig(level=logging.INFO)
    RoboclawErrorLogger(logging).decode_error(0x40068001)


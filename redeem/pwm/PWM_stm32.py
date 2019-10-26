"""
This is an implementation of the PWM controlled by an STM32 uC

Author: Elias Bakken
email: elias(at)iagent(dot)no
Website: http://www.thing-printer.com
License: GNU GPL v3: http://www.gnu.org/copyleft/gpl.html

 Redeem is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Redeem is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Redeem.  If not, see <http://www.gnu.org/licenses/>.
"""

from PWM import PWM_Output


class PWM_stm32_Output(PWM_Output):
  def __init__(self, uart, channel):
    self.uart = uart
    self.channel = channel

  def set_value(self, value):
    self.uart.write("PWM:%i:%i".format(self.channel, value))


class PWM_stm32(object):
  def __init__(self, uart):
    self.uart = uart

  def get_output(self, channel):
    return PWM_stm32_Output(self, channel)


if __name__ == "__main__":
  import serial
  uart = serial.Serial('/dev/ttyS1', 115200, timeout=0.1)
  pwm = PWM_stm32(uart, 0)
  pwm.set_value(0.5)

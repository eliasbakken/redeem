import logging
import glob
import struct
import time

from redeem.Enable import Enable
from redeem.EndStop import EndStop
from redeem.Key_pin import Key_pin, Key_pin_listener
from redeem.Mosfet import Mosfet
from redeem.Fan import Fan
from redeem.GPIO import AM335x_GPIO_Controller
from redeem.pwm.PWM_stm32 import PWM_stm32
from redeem.steppers.Stepper_TMC2209 import StepperBankUart, Stepper_TMC2209

import redeem.SPI as SPI


def probe_recore(printer):
  results = find_recore()
  if results == None:
    return

  eeprom_path, eeprom_data, revision = results
  printer.key = get_recore_key(eeprom_path, eeprom_data)
  build_recore_printer(printer, revision)


def find_recore():
  found = False
  revision = None
  eeprom_data = None
  eeprom_path = None

  nvmem_path = "/sys/bus/nvmem/devices/sunxi-sid0/nvmem"
  try:
    with open(nvmem_path, "rb") as f:
      data = f.read(16)
      id = data[0:4]
      if id == '\x92\xc0\x00\xba':
        revision = "A0"
        eeprom_data = data
        eeprom_path = nvmem_path
        found = True
  except IOError:
    pass

  if not found:
    return None

  logging.debug("Found recore revision %s at %s", revision, eeprom_path)
  return eeprom_path, eeprom_data, revision


def get_recore_key(eeprom_path, eeprom_data):
  """ Get the generated key from the config or create one """
  recore_key = "".join(struct.unpack('7c',
                                     eeprom_data[5:12]))    # TODO this is probably wrong as well
  logging.debug("Found recore key: '" + recore_key + "'")
  if recore_key == '\x00' * 7:
    logging.debug("recore key invalid")
    import random
    import string
    recore_key = ''.join(
        random.SystemRandom().choice(string.ascii_uppercase + string.digits) for _ in range(7))
    eeprom_data = eeprom_data[0:5] + recore_key + eeprom_data[12:]
    logging.debug("New recore key: '" + recore_key + "'")
    try:
      with open(eeprom_path, "wb") as f:
        f.write(eeprom_data[:120])
    except IOError as e:
      logging.warning("Unable to write new key to EEPROM")
  return recore_key


def build_recore_printer(printer, revision):
  import serial
  printer.NUM_AXES = 6
  printer.board = "recore"
  printer.revision = revision

  gpio = AM335x_GPIO_Controller()

  uart1 = serial.Serial('/dev/ttyS2', 115200, timeout=0.1)
  pwm = PWM_stm32(uart1)

  # Enable PWM and steppers
  printer.enable = Enable(gpio.output(6, 1), False)
  printer.enable.set_disabled()
  #time.sleep(1)
  printer.enable.set_enabled()
  #time.sleep(1)

  # Init the end stops
  EndStop.inputdev = "/dev/input/by-path/platform-switches-event"
  Key_pin.listener = Key_pin_listener(EndStop.inputdev)

  printer.end_stop_inputs["X1"] = gpio.input(1, 28)
  printer.end_stop_inputs["X2"] = gpio.input(1, 17)
  printer.end_stop_inputs["Y1"] = gpio.input(1, 27)
  printer.end_stop_inputs["Y2"] = gpio.input(1, 29)
  printer.end_stop_inputs["Z1"] = gpio.input(1, 14)
  printer.end_stop_inputs["Z2"] = gpio.input(1, 15)

  printer.end_stop_keycodes["X1"] = 112
  printer.end_stop_keycodes["X2"] = 113
  printer.end_stop_keycodes["Y1"] = 114
  printer.end_stop_keycodes["Y2"] = 115
  printer.end_stop_keycodes["Z1"] = 116
  printer.end_stop_keycodes["Z2"] = 117

  uart = serial.Serial('/dev/ttyS2', 115200, timeout=0.1)
  stepper_bank = StepperBankUart([uart], 1)

  printer.steppers["X"] = Stepper_TMC2209(
      gpio.output(3, 14), gpio.output(3, 15), gpio.input(0, 27), "X", stepper_bank, 0)
  '''printer.steppers["Y"] = Stepper_TMC2130(
      gpio.output(3, 16), gpio.output(3, 17), gpio.input(2, 0), "Y", stepper_bank, 1)
  printer.steppers["Z"] = Stepper_TMC2130(
      gpio.output(3, 18), gpio.output(3, 19), gpio.input(1, 16), "Z", stepper_bank, 2)
  printer.steppers["E"] = Stepper_TMC2130(
      gpio.output(3, 20), gpio.output(3, 21), gpio.input(2, 1), "E", stepper_bank, 3)
  printer.steppers["H"] = Stepper_TMC2130(
      gpio.output(2, 26), gpio.output(2, 27), gpio.input(0, 29), "H", stepper_bank, 4)
  printer.steppers["A"] = Stepper_TMC2130(
      gpio.output(2, 28), gpio.output(2, 29), gpio.input(0, 26), "A", stepper_bank, 5)
  '''
  for axis, stepper in printer.steppers.iteritems():
    stepper.initialize_registers()
    stepper.sanity_test()

  stepper_bank.start_watcher_thread()

  printer.mosfets["E"] = Mosfet(pwm.get_output(0))
  printer.mosfets["H"] = Mosfet(pwm.get_output(1))
  printer.mosfets["A"] = Mosfet(pwm.get_output(2))
  printer.mosfets["HBP"] = Mosfet(pwm.get_output(3))

  printer.thermistor_inputs["E"] = "/sys/bus/iio/devices/iio:device0/in_voltage0_raw"
  printer.thermistor_inputs["H"] = "/sys/bus/iio/devices/iio:device0/in_voltage1_raw"
  printer.thermistor_inputs["A"] = "/sys/bus/iio/devices/iio:device0/in_voltage3_raw"
  printer.thermistor_inputs["HBP"] = "/sys/bus/iio/devices/iio:device0/in_voltage2_raw"
  printer.thermistor_inputs["BOARD"] = "/sys/bus/iio/devices/iio:device0/in_voltage4_raw"
  printer.thermistor_inputs["I"] = "/sys/bus/iio/devices/iio:device0/in_voltage6_raw"
  printer.thermistor_inputs["U"] = "/sys/bus/iio/devices/iio:device0/in_voltage7_raw"

  printer.fans.append(Fan(pwm.get_output(4)))
  printer.fans.append(Fan(pwm.get_output(5)))
  printer.fans.append(Fan(pwm.get_output(6)))
  printer.fans.append(Fan(pwm.get_output(7)))

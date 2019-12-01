import ctypes
import logging
import math
import threading
import time
from Stepper import Stepper


def int_to_bytes(value):
  return [
      int((value & 0xff000000) >> 24),
      int((value & 0xff0000) >> 16),
      int((value & 0xff00) >> 8),
      int((value & 0xff))
  ]


def bytes_to_int(bytes):
  return bytes[0] << 24 | bytes[1] << 16 | bytes[2] << 8 | bytes[3]


class StepperBankUart(object):
  def __init__(self, uarts, stepper_count):
    self.uarts = uarts
    self.lock = threading.Lock()
    self.steppers = [None] * stepper_count

  def add_stepper(self, index, stepper):
    self.steppers[index] = stepper

  def _update_status_from_transfer(self, data):
    for index, stepper in enumerate(self.steppers):
      if stepper is not None:
        reverse_index = len(self.steppers) - index - 1
        stepper.update_status(data[reverse_index * 5] & 0xf)

  def _index_to_uart_addr(self, index):
    return index % 4, self.uarts[index / 4]

  def _crc_byte(self, crc, byte):
    if (crc >> 7) ^ byte & 1:
      crc = (crc << 1) ^ 7
    else:
      crc = crc << 1
    byte = byte >> 1

  def _add_crc(self, bytes):
    crc = 0
    for byte in bytes:
      for i in xrange(8):
        if (crc >> 7) ^ byte & 1:
          crc = (crc << 1) ^ 0x7
        else:
          crc = crc << 1
        byte = byte >> 1
        crc = crc & 0xFF
    return bytes + [crc]

  def _single_register_transfer(self, index, reg_addr, value, write):
    with self.lock:
      stepper_addr, uart = self._index_to_uart_addr(index)

      if write:
        to_send = self._add_crc([0x5, stepper_addr, reg_addr | 0x80] + int_to_bytes(value))
      else:
        to_send = self._add_crc([0x5, stepper_addr, reg_addr])

      #print("write", [hex(byte) for byte in to_send])
      uart.write("".join([chr(byte) for byte in to_send]))
      # Discard what we just sent
      simplex = uart.read(len(to_send))
      data = [ord(byte) for byte in uart.read(100)]
      if write:
        return
      data = data[3:7]
      self.steppers[index].update_register(reg_addr, bytes_to_int(data))

  def read_single_register(self, index, addr):
    self._single_register_transfer(index, addr, 0, False)

  def write_single_register(self, index, addr):
    self._single_register_transfer(index, addr, self.steppers[index].get_register_value(addr), True)

  def _bulk_register_transfer(self, addr, values, write):
    with self.lock:
      if write and len(values) != len(self.steppers):
        raise RuntimeError("UART bulk write failed - value count must match stepper count")

    for index in range(len(self.steppers)):
      if values is not None:
        value = values[index]
        self._single_register_transfer(index, addr, value, write)
      else:
        self._single_register_transfer(index, addr, 0, write)

  def read_all_registers(self, addr):
    self._bulk_register_transfer(addr, None, False)

  def write_all_registers(self, addr):
    values = [
        stepper.get_register_value(addr) if stepper is not None else None
        for stepper in self.steppers
    ]
    self._bulk_register_transfer(addr, values, True)

  def start_watcher_thread(self):
    self.t = threading.Thread(target=self.watch_status, name="Stepper Status Watcher")
    self.t.daemon = True
    self.t.start()

  def watch_status(self):
    time.sleep(2)
    while True:
      time.sleep(0.1)
      self.read_all_registers(self.steppers[0].gstat.addr)
      self.read_all_registers(self.steppers[0].drv_status.addr)
      self.read_all_registers(self.steppers[0].lost_steps.addr)


c_uint = ctypes.c_uint32

# don't let the formatter mangle these
# yapf: disable

class GCONF_Register_bits(ctypes.Structure):
  _fields_ = [
      ("i_scale_analog", c_uint, 1),
      ("internal_rsense", c_uint, 1),
      ("en_spreadcycle", c_uint, 1),
      ("shaft", c_uint, 1),
      ("index_otpw", c_uint, 1),
      ("index_step", c_uint, 1),
      ("pdn_disable", c_uint, 1),
      ("mstep_reg_select", c_uint, 1),
      ("multistep_filt", c_uint, 1),
      ("test_mode", c_uint, 1),
  ]


class GCONF_Register_data(ctypes.Union):
  _fields_ = [("bits", GCONF_Register_bits), ("register", c_uint)]


class GSTAT_Register_bits(ctypes.Structure):
  _fields_ = [
      ("reset", c_uint, 1),
      ("drv_err", c_uint, 1),
      ("uv_cp", c_uint, 1)
  ]


class GSTAT_Register_data(ctypes.Union):
  _fields_ = [("bits", GSTAT_Register_bits), ("register", c_uint)]

class IFCNT_Register_bits(ctypes.Structure):
  _fields_ = [
      ("uart", c_uint, 8)
  ]

class IFCNT_Register_data(ctypes.Union):
  _fields_ = [("bits", IFCNT_Register_bits), ("register", c_uint)]

class SLAVECONF_Register_bits(ctypes.Structure):
  _fields_ = [
      ("padding", c_uint, 8),
      ("senddelay", c_uint, 4)
  ]

class SLAVECONF_Register_data(ctypes.Union):
  _fields_ = [("bits", SLAVECONF_Register_bits), ("register", c_uint)]

class IOIN_Register_bits(ctypes.Structure):
  _fields_ = [
      ("enn", c_uint, 1),
      ("0", c_uint, 1),
      ("ms1", c_uint, 1),
      ("ms2", c_uint, 1),
      ("diag", c_uint, 1),
      ("0", c_uint, 1),
      ("pdn_uart", c_uint, 1),
      ("step", c_uint, 1),
      ("spread_en", c_uint, 1),
      ("dir", c_uint, 1),
      ("padding", c_uint, 14),
      ("version", c_uint, 8)
  ]


class IOIN_Register_data(ctypes.Union):
  _fields_ = [("bits", IOIN_Register_bits), ("register", c_uint)]


class IHOLD_IRUN_Register_bits(ctypes.Structure):
  _fields_ = [
      ("ihold", c_uint, 5),
      ("irun", c_uint, 5),
      ("iholddelay", c_uint, 4)
  ]


class IHOLD_IRUN_Register_data(ctypes.Union):
  _fields_ = [("bits", IHOLD_IRUN_Register_bits), ("register", c_uint)]


class CHOPCONF_Register_bits(ctypes.Structure):
  _fields_ = [
      ("toff", c_uint, 4),
      ("hstrt", c_uint, 3),
      ("hend", c_uint, 4),
      ("fd3", c_uint, 1),
      ("disfdcc", c_uint, 1),
      ("rndtf", c_uint, 1),
      ("chm", c_uint, 1),
      ("tbl", c_uint, 2),
      ("vsense", c_uint, 1),
      ("vhighfs", c_uint, 1),
      ("vhighchm", c_uint, 1),
      ("sync", c_uint, 4),
      ("mres", c_uint, 4),
      ("intpol", c_uint, 1),
      ("dedge", c_uint, 1),
      ("diss2g", c_uint, 1),
      ("reserved", c_uint, 1)
  ]


class CHOPCONF_Register_data(ctypes.Union):
  _fields_ = [("bits", CHOPCONF_Register_bits), ("register", c_uint)]


class COOLCONF_Register_bits(ctypes.Structure):
  _fields_ = [
      ("semin", c_uint, 4),
      ("reserved", c_uint, 1),
      ("seup", c_uint, 2),
      ("reserved2", c_uint, 1),
      ("semax", c_uint, 4),
      ("reserved3", c_uint, 1),
      ("sedn", c_uint, 2),
      ("seimin", c_uint, 1),
      ("sgt", c_uint, 7),
      ("reserved4", c_uint, 1),
      ("sfilt", c_uint, 1),
      ("reserved5", c_uint, 7),
  ]


class COOLCONF_Register_data(ctypes.Union):
  _fields_ = [("bits", COOLCONF_Register_bits), ("register", c_uint)]


class PWMCONF_Register_bits(ctypes.Structure):
  _fields_ = [
      ("pwm_ampl", c_uint, 8),
      ("pwm_grad", c_uint, 8),
      ("pwm_freq", c_uint, 2),
      ("pwm_autoscale", c_uint, 1),
      ("pwm_symmetric", c_uint, 1),
      ("freewheel", c_uint, 2),
      ("reserved", c_uint, 10),
  ]


class PWMCONF_Register_data(ctypes.Union):
  _fields_ = [("bits", PWMCONF_Register_bits), ("register", c_uint)]


class DRV_STATUS_Register_bits(ctypes.Structure):
  _fields_ = [
      ("otpw", c_uint, 1),
      ("ot", c_uint, 1),
      ("s2ga", c_uint, 1),
      ("s2gb", c_uint, 1),
      ("s2vsa", c_uint, 1),
      ("s2vsb", c_uint, 1),
      ("ola", c_uint, 1),
      ("olb", c_uint, 1),
      ("t120", c_uint, 1),
      ("t143", c_uint, 1),
      ("t150", c_uint, 1),
      ("t157", c_uint, 1),
      ("reserved1", c_uint, 4),
      ("cs_actual", c_uint, 5),
      ("reserved2", c_uint, 9),
      ("stealth", c_uint, 1),
      ("stst", c_uint, 1),
  ]


class DRV_STATUS_Register_data(ctypes.Union):
  _fields_ = [("bits", DRV_STATUS_Register_bits), ("register", c_uint)]


class Generic_Register_data(ctypes.Structure):
  _fields_ = [("register", c_uint)]

# yapf: enable


class Register(object):
  def __init__(self, name, addr, mode, data):
    self.name = name
    self.addr = addr
    self.mode = mode
    self.data = data
    self.data.register = 0

  def can_readback(self):
    return 'r' in self.mode and not 'c' in self.mode


class Stepper_TMC2209(Stepper):
  def __init__(self, stepPin, dirPin, faultPin, name, bank, bank_index):
    Stepper.__init__(self, stepPin, dirPin, faultPin, -1, name)
    logging.debug("Adding stepper with step {}, dir {}".format(stepPin, dirPin))

    self.name = name
    self.bank = bank
    self.bank_index = bank_index
    self.bank.add_stepper(bank_index, self)
    self.current_enabled = False

    self.gconf = Register("gconf", 0x0, "rw", GCONF_Register_data())
    self.gstat = Register("gstat", 0x01, "rc", GSTAT_Register_data())
    self.ifcnt = Register("ifcnt", 0x02, "r", IFCNT_Register_data())
    self.slaveconf = Register("slaveconf", 0x03, "w", SLAVECONF_Register_data())
    self.ioin = Register("ioin", 0x06, "r", IOIN_Register_data())
    self.ihold_irun = Register("ihold_irun", 0x10, "w", IHOLD_IRUN_Register_data())
    self.tpowerdown = Register("tpowerdown", 0x11, "w", Generic_Register_data())
    self.tstep = Register("tstep", 0x12, "r", Generic_Register_data())
    self.tpwmthrs = Register("tpwmthrs", 0x13, "w", Generic_Register_data())
    self.tcoolthrs = Register("tcoolthrs", 0x14, "w", Generic_Register_data())
    self.vdcmin = Register("vdcmin", 0x33, "w", Generic_Register_data())
    self.chopconf = Register("chopconf", 0x6C, "rw", CHOPCONF_Register_data())
    self.coolconf = Register("coolconf", 0x42, "w", COOLCONF_Register_data())
    self.dcctrl = Register("dcctrl", 0x6E, "w", Generic_Register_data())
    self.drv_status = Register("drv_status", 0x6F, "r", DRV_STATUS_Register_data())
    self.pwmconf = Register("pwmconf", 0x70, "w", PWMCONF_Register_data())
    self.pwm_scale = Register("pwm_scale", 0x71, "r", Generic_Register_data())
    self.lost_steps = Register("lost_steps", 0x73, "r", Generic_Register_data())

    self.registers = [
        self.gconf, self.gstat, self.ifcnt, self.slaveconf, self.ioin, self.ihold_irun,
        self.tpowerdown, self.tstep, self.tpwmthrs, self.tcoolthrs, self.chopconf, self.coolconf,
        self.drv_status, self.pwmconf, self.pwm_scale, self.lost_steps
    ]

    self.registers_by_addr = {}
    for register in self.registers:
      self.registers_by_addr[register.addr] = register

  def initialize_registers(self):
    #print self.bank.read_single_register(0, 6)
    self.tpowerdown.data.register = 10
    self._write_register(self.tpowerdown)

    self.gconf.data.bits.i_scale_analog = 0
    self.gconf.data.bits.internal_rsense = 1
    self.gconf.data.bits.mstep_reg_select = 1
    self.gconf.data.bits.en_spreadcycle = 1
    self._write_register(self.gconf)

    self.chopconf.data.bits.vsense = 1    # note that this affects the current calculations in set_current_value
    self.chopconf.data.bits.tbl = 2
    self.chopconf.data.bits.toff = 0    # this is changed below
    self.chopconf.data.bits.hstrt = 4
    self.chopconf.data.bits.hend = 1
    self._write_register(self.chopconf)

    self.ihold_irun.data.bits.iholddelay = 10
    self._write_register(self.ihold_irun)

    self.pwmconf.data.bits.pwm_ampl = 200
    self.pwmconf.data.bits.pwm_grad = 1
    self.pwmconf.data.bits.pwm_autoscale = 1
    self._write_register(self.pwmconf)

    self.chopconf.data.bits.toff = 3    # set but don't write this so we start out "disabled"
    self.current_enabled = False

    self._read_register(self.gstat)

  def sanity_test(self):
    errors = 0

    self._read_register(self.ioin)
    if self.ioin.data.bits.version != 0x21:
      logging.error("sanity test failed for stepper %s - expected version %x, ioin was actually %x",
                    self.name, 0x21, self.ioin.data.bits.version)
      errors += 1
    else:
      # logging.debug("version check succeeded for stepper %s", self.name)
      pass

    old_conf = self.pwmconf.data.register
    magic = 0x00330021 + self.bank_index
    self.pwmconf.data.register = magic
    self._write_register(self.pwmconf)
    self._read_register(self.pwmconf)
    if self.pwmconf.data.register != magic:
      logging.error("sanity test failed for stepper %s: %x instead of %x", self.name,
                    self.pwmconf.data.register, magic)
      errors += 1
      if self.pwmconf.data.register != 0:
        logging.error("SANITY TEST MISMATCH FOR STEPPER %s - my index is %d, magic index is %d",
                      self.name, self.bank_index, self.pwmconf.data.register - 0x12345678)
    else:
      # logging.debug("sanity test succeeded for stepper %s", self.name)
      pass

    # Reset to the old config
    self._read_register(self.ifcnt)
    before = self.ifcnt.data.register
    self.pwmconf.data.register = old_conf
    self._write_register(self.pwmconf)
    self._read_register(self.ifcnt)
    after = self.ifcnt.data.register
    if after - before != 1:
      logging.error("SANITY TEST MISMATCH FOR STEPPER %s - before is %d, after is %d", self.name,
                    before, after)

    self.step_pin.disable()
    self.dir_pin.disable()

    self._read_register(self.ioin)
    if self.ioin.data.bits.step != 0 or self.ioin.data.bits.dir != 0:
      errors += 1
      logging.error("sanity test failed for stepper %s: ioin was %x but step and dir were zero",
                    self.name, self.ioin.data.register)

    self.step_pin.enable()
    self.dir_pin.enable()

    self._read_register(self.ioin)
    if self.ioin.data.bits.step != 1 or self.ioin.data.bits.dir != 1:
      logging.error("sanity test failed for stepper %s: ioin was %x but step and dir were one",
                    self.name, self.ioin.data.register)
      errors += 1

    self.step_pin.disable()

    self._read_register(self.ioin)
    if self.ioin.data.bits.step != 0 or self.ioin.data.bits.dir != 1:
      logging.error(
          "sanity test failed for stepper %s: ioin was %x but step was zero and dir was one",
          self.name, self.ioin.data.register)
      errors += 1

    self.dir_pin.disable()

    self._read_register(self.ioin)
    if self.ioin.data.bits.step != 0 or self.ioin.data.bits.dir != 0:
      logging.error(
          "sanity test failed for stepper %s: ioin was %x but step and dir were zero (second time)",
          self.name, self.ioin.data.register)
      errors += 1

    logging.debug("sanity test complete for stepper %s, %d errors", self.name, errors)

  def _read_register(self, register):
    self.bank.read_single_register(self.bank_index, register.addr)

  def _write_register(self, register):
    logging.debug("stepper %s writing register %s as %x", self.name, register.name,
                  register.data.register)
    self.bank.write_single_register(self.bank_index, register.addr)

    #if register.can_readback():
    #  desired_value = register.data.register
    #  self._read_register(register)
    #  if desired_value != register.data.register:
    #    logging.error("%s write failed - wrote %x, read %x", register.name, desired_value, register.data.register)
    #  else:
    #    # logging.debug("%s write succeeded", register.name)
    #    pass

  def get_register_value(self, addr):
    return self.registers_by_addr[addr].data.register

  def update_register(self, addr, value):
    register = self.registers_by_addr[addr]

    if register.data.register == value:
      # no change
      return

    if register is self.gstat:
      logging.debug("stepper %s gstat changed: %s", self.name,
                    self.get_gstat_change_as_string(register.data.register, value))
    elif register is self.drv_status:
      logging.debug("stepper %s drv_status changed: %s", self.name,
                    self.get_drv_status_change_as_string(register.data.register, value))
    elif register is self.lost_steps:
      logging.debug("stepper %s lost_steps changed: %d -> %d", self.name, register.data.register,
                    value)
    else:
      logging.debug("stepper %s register %s changed from %x to %x", self.name, register.name,
                    register.data.register, value)

    register.data.register = value

  def update_status(self, value):
    if value != self.status.register:
      self.status.register = value
      logging.debug("stepper %s status changed to %x: %s", self.name, value,
                    self.get_status_as_string())

  def get_status(self):
    return self.status.register

  def get_status_as_string(self):
    result = ""
    if self.status.bits.reset_flag != 0:
      result += "driver_reset "
    if self.status.bits.driver_error != 0:
      result += "driver_error "
    if self.status.bits.sg2 != 0:
      result += "stallguard "
    if self.status.bits.standstill != 0:
      result += "standstill "
    return result

  def make_change_prefix(self, string, bit):
    if bit:
      return "+" + string
    else:
      return "-" + string

  def get_bit_change(self, change_bit, new_bit, name):
    if not change_bit:
      return ""

    return self.make_change_prefix(name, new_bit)

  def get_gstat_change_as_string(self, old, new):
    change = GSTAT_Register_data()
    change.register = old ^ new

    new_reg = GSTAT_Register_data()
    new_reg.register = new

    result = "{:x} to {:x} ".format(old, new)
    result += self.get_bit_change(change.bits.reset, new_reg.bits.reset, "reset ")
    result += self.get_bit_change(change.bits.drv_err, new_reg.bits.drv_err, "drv_err ")
    result += self.get_bit_change(change.bits.uv_cp, new_reg.bits.uv_cp, "undervoltage ")
    return result

  def get_drv_status_change_as_string(self, old, new):
    change = DRV_STATUS_Register_data()
    change.register = int(old ^ new)

    new_reg = DRV_STATUS_Register_data()
    new_reg.register = int(new)

    result = "{:x} to {:x} ".format(old, new)

    if change.bits.cs_actual:
      result += "current_scale: {} -> {} ".format(self.drv_status.data.bits.cs_actual,
                                                  new_reg.bits.cs_actual)

    result += self.get_bit_change(change.bits.otpw, new_reg.bits.otpw, "overtemperature_warn ")
    result += self.get_bit_change(change.bits.ot, new_reg.bits.ot, "overtemperature ")
    result += self.get_bit_change(change.bits.s2vsa, new_reg.bits.s2ga,
                                  "low side short indicator phase A ")
    result += self.get_bit_change(change.bits.s2vsb, new_reg.bits.s2ga,
                                  "low side short indicator phase B ")
    result += self.get_bit_change(change.bits.s2ga, new_reg.bits.s2ga, "phase_A_short ")
    result += self.get_bit_change(change.bits.s2gb, new_reg.bits.s2gb, "phase_B_short ")
    result += self.get_bit_change(change.bits.ola, new_reg.bits.ola, "open_load_A ")
    result += self.get_bit_change(change.bits.olb, new_reg.bits.olb, "open_load_B ")
    result += self.get_bit_change(change.bits.t120, new_reg.bits.ot, "120 C comparator ")
    result += self.get_bit_change(change.bits.t143, new_reg.bits.ot, "143 C comparator ")
    result += self.get_bit_change(change.bits.t150, new_reg.bits.ot, "150 C comparator ")
    result += self.get_bit_change(change.bits.t157, new_reg.bits.ot, "157 C comparator ")
    result += self.get_bit_change(change.bits.stst, new_reg.bits.stealth, "stealth ")
    result += self.get_bit_change(change.bits.stst, new_reg.bits.stst, "standstill ")

    return result

  def set_microstepping(self, value, force_update=False):
    stealthchop = False
    interpolation = False

    if value == 0:
      # full step
      self.microsteps = 1
    elif value == 1:
      # half step
      self.microsteps = 2
    elif value == 2:
      # half step with 256 microstep interpolation
      interpolation = True
      self.microsteps = 2
    elif value == 3:
      # quarter step
      self.microsteps = 4
    elif value == 4:
      # 16th step
      self.microsteps = 16
    elif value == 5:
      # quarter step with 256 microstep interpolation
      self.microsteps = 4
      interpolation = True
    elif value == 6:
      # 16th step with 256 microstep interpolation
      self.microsteps = 16
      interpolation = True
    elif value == 7:
      # quarter step, stealthchop, 256 microstep interpolation
      self.microsteps = 4
      stealthchop = True
      interpolation = True
    elif value == 8:
      # 16th step, stealthchop, 256 microstep interpolation
      self.microsteps = 16
      stealthchop = True
      interpolation = True

    self.gconf.data.bits.en_pwm_mode = 1 if stealthchop else 0
    self.chopconf.data.bits.intpol = 1 if interpolation else 0

    # the actual formula is microsteps = 256/2^MRES, so log2(256/microsteps) = MRES
    self.chopconf.data.bits.mres = int(0.5 + math.log(256.0 / self.microsteps, 2))

    logging.debug("microsteps now %d, setting mres to %d", self.microsteps,
                  self.chopconf.data.bits.mres)

    self._write_register(self.chopconf)
    self._write_register(self.gconf)

  def set_current_value(self, i_rms):
    # we have 0.1 ohm sense resistors with high sensitivity enabled (vsense = 1)
    # calculations here come from Trinamics TMC2130 spec sheet, page 55
    # our peak RMS current with IRUN==31 is 1.06A

    # I_rms = (current_scale + 1) / 32 * I_peak
    # which solves to...
    current_scale = int(0.5 + (32 * i_rms / 1.06 - 1))

    current_scale = max(0, min(31, current_scale))
    hold_scale = max(0, int(current_scale * 0.33 + 0.5))
    logging.debug("converted current %f to scale %d run, %d hold", i_rms, current_scale, hold_scale)

    self.ihold_irun.data.bits.irun = 1    #current_scale
    self.ihold_irun.data.bits.ihold = 1    #hold_scale
    self._write_register(self.ihold_irun)

  def set_disabled(self, force_update=False):
    if self.current_enabled is False:
      return

    # preserve the register value for when we re-enable
    original_value = self.chopconf.data.register

    self.chopconf.data.bits.toff = 0
    self._write_register(self.chopconf)
    self.current_enabled = False

    self.chopconf.data.register = original_value

  def set_enabled(self, force_update=False):
    if self.current_enabled is True:
      return

    self._write_register(self.chopconf)
    self.current_enabled = True

  def set_decay(self, value):
    self.decay = value
    # TODO stub

  def reset(self):
    self.set_disabled()
    self.set_enabled()

  def set_current_disabled(self):
    self.set_disabled()

  def set_current_enabled(self):
    self.set_enabled()


if __name__ == "__main__":
  import serial
  uart = serial.Serial('/dev/ttyS2', 115200, timeout=0.1)
  banks = StepperBankUart([uart], 1)
  banks.add_stepper(0, )
  print banks.read_single_register(0, 6)

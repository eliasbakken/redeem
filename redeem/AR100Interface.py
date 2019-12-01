"""
AR100 interface file

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

import os
import logging

import struct
import mmap
import time

SRAM_START = 0x00040000
SRAM_END = 0x00053fff
SRAM_LEN = SRAM_END - SRAM_START
FIRMWARE_BASE = 0x00010000
FIRMWARE_MAX_SIZE = 0x00010000
PRU_ICSS_LEN = 4 * 1024
RING_BUFFER_BASE = 0x13000
RING_BUFFER_START = 0x00040000 + 0x13000
RING_BUFFER_END = 0x00040000 + 0x130fff
RING_BUFFER_LENGTH = RING_BUFFER_END - RING_BUFFER_START

CPU_CFG_REG = 0x01f01c00


class AR100Interface:
  def __init__(self):
    with open("/dev/mem", "r+b") as f:
      self.sram = mmap.mmap(f.fileno(), SRAM_LEN, offset=SRAM_START)
      self.cpucfg = mmap.mmap(f.fileno(), mmap.PAGESIZE, offset=self.page_base(CPU_CFG_REG))
      self.ring_buffer = mmap.mmap(f.fileno(), RING_BUFFER_LENGTH, offset=RING_BUFFER_START)

  def write_firmware(self, data):
    print hex(len(data))
    if len(data) > FIRMWARE_MAX_SIZE:
      print "Firmware is too big"
      return
    self.sram.seek(FIRMWARE_BASE)
    for byte in data:
      self.sram.write(byte)

  def assert_reset(self):
    self._set_cpu_cfg_reg(self._get_cpu_cfg_reg() & ~0x1)

  def deassert_reset(self):
    self._set_cpu_cfg_reg(self._get_cpu_cfg_reg() | 0x1)

  def _get_cpu_cfg_reg(self):
    offset = self.page_offset(CPU_CFG_REG)
    return struct.unpack('I', self.cpucfg[offset:offset + 4])[0]

  def _set_cpu_cfg_reg(self, value):
    offset = self.page_offset(CPU_CFG_REG)
    self.cpucfg[offset:offset + 4] = struct.pack('I', value)

  def page_base(self, addr):
    return addr & ~(mmap.PAGESIZE - 1)

  def page_offset(self, addr):
    return addr & (mmap.PAGESIZE - 1)

  def write_pattern(self):
    self.ring_buffer.seek(0)
    for i in xrange(200):
      self.ring_buffer.write(struct.pack('I', i % 2))
      self.ring_buffer.write(struct.pack('I', i * 10))
    for i in xrange(100):
      self.ring_buffer.write(struct.pack('I', i % 2))
      self.ring_buffer.write(struct.pack('I', 1000 - (i * 10)))

  @staticmethod
  def get_shared_long(offset):
      print "get_shared_long"
    lon = [-1]
    with open("/dev/mem", "r+b") as f:
      sram_mem = mmap.mmap(f.fileno(), PRU_ICSS_LEN, offset=SRAM_A2)
      lon = struct.unpack('L', ddr_mem[RING_BUFFER_START + offset:RING_BUFFER_START + offset + 4])
    return lon[0]

  @staticmethod
  def set_shared_long(offset, L):
    print "set_shared_long"
    with open("/dev/mem", "r+b") as f:
      ddr_mem = mmap.mmap(f.fileno(), PRU_ICSS_LEN, offset=SRAM_A2)
      lon = struct.pack('L', L)
      ddr_mem[SHARED_RAM_START + offset:SHARED_RAM_START + offset + 4] = lon
    return

  @staticmethod
  def set_active_endstops(L):
    PruInterface.set_shared_long(8, L)
    return

  @staticmethod
  def get_steps_remaining():
    return PruInterface.get_shared_long(16)

  @staticmethod
  def get_endstop_triggered():
    return PruInterface.get_shared_long(20)


if __name__ == "__main__":
  ar100 = AR100Interface()
  firmware = "/root/scp.bin"
  print "writing " + firmware
  with open(firmware, "rb") as f:
    #ar100.assert_reset()
    #ar100.write_firmware(f.read())
    #ar100.deassert_reset()
    ar100.write_pattern()
    #print "Firmware written"

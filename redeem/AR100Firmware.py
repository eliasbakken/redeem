"""
AR100Firmware.py file for Recore.

Handles the compilation of the PRU firmware based on the different printer settings on the fly.

Author: Mathieu Monney
email: zittix(at)xwaves(dot)net
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
import subprocess
import shutil
import re
from six import iteritems

from Printer import Printer


class AR100Firmware:
  def __init__(self, firmware_source_file0, binary_filename0, printer):
    """
    Create and initialize a AR100Firmware

    Parameters
    ----------

    firmware_source_file : string
        The path to the firmware source to use to produce the firmware
    binary_filename : string
        Full path to the file where to store the final firmware file
        without the extension (without .bin)
    """

    self.firmware_source_file0 = os.path.realpath(firmware_source_file0)
    self.binary_filename0 = os.path.realpath(binary_filename0)
    self.config = printer.config
    self.printer = printer

    #Remove the bin extension of the firmware output filename
    if os.path.splitext(self.binary_filename0)[1] != '.bin':
      logging.error('Invalid binary output filename on file 0. '
                    'It should have the .bin extension.')
      raise RuntimeError('Invalid binary output filename on file 0. '
                         'It should have the .bin extension.')

  def is_needing_firmware_compilation(self):
    """ Returns True if the firmware needs recompilation """
    return False

  def produce_firmware(self, unconditionally=False):
    if not unconditionally:
      if not self.is_needing_firmware_compilation():
        return True
    return True

  def build_firmware(self, input, output):
    return True

  def get_firmware(self, prunum=0):
    """ Return the path to the firmware bin file, None if the firmware
        cannot be produced. """
    if self.is_needing_firmware_compilation():
      if not self.produce_firmware():
        return None

    return self.binary_filename0

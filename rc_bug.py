#!/usr/bin/python3

import sys,os, time
from smbus2 import SMBus, i2c_msg

set_command_mode = 0x01
set_reset_timeout = 0x8c
set_rc_input_scaling_degree = 0x20
set_rc_invert_input_direction = 0x21
set_rc_input_minimum = 0x22
set_rc_neutral_minimum = 0x24
set_rc_neutral_maximum = 0x26
set_rc_input_maximum = 0x28
set_rc_target_minimum = 0x2A
set_rc_target_maximum = 0x32

class TicI2C(object):
  def __init__(self, bus, address):
    self.bus = bus
    self.address = address
 
  def command(self, code):
     write = i2c_msg.write(self.address, [code])
     self.bus.i2c_rdwr(write)
 
  def report_status(self, errors, msg=""):
      print(msg)
      if errors & 0x01: print(">>Intentionally de-energized")
      if errors & 0x02: print(">>Motor driver error")
      if errors & 0x04: print(">>Low VIN")
      if errors & 0x08: print(">>Kill switch active")
      if errors & 0x10: print(">>Required input invalid")
      if errors & 0x20: print(">>Serial error")
      if errors & 0x40: print(">>Command timeout")
      if errors & 0x80: print(">>Safe start violation")
      if errors & 0x100: print(">>ERR line high")

  def errors_occurred(self, report=True, msg=None):
    if msg is not None:
        print(msg)
    write = i2c_msg.write(self.address, [0xA2, 0x04])
    read = i2c_msg.read(self.address, 4)
    self.bus.i2c_rdwr(write, read)
    dl = list(read)
    errors = dl[0] + (dl[1] << 8) + (dl[2] << 16) + (dl[3] << 24)
    if report:
        self.report_status(errors, "Errors occurred:")

  def set8(self, offset, data):
    write = i2c_msg.write(self.address, [offset, data])
    self.bus.i2c_rdwr(write)

  def set32(self, offset, data):
    command = [offset,
      data >> 0 & 0xFF,
      data >> 8 & 0xFF,
      data >> 16 & 0xFF,
      data >> 24 & 0xFF]
    write = i2c_msg.write(self.address, command)
    self.bus.i2c_rdwr(write)
     
i2c = SMBus(1)
tic = TicI2C(i2c, 14)

while 1:
    tic.set8(set_command_mode, 2)

tic.set8(set_command_mode, 2)
tic.set8(set_rc_input_scaling_degree, 1)
tic.set8(set_rc_invert_input_direction, 1)
tic.set32(set_rc_input_minimum, 1393)
tic.set32(set_rc_input_maximum, 3102)
tic.set32(set_rc_neutral_minimum, 2330)
tic.set32(set_rc_neutral_maximum, 2405)
tic.set32(set_rc_target_minimum, -2000) 
tic.set32(set_rc_target_maximum, 2000)
tic.command(set_reset_timeout)
tic.errors_occurred()


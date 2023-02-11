#!/usr/bin/python3
# Must issue exit-safe-start before first movement command after energizing motor

import sys,os,argparse, time
#import smbus
from smbus2 import SMBus, i2c_msg
from argparse import RawTextHelpFormatter
import RPi.GPIO as GPIO

op = argparse.ArgumentParser(allow_abbrev=False, formatter_class=RawTextHelpFormatter, description="""
Description of script goes here
""")

op.add_argument('--port',             help='2=T249, 3=36v4 (defaults to port 3)', type=int, default=3)
op.add_argument('--set',              help='Set offset,data')
op.add_argument('--clr',              help='Clear errors occurred, reset timeout', action='store_true')
op.add_argument('--stop',             help='Stop and set position', type=int)
op.add_argument('--ess',              help='Exit safe start', action='store_true')
op.add_argument('--config',           help='Set standard configuration', action='store_true')
op.add_argument('--mqtt',             help='Setup MQTT control loop',action='store_true')
op.add_argument('--off',              help='De-energize motor',action='store_true')
op.add_argument('--on',               help='Energize motor',action='store_true')
op.add_argument('--settings',         help='Settings file')
op.add_argument('--nosettings', '-ns',help='Do not update from settings file', action='store_true')
op.add_argument('-s', '--status',     help='Display current status',action='store_true')
op.add_argument('--stat',             help='Display brief status',action='store_true')
op.add_argument('-sp', '--show_position',  help='Show current position',action='store_true')
op.add_argument('-p', '--position',   help='Position', type=int)
op.add_argument('--step',             help='Step given amount', type=int)
op.add_argument('--lrc',              help='Left-right-count, given as l,r,c')
op.add_argument('--step_mode', '-sm', help='Set step mode - full,1,2,4,8,16,32 ')
op.add_argument('--set_position',     help='Set current position')
op.add_argument('--acc',              help='Set acceleration')
op.add_argument('--speed',            help='Set max speed')
op.add_argument('--reset',            help='Reset TIC', action='store_true')
op.add_argument('--test',             help='Run test routine', action='store_true')
op.add_argument('--test_gpio',        help='Run gpio test loop', action='store_true')
op.add_argument('-X',                 help='Show commands, but do not execute',action='store_true')
op.add_argument('cmdline',            help='Positional arguments',nargs='*')
opts = op.parse_args(sys.argv[1:])

get_target_position = 0x0A
get_target_velocity = 0x0E
get_max_speed = 0x16
get_current_velocity = 0x26

set_target_velocity = 0xE3
set_halt_and_set = 0xEC
set_current_limit = 0x91
set_max_speed = 0xE6
set_max_accel = 0xEA
set_max_decel = 0xE9
set_starting_speed = 0x43
set_step_mode = 0x94
set_reset_timeout = 0x8c
set_reset = 0xB0

class TicI2C(object):
  def __init__(self, bus, address):
    self.bus = bus
    self.address = address
 
  # Sends the "Exit safe start" command.
  def exit_safe_start(self):
    self.command(0x83)

  def command(self, code):
     write = i2c_msg.write(self.address, [code])
     self.bus.i2c_rdwr(write)
 
  def de_energize(self):
      self.command(0x86)
      print("Motor de-energized")
      GPIO.output(27, 0)

  def energize(self):
      GPIO.output(27, 1)
      self.command(0x85)
      print("Motor energized")

  # Sets the target position.
  # For more information about what this command does, see the
  # "Set target position" command in the "Command reference" section of the
  # Tic user's guide.
  def set_target_position(self, target, sleep=0.5):
    print("Set target position=", target)
    self.command(set_reset_timeout)
    command = [0xE0,
      target >> 0 & 0xFF,
      target >> 8 & 0xFF,
      target >> 16 & 0xFF,
      target >> 24 & 0xFF]
    write = i2c_msg.write(self.address, command)
    self.bus.i2c_rdwr(write)
    self.wait_for_motor(sleep)
 
  # Gets one or more variables from the Tic.
  def get_variables(self, offset, length):
    write = i2c_msg.write(self.address, [0xA1, offset])
    read = i2c_msg.read(self.address, length)
    self.bus.i2c_rdwr(write, read)
    return list(read)

  def report_status(self, errors, msg):
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

  def errors_occurred(self, report=True):
    write = i2c_msg.write(self.address, [0xA2, 0x04])
    read = i2c_msg.read(self.address, 4)
    self.bus.i2c_rdwr(write, read)
    dl = list(read)
    errors = dl[0] + (dl[1] << 8) + (dl[2] << 16) + (dl[3] << 24)
    if report:
        self.report_status(errors, "Errors occurred:")

  def get8(self, offset):
      dl = self.get_variables(offset, 2)
      return dl[0]

  def get16(self, offset):
      dl = self.get_variables(offset, 2)
      data = dl[0] + (dl[1] << 8)
      return data

  def get32(self, offset):
      dl = self.get_variables(offset, 4)
      data = dl[0] + (dl[1] << 8) + (dl[2] << 16) + (dl[3] << 24)
      return data

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
     
  # Gets the "Current position" variable from the Tic.
  def get_current_position(self):
    b = self.get_variables(0x22, 4)
    position = b[0] + (b[1] << 8) + (b[2] << 16) + (b[3] << 24)
    #if position >= (1 << 31):
    #  position -= (1 << 32)
    return position

  def step(self, count):
    #self.command(set_reset_timeout)
    position = self.get_current_position()
    target = position + count
    print("Stepping from %0d to %0d" % (position, target))
    self.set_target_position(target)

  def wait(self, seconds, count):
    for i in range(int(count)):
        self.command(set_reset_timeout)
        time.sleep(seconds)
    self.command(set_reset_timeout)
    time.sleep(0.1)

  def wait_for_motor(self, sleep=0.5):
    done = False
    print("Wait for motor", end='')
    while not done:
        time.sleep(sleep)
        velocity = self.get32(get_current_velocity)
        done = True if velocity == 0 else False
        print(".", end='')
        sys.stdout.flush()
        #print(f"Current velocity: >{velocity}<")
        self.command(set_reset_timeout)
    print("done")

  def lrc(self, left_step, right_step, count):
    for i in range(int(count)):
        print('Exeuting lrc: left=%s right=%s i=%d of %s' % (left_step, right_step, i, count))
        self.step(left_step)
        self.wait(0.5,2)
        self.step(right_step)
        self.wait(0.5,2)

dev_addr = 14
i2c = SMBus(1 if opts.port == 3 else 4)
tic = TicI2C(i2c, dev_addr)

U1_ERR_PIN = 14
U1_RST_PIN = 25
U2_ERR_PIN = 19
U2_RST_PIN = 16
SC_EN_PIN = 27
TC_EN_PIN = 4
XD_SDA_PIN = 12 # External display
XD_SCL_PIN = 13 # External display
GREEN_LED_PIN = 22
RED_LED_PIN = 10

# Configure GPIO pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(U1_ERR_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(U1_RST_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(U2_ERR_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(U2_RST_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(SC_EN_PIN, GPIO.OUT)
GPIO.setup(TC_EN_PIN, GPIO.OUT)
GPIO.setup(XD_SDA_PIN, GPIO.OUT)
GPIO.setup(XD_SCL_PIN, GPIO.OUT)
GPIO.setup(GREEN_LED_PIN, GPIO.OUT)
GPIO.setup(RED_LED_PIN, GPIO.OUT)

GPIO.output(SC_EN_PIN, 0)
GPIO.output(TC_EN_PIN, 0)
GPIO.output(GREEN_LED_PIN, 0)
GPIO.output(RED_LED_PIN, 0)

if opts.test_gpio:
    print("Running GPIO test loop...")
    count = 10
    led = 0
    for i in range(1,1000):
        GPIO.output(SC_EN_PIN, GPIO.HIGH)
        GPIO.output(XD_SCL_PIN, GPIO.HIGH)
        time.sleep(0.1)
        GPIO.output(SC_EN_PIN, GPIO.LOW)
        GPIO.output(XD_SCL_PIN, GPIO.LOW)
        time.sleep(0.1)
        count -= 1
        if count == 0:
            led = 1 if led==0 else 0
            GPIO.output(GREEN_LED_PIN, led)
            count = 10

    GPIO.cleanup()
    print("GPIO test loop complete")
    sys.exit()

if opts.reset:
  tic.command(set_reset)

if opts.status:
  tic.report_status(tic.get32(0x02), "Error status:")
  tic.errors_occurred()
  sys.exit()

position = tic.get_current_position()
print("Planning mode=", tic.get8(0x09))
print("Current position is %d" % position)
print("Current VIN=", tic.get16(0x33))
print("Current up time=", tic.get32(0x35))
print("Target position=", tic.get32(get_target_position))
print("Target velocity=", tic.get32(0x0E))
print("Max accel=", tic.get32(0x1E))
print("Max decel=", tic.get32(0x1A))
print("Max speed=", tic.get32(0x16))
print("Starting speed=", tic.get32(0x12))
print("Current limit=", tic.get8(0x4A))
print("Step mode=", tic.get8(0x49))
print("--------------")
print("Op state=", tic.get8(0x00))
print("Errors occurred=", tic.get32(0x04))

if opts.config:
    tic.set32(set_target_velocity, 0)
    tic.set32(set_max_speed, 8000000)
    tic.set32(set_max_accel, 20000)
    tic.set32(set_max_decel, 0) # 0->matches acceleration
    tic.set32(set_starting_speed, 1000)
    tic.set8(set_step_mode, 0)
    tic.set8(set_current_limit, 30)
    tic.command(set_reset_timeout)

"""
speed, accel, starting speed, current
249: dramatically better than 36v4
good 1M, 8k, 50k, 30
good 1M, 10k, 50k, 30
bad  1M, 10k, 50k, 18
bad  2M,  5k,  5k, 30 - stumbles as it accelerates, slower accel doesn't help
good 2M,  5k,  5k, 40 - ok at 600 steps, falters at 1000
good 2M,  5k,  5k, 60 - signifantly reduced falter at 1000 steps
36v4:
bad  1M, 10k, 50k, 8
bad  1M, 5k, 50k, 16
bad  1M, 5k, 10k, 8
bad  1M, 100k, 5k, 8 - somewhat ok, but doesn't always change direction
bad  1M, 100k, 5k, 6 - much worse
ok   1M, 100k, 5k, 9 - still sometimes doesn't change direction (sdcd)
bad  1M, 100k, 5k, 10 - doesn't change direction 
bad  1M, 10k, 5k, 9 - stumbles start badly
ok   1M, 500k, 5k, 9
ok   1M, 200k, 5k, 9 - sdcd
ok   1M, 500k, 5k, 9 - sdcd, sometimes stumbles start (sss)
bad  1M, 500k, 5k, 10 - worse
crap 1M, 500k, 5k, 12 - much worse
ok   1M, 100k, 5k, 9  - best, accel +/- seems to make it worse, faster speed is worse
bad  1M, 500k, 5k, 9  - sdcd
ok   1M, 100k, 5k, 9
ok   1M,  50k, 5k, 9
crap 2M,  50k, 5k, 9 - more current doesn't help


torque is much weaker on 36v4
starting speed doesn't seem to make much difference
"""
if opts.off:
    tic.de_energize()

if opts.on:
    tic.energize()
    tic.errors_occurred()
    tic.exit_safe_start()

if opts.clr:
    tic.command(set_reset_timeout)
    tic.errors_occurred()

if opts.ess:
    tic.exit_safe_start()
    tic.errors_occurred()

if opts.lrc:
    tic.errors_occurred(report=False)
    tic.exit_safe_start()
    data = opts.lrc.split(',')
    tic.lrc(int(data[0]), int(data[1]), int(data[2]))

if opts.position is not None:
    tic.set_target_position(opts.position)

if opts.step is not None:
    tic.errors_occurred(report=False)
    tic.exit_safe_start()
    tic.step(opts.step)

if opts.stop is not None:
    tic.set32(set_halt_and_set, opts.stop)

if opts.test:
    tic.errors_occurred()
    for i in range(3):
        tic.step(100);

position = tic.get_current_position()
print("Current position is %d" % position)

tic.errors_occurred()


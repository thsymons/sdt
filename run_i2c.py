#!/usr/bin/python3
# Must issue exit-safe-start before first movement command after energizing motor

import sys,os,argparse, time
#import smbus
from smbus2 import SMBus, i2c_msg
from argparse import RawTextHelpFormatter
import RPi.GPIO as GPIO
from subprocess import call
import curses

op = argparse.ArgumentParser(allow_abbrev=False, formatter_class=RawTextHelpFormatter, description="""
Description of script goes here
""")

op.add_argument('--port',             help='Select motor - t or s, defaults to t', default='t', choices=['t', 's'])
op.add_argument('--set',              help='Set offset,data')
op.add_argument('--clr',              help='Clear errors occurred, reset timeout', action='store_true')
op.add_argument('--stop',             help='Stop and set position', type=int)
op.add_argument('--ess',              help='Exit safe start', action='store_true')
op.add_argument('--config',           help='Set standard configuration', action='store_true')
op.add_argument('--rc',               help='Enable RC control mode', action='store_true')
op.add_argument('--gorc',             help='Enable full RC tractor control', action='store_true')
op.add_argument('--setup_rc',         help='Setup full RC tractor control', action='store_true')
op.add_argument('--alloff',           help='De-energize both ports', action='store_true')
op.add_argument('--getch',            help='Enable getch mode', action='store_true')
op.add_argument('--bench',            help='Enable bench motor control', action='store_true')
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
op.add_argument('--ssa',              help='Set step, speed, acceleration')
op.add_argument('--reset',            help='Reset TIC', action='store_true')
op.add_argument('--test',             help='Run test routine', action='store_true')
op.add_argument('--test_gpio',        help='Run gpio test loop', action='store_true')
op.add_argument('--joy_test',         help='Test joystick, with motors', action='store_true')
op.add_argument('--joy_testx',        help='Test joystick, without motors', action='store_true')
op.add_argument('--i2c_loop',         help='Infinite write loop to I2C', action='store_true')
op.add_argument('--shutdown',         help='Wait for shutdown button', action='store_true')
op.add_argument('-X',                 help='Show commands, but do not execute',action='store_true')
op.add_argument('cmdline',            help='Positional arguments',nargs='*')
opts = op.parse_args(sys.argv[1:])

get_target_position = 0x0A
get_target_velocity = 0x0E
get_max_speed = 0x16
get_current_velocity = 0x26

set_command_mode = 0x01
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
set_rc_input_scaling_degree = 0x20
set_rc_invert_input_direction = 0x21
set_rc_input_minimum = 0x22
set_rc_neutral_minimum = 0x24
set_rc_neutral_maximum = 0x26
set_rc_input_maximum = 0x28
set_rc_target_minimum = 0x2A
set_rc_target_maximum = 0x32
rc_pulse_width = 0x3D
get_max_speed = 0x16
get_max_accel = 0x1E
get_max_decel = 0x1A
get_starting_speed = 0x12

SC_PORT = 3 # steering control
TC_PORT = 2 # throttle control

win = None
last_line = 4
def show_msg(msg, line=0, clr=False):
  global win
  if opts.getch:
    win.addstr(line+3,0,"")
    if clr:
      win.clrtobot()
    else:
      win.clrtoeol()
    win.addstr(line+3,0,msg)
  else:
    print(msg)

# Note: I2C is disabled
# To enable:  sudo raspi-config nonint do_i2c 0
# or set to 1 to disable, yes, 1=disable, 0=enable
# sudo vim /boot/config.txt
# i2cdetect -y 1/4

class TicI2C(object):

  def __init__(self, bus, address, en_pin):
    self.bus = bus
    self.address = address
    self.en_pin = en_pin
 
  def test(self, msg=""):
      print('Tic %s en_pin=%0d' % (msg, self.en_pin))

  # Sends the "Exit safe start" command.
  def exit_safe_start(self):
    self.command(0x83)

  def command(self, code):
     write = i2c_msg.write(self.address, [code])
     self.bus.i2c_rdwr(write)
 
  def de_energize(self):
      self.command(0x86)
      print("Motor de-energized")
      GPIO.output(self.en_pin, 1)

  def energize(self):
      GPIO.output(self.en_pin, 0)
      self.command(0x85)
      show_msg("Motor energized")

  # Sets the target position.
  # For more information about what this command does, see the
  # "Set target position" command in the "Command reference" section of the
  # Tic user's guide.
  def set_target_position(self, target, sleep=0.5):
    #print("Set target position=", target)
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
    command = [offset, data]
    write = i2c_msg.write(self.address, command)
    self.bus.i2c_rdwr(write)
    time.sleep(0.1)

  def set16(self, offset, data):
    command = [offset,
      data >> 0 & 0xFF,
      data >> 8 & 0xFF]
    write = i2c_msg.write(self.address, command)
    self.bus.i2c_rdwr(write)
    time.sleep(0.1)

  def set32(self, offset, data):
    command = [offset,
      data >> 0 & 0xFF,
      data >> 8 & 0xFF,
      data >> 16 & 0xFF,
      data >> 24 & 0xFF]
    write = i2c_msg.write(self.address, command)
    self.bus.i2c_rdwr(write)
    time.sleep(0.1)

#  def set32(self, offset, data):
#    command = [offset,
#      data >> 0 & 0xFF,
#      data >> 8 & 0xFF,
#      data >> 16 & 0xFF,
#      data >> 24 & 0xFF]
#    write = i2c_msg.write(self.address, command)
#    self.bus.i2c_rdwr(write)
#    rdata = self.get32(offset)
     
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
    show_msg("Stepping from %0d to %0d" % (position, target), line=2)
    self.set_target_position(target)

  def go_step(self, steps):
    global last_line
    self.errors_occurred(report=False)
    self.exit_safe_start()
    position = self.get_current_position()
    target = position + steps
    if last_line > 12:
      last_line = 4
      show_msg("", last_line, clr=True)
    show_msg("Stepping from %0d to %0d" % (position, target), line=last_line)
    last_line += 1
    self.set_target_position(target)

  def wait(self, seconds, count):
    for i in range(int(count)):
        self.command(set_reset_timeout)
        time.sleep(seconds)
    self.command(set_reset_timeout)
    time.sleep(0.1)

  def wait_for_motor(self, sleep=0.5):
    done = False
    #show_msg("Wait for motor...")
    while not done:
        time.sleep(sleep)
        velocity = self.get32(get_current_velocity)
        done = True if velocity == 0 else False
        #print(".", end='')
        sys.stdout.flush()
        show_msg("velocity=%8d position=%10d" % (velocity, self.get_current_position()), line=1)
        self.command(set_reset_timeout)
    #show_msg("Wait for motor...done")

  def wait_forever(self, sleep=0.5):
    done = False
    print("Wait forever", end='')
    while not done:
        time.sleep(sleep)
        print(".", end='')
        sys.stdout.flush()
        #print(f"Current velocity: >{velocity}<")
        self.command(set_reset_timeout)

  def lrc(self, left_step, right_step, count):
    for i in range(int(count)):
        print('Exeuting lrc: left=%s right=%s i=%d of %s' % (left_step, right_step, i, count))
        self.step(left_step)
        self.wait(0.5,2)
        self.step(right_step)
        self.wait(0.5,2)

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
GO_PIN = 9
STOP_PIN = 11
# Joystick control pins
FWD_PIN = 0
BACK_PIN = 1
LEFT_PIN = 9
RIGHT_PIN = 11

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

tic = None
dev_addr = 14
def setup_port(port):
  global tic
  if port == 's':
      i2c = SMBus(1)
      en_pin = TC_EN_PIN
  else:
      i2c = SMBus(4)
      en_pin = SC_EN_PIN
  tic = TicI2C(i2c, dev_addr, en_pin)

# Configure GPIO pins
GPIO.setup(U1_ERR_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(U1_RST_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(U2_ERR_PIN, GPIO.OUT)
GPIO.setup(U2_RST_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(SC_EN_PIN, GPIO.OUT)
GPIO.setup(TC_EN_PIN, GPIO.OUT)
GPIO.setup(XD_SDA_PIN, GPIO.OUT)
GPIO.setup(XD_SCL_PIN, GPIO.OUT)
GPIO.setup(GREEN_LED_PIN, GPIO.OUT)
GPIO.setup(RED_LED_PIN, GPIO.OUT)
GPIO.setup(FWD_PIN, GPIO.IN)
GPIO.setup(BACK_PIN, GPIO.IN)
GPIO.setup(LEFT_PIN, GPIO.IN)
GPIO.setup(RIGHT_PIN, GPIO.IN)

GPIO.output(SC_EN_PIN, 0)
GPIO.output(TC_EN_PIN, 0)
GPIO.output(GREEN_LED_PIN, 0)
GPIO.output(RED_LED_PIN, 0)
GPIO.output(U2_ERR_PIN, 0)


if opts.test_gpio:
    print("Running GPIO test loop...")
    count = 10
    led = 0
    while 1:
        #GPIO.output(SC_EN_PIN, GPIO.HIGH)
        GPIO.output(XD_SCL_PIN, GPIO.HIGH)
        #time.sleep(0.1)
        #GPIO.output(SC_EN_PIN, GPIO.LOW)
        GPIO.output(XD_SCL_PIN, GPIO.LOW)
        #time.sleep(0.1)
        continue
        count -= 1
        if count == 0:
            led = 1 if led==0 else 0
            GPIO.output(GREEN_LED_PIN, led)
            count = 100

    GPIO.cleanup()
    print("GPIO test loop complete")
    sys.exit()

if opts.joy_testx:
    print("Test joystick inputs...")
    while True:
      if GPIO.input(FWD_PIN) == 0:
        print("Joystick moved: FWD")
      elif GPIO.input(BACK_PIN) == 0:
        print("Joystick moved: BACK")
      elif GPIO.input(LEFT_PIN) == 0:
        print("Joystick moved: LEFT")
      elif GPIO.input(RIGHT_PIN) == 0:
        print("Joystick moved: RIGHT")
      time.sleep(0.5)

def enable_port(tic):
  tic.energize()
  tic.errors_occurred()
  tic.exit_safe_start()

def motor_info(tic, msg):
  print(f'Motor info: {msg}')
  print('max_speed', tic.get32(get_max_speed))
  print('max_accel', tic.get32(get_max_accel))

def go_step(tic, steps):
  tic.errors_occurred(report=False)
  tic.exit_safe_start()
  tic.step(steps)

if opts.joy_test:
    print("Test joystick inputs...")
    setup_port(TC_PORT)
    throttle = tic
    throttle.set32(set_max_speed, 40000000)
    throttle.set32(set_max_accel, 200000)
    motor_info(throttle, "throttle")
    enable_port(throttle)
    time.sleep(0.5)
    setup_port(SC_PORT)
    steering = tic
    steering.set32(set_max_speed, 40000000)
    steering.set32(set_max_accel, 200000)
    motor_info(steering, "steering")
    enable_port(steering)
    throttle_steps = 1000
    steering_steps = 1000
    time.sleep(0.5)
    while True:
      if GPIO.input(FWD_PIN) == 0:
        go_step(throttle, throttle_steps)
      elif GPIO.input(BACK_PIN) == 0:
        go_step(throttle, -throttle_steps)
      elif GPIO.input(LEFT_PIN) == 0:
        go_step(steering, steering_steps)
      elif GPIO.input(RIGHT_PIN) == 0:
        go_step(steering, -steering_steps)
      time.sleep(0.5)

if opts.ssa is not None:
    step, speed, acc = opts.ssa.split(',')
    setup_port(opts.port)
    tic.errors_occurred(report=False)
    tic.exit_safe_start()
    tic.set32(set_starting_speed, 4000)
    tic.set32(set_max_speed, int(speed))
    tic.set32(set_max_accel, int(acc))
    time.sleep(0.5)
    tic.step(int(step))
    sys.exit()

if not (opts.gorc or opts.setup_rc or opts.rc):
  setup_port(opts.port)

if opts.i2c_loop:
    #GPIO.output(XD_SCL_PIN, GPIO.LOW)
    tic.set8(set_command_mode, 0x2)
    time.sleep(0.01)
    #GPIO.output(XD_SCL_PIN, GPIO.HIGH)
    #tic.errors_occurred()
    sys.exit()

if opts.shutdown:
    while True:
      i = GPIO.input(STOP_PIN)
      if i == 0:
         call("sudo nohup shutdown -h now", shell=True)

if opts.reset:
  tic.command(set_reset)

if opts.status:
  tic.report_status(tic.get32(0x02), "Error status:")
  tic.errors_occurred()
  sys.exit()

def setup_rc(port):
    """
    Seems can only set RC parameters in Tic GUI.
    But can still control other parameters here over i2c.
    Seems like can also not set command mode over i2c.
    """
    global tic
    tic.command(set_reset)
    tic.set8(set_command_mode, 2)
    print("mode=", tic.get8(set_command_mode))
    tic.set8(set_command_mode, 2)
    print("mode=", tic.get8(set_command_mode))
    tic.set32(set_halt_and_set, 100000) # set current position = 0
    tic.set8(set_rc_input_scaling_degree, 1)
    tic.set8(set_rc_invert_input_direction, 1)
    if port == TC_PORT: # throttle control
      tic.set32(set_starting_speed, 4000)
      tic.set32(set_max_speed, 100000000)
      tic.set32(set_max_accel, 20000)
      tic.set32(set_rc_input_minimum, 1004) # 16 ?
      tic.set32(set_rc_input_maximum, 1766) # 16 ?
      tic.set32(set_rc_neutral_minimum, 1265) # 16 ?
      tic.set32(set_rc_neutral_maximum, 1270) # 16 ?
      tic.set32(set_rc_target_minimum, -20000) 
      tic.set32(set_rc_target_maximum, 20000)
      print('Steering control settings')
      print('  target min=', tic.get32(set_rc_target_minimum))
      print('  target max=', tic.get32(set_rc_target_maximum))
    else: # steering control
      tic.set32(set_starting_speed, 4000)
      tic.set32(set_max_speed, 200000000)
      tic.set32(set_max_accel, 200000)
      tic.set32(set_rc_input_minimum, 916) # 16 ?
      tic.set32(set_rc_input_maximum, 2080) # 16 ?
      tic.set32(set_rc_neutral_minimum, 1575) # 16 ?
      tic.set32(set_rc_neutral_maximum, 1580) # 16 ?
      tic.set32(set_rc_target_minimum, -200000) 
      tic.set32(set_rc_target_maximum, 200000)
      print('Throttle control settings')
      print('  target min=', tic.get32(set_rc_target_minimum))
      print('  target max=', tic.get32(set_rc_target_maximum))
    time.sleep(1)

def report_rc(tic):
  print('max_speed', tic.get32(get_max_speed))
  print('max_accel', tic.get32(get_max_accel))
  print('input_min', tic.get32(set_rc_input_minimum))
  print('input_max', tic.get32(set_rc_input_maximum))
  print('neutral_min', tic.get32(set_rc_neutral_minimum))
  print('neutral_max', tic.get32(set_rc_neutral_maximum))
  print('target_min', tic.get32(set_rc_target_minimum))
  print('target_max', tic.get32(set_rc_target_maximum))
 
if opts.rc:
  setup_port(port)
  setup_rc(opts.port)
  enable_port(tic)
  print('RC mode setup for port=%s' % 'throttle' if opts.port==3 else 'steering')
  sys.exit()

if opts.setup_rc:
  setup_port(2)
  setup_rc(2) # throttle control
  setup_port(3)
  setup_rc(3) # steering control
  print('RC mode setup for steering and throttle control')

if opts.gorc:
  setup_port(TC_PORT)
  throttle = tic
  report_rc(throttle)
  setup_port(SC_PORT)
  steering = tic
  report_rc(steering)
  #steering.set8(set_command_mode, 0)
  steering.set32(set_max_speed, 200000000)
  steering.set32(set_max_accel, 200000)
  #steering.set32(set_rc_target_minimum, -4000) 
  #steering.set32(set_rc_target_maximum, 4000)
  steering.set8(set_command_mode, 2)
  enable_port(steering)
  enable_port(throttle)
#  while 1:
#      print('RC pulse width: steering: pos=%0d rc=%0d' % (steering.get_current_position(), int(steering.get16(rc_pulse_width)/12)), 
#                           ' throttle: pos=%0d rc=%0d' % (throttle.get_current_position(), int(throttle.get16(rc_pulse_width)/12)))
#      time.sleep(2)
  sys.exit()

def stat(tic):
  show_msg('**** Getch status ****', clr=True)
  show_msg('start_speed=%0d' % tic.get32(get_starting_speed), 1)
  show_msg('max_speed=%0d' % tic.get32(get_max_speed), 2)
  show_msg('max_accel=%0d' % tic.get32(get_max_accel), 3)
  show_msg('max_decel=%0d' % tic.get32(get_max_decel), 4)
  show_msg('position=%0d' % tic.get_current_position(), 5)

if opts.getch:
  setup_port(TC_PORT)
  throttle = tic
  report_rc(throttle)
  setup_port(SC_PORT)
  steering = tic
  steering.set8(set_command_mode, 0)
  throttle.set8(set_command_mode, 0)
  report_rc(steering)
  steering.set32(set_halt_and_set, 10000) # set current position = 0
  steering.set32(set_max_speed, 20000000)
  steering.set32(set_max_accel, 10000)
  steering.set32(set_max_decel, 100000) # 0->matches acceleration
  throttle.set32(set_halt_and_set, 40000) # set current position = 0
  factor = 1 if opts.bench else 10
  throttle.set32(set_starting_speed, 20000*factor)
  throttle.set32(set_max_speed, 8000000*factor)
  throttle.set32(set_max_accel, 20000*factor)
  throttle.set32(set_max_decel, 20000*factor) # 0->matches acceleration
  win = curses.initscr()
  steering.energize()
  throttle.energize()
  sc_step = 400
  tc_step = 500*factor
  step_inc = 100
  last = SC_PORT
  # hjkl for left, down, up, right
  while 1:
    win.addstr(0,0,"input:")
    ch = chr(win.getch())
    if ch == 'h':
      show_msg('steer left...', line=3)
      steering.go_step(-sc_step)
      last = SC_PORT
    elif ch == 'l':
      show_msg('steer right...', line=3)
      steering.go_step(sc_step)
      last = SC_PORT
    elif ch == 'k':
      show_msg('throttle up...', line=3)
      throttle.go_step(tc_step)
      last = TC_PORT
    elif ch == 'j':
      show_msg('throttle down...', line=3)
      throttle.go_step(-tc_step)
      last = TC_PORT
    elif ch.lower() == '+':
      if last == SC_PORT:
        sc_step += step_inc
        show_msg('SC == %0d' % sc_step, line=3)
      else:
        tc_step += step_inc
        show_msg('TC == %0d' % tc_step, line=3)
    elif ch.lower() == '-':
      if last == SC_PORT:
        show_msg('SC == %0d' % sc_step, line=3)
        sc_step -= step_inc
      else:
        tc_step -= step_inc
        show_msg('TC == %0d' % tc_step, line=3)
    elif ch == '?':
      tic = steering if last == SC_PORT else throttle
      stat(tic)
    elif ch.lower() == 'q':
      break
  win.endwin()
  sys.exit()


if opts.alloff:
  setup_rc(TC_PORT)
  tic.de_energize()
  setup_rc(SC_PORT)
  tic.de_energize()
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
    print('\nConfigure TIC')
    #tic.command(set_reset_timeout)
    #tic.exit_safe_start()
    #tic.set8(set_command_mode, 0)
    #print("mode=", tic.get8(set_command_mode))
    tic.set32(set_target_velocity, 0)
    tic.set32(set_max_speed, 200000000)
    tic.set32(set_max_accel, 200000)
    tic.set32(set_max_decel, 0) # 0->matches acceleration
    tic.set32(set_starting_speed, 4000)
    tic.set8(set_step_mode, 0)
    tic.set8(set_current_limit, 30)
    tic.command(set_reset_timeout)
    tic.errors_occurred()
    print("Configuration complete\n")

"""
speed, accel, starting speed, current, usteps
TC: 200M, 200K, 4K, 2A, 8 <- OK

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

#if opts.rc:
#    tic.wait_forever()

position = tic.get_current_position()
print("Current position is %d" % position)

tic.errors_occurred()


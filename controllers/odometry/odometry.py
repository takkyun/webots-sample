from numpy import pi
from controller import Robot
from libs.dotmap import dotmap
from libs.odometry_goto import OdometryGoto
from libs.odometry_track import OdometryTrack

ENCODER_UNIT = 159.23
SPEED_UNIT = 0.00628

class Controller(Robot):
  def __init__(self):
    super(Controller, self).__init__()
    self.timeStep = int(self.getBasicTimeStep())

    self.motor = dotmap({
      'left': self.getDevice('left wheel motor'),
      'right': self.getDevice('right wheel motor'),
    })
    self.motor.left.setPosition(float('inf'))
    self.motor.left.setVelocity(0.0)
    self.motor.right.setPosition(float('inf'))
    self.motor.right.setVelocity(0.0)

    self.position_sensor = dotmap({
      'left': self.getDevice('left wheel sensor'),
      'right': self.getDevice('right wheel sensor'),
    })
    self.position_sensor.left.enable(self.timeStep)
    self.position_sensor.right.enable(self.timeStep)

    self.speed = dotmap({
      'left': 0,
      'right': 0,
    })
    self.prev_speed = dotmap({
      'left': 0,
      'right': 0,
    })
    self.offset = dotmap({
      'left': 0.0,
      'right': 0.0,
    })

    # required to get the position sensor values
    self.step(self.timeStep)

    position = self.get_position()
    self.track = OdometryTrack(position.left, position.right)
    self.goto = OdometryGoto(self.track)
    self.track.result.x = 0
    self.track.result.y = 0
    self.track.result.theta = pi / 2

  def reset(self):
    self.speed.left = 0
    self.speed.right = 0
    self.prev_speed.left = 0
    self.prev_speed.right = 0
    self.motor.left.setVelocity(0.0)
    self.motor.right.setVelocity(0.0)

  def set_speed(self, left, right):
    self.prev_speed.left = self.speed.left
    self.prev_speed.right = self.speed.right
    self.speed.left = left
    self.speed.right = right
    if (self.speed.left != self.prev_speed.left or self.speed.right != self.prev_speed.right):
      self.motor.left.setVelocity(SPEED_UNIT * self.speed.left)
      self.motor.right.setVelocity(SPEED_UNIT * self.speed.right)

  def get_position(self):
    return dotmap({
      'left': ENCODER_UNIT * self.position_sensor.left.getValue(),
      'right': ENCODER_UNIT * self.position_sensor.right.getValue(),
    })

  def print_current_position(self):
    print('current position:', self.track.result.x, self.track.result.y, self.track.result.theta)

  def run_step(self):
    if self.step(self.timeStep) == -1:
      self.cleanup()
      exit()


  def goto_position(self, x, y, theta):
    print('goto', x, y, theta)
    self.goto.set_goal(x, y, theta)
    while self.goto.result.atgoal == 0:
      pos = self.get_position()
      self.track.step_pos(pos.left - self.offset.left, pos.right - self.offset.right)
      self.goto.step()
      self.set_speed(self.goto.result.speed_left, self.goto.result.speed_right)
      self.run_step()
    self.print_current_position()

  def run(self):
    self.reset()
    self.goto_position(0, 0.1, pi / 2)
    self.goto_position(0, 0, - pi / 2)
    self.reset()

controller = Controller()
controller.run()

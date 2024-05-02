##
## Copyright 1996-2023 Cyberbotics Ltd.
##
## Licensed under the Apache License, Version 2.0 (the "License");
## you may not use this file except in compliance with the License.
## You may obtain a copy of the License at
##
##   https://www.apache.org/licenses/LICENSE-2.0
##
## Unless required by applicable law or agreed to in writing, software
## distributed under the License is distributed on an "AS IS" BASIS,
## WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
## See the License for the specific language governing permissions and
## limitations under the License.
##
## (c) 2006 - 2008 EPFL, Lausanne, Switzerland
## Thomas Lochmatter
## Adapted by Nicolas Heiniger for the e-puck
##
## original code: https://github.com/cyberbotics/webots/tree/master/projects/samples/curriculum/lib
## ported by: (c) 2024 Takuya Otani
##

from numpy import pi, cos, sin

# calibration for an accurate odometry
INCREMENTS_PER_TOUR = 1000.0;   # from e-puck.org
AXIS_WHEEL_RATIO = 1.293;      # from e-puck.org
WHEEL_DIAMETER_LEFT = 0.0416;   # from e-puck.org
WHEEL_DIAMETER_RIGHT = 0.0404;  # from e-puck.org
SCALING_FACTOR = 0.976;         # default is 1

class OdometryConfiguration:
  wheel_distance: float
  wheel_conversion_left: float
  wheel_conversion_right: float
  def __init__(self):
    self.wheel_distance = AXIS_WHEEL_RATIO * SCALING_FACTOR *(WHEEL_DIAMETER_LEFT + WHEEL_DIAMETER_RIGHT) / 2.0
    self.wheel_conversion_left = WHEEL_DIAMETER_LEFT * SCALING_FACTOR * pi / INCREMENTS_PER_TOUR
    self.wheel_conversion_right = WHEEL_DIAMETER_RIGHT * SCALING_FACTOR * pi / INCREMENTS_PER_TOUR

class OdometryState:
  pos_left_prev: float
  pos_right_prev: float
  def __init__(self, pos_left_prev, pos_right_prev):
    self.pos_left_prev = pos_left_prev
    self.pos_right_prev = pos_right_prev

class OdometryResult:
  x: float
  y: float
  theta: float
  def __init__(self, x, y, theta):
    self.x = x
    self.y = y
    self.theta = theta

class OdometryTrack:
  configuration: OdometryConfiguration
  state: OdometryState
  result: OdometryResult

  def __init__(self, pos_left, pos_right):
    self.configuration = OdometryConfiguration()
    self.state = OdometryState(pos_left, pos_right)
    self.result = OdometryResult(0, 0, 0)

  def step_pos(self, pos_left, pos_right):
    delta_pos_left = pos_left - self.state.pos_left_prev
    delta_pos_right = pos_right - self.state.pos_right_prev
    delta_left = delta_pos_left * self.configuration.wheel_conversion_left
    delta_right = delta_pos_right * self.configuration.wheel_conversion_right
    delta_theta = (delta_right - delta_left) / self.configuration.wheel_distance
    theta2 = self.result.theta + delta_theta * 0.5
    delta_x = (delta_left + delta_right) * 0.5 * cos(theta2)
    delta_y = (delta_left + delta_right) * 0.5 * sin(theta2)

    self.result.x += delta_x
    self.result.y += delta_y
    self.result.theta += delta_theta
    if (self.result.theta > pi):
      self.result.theta -= 2 * pi
    elif (self.result.theta < -pi):
      self.result.theta += 2 * pi
    self.state.pos_left_prev = pos_left
    self.state.pos_right_prev = pos_right

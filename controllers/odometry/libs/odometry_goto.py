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
##
## original code: https://github.com/cyberbotics/webots/tree/master/projects/samples/curriculum/lib
## ported by: (c) 2024 Takuya Otani
##

from numpy import pi, sqrt, arctan2
from .odometry_track import OdometryTrack

class OdometryGotoConfiguration:
  speed_min: float
  def __init__(self, speed_min):
    self.speed_min = speed_min

class OdometryGotoState:
  goal_x: float
  goal_y: float
  goal_theta: float
  def __init__(self, goal_x, goal_y, goal_theta):
    self.goal_x = goal_x
    self.goal_y = goal_y
    self.goal_theta = goal_theta

class OdometryGotoResult:
  speed_left: int
  speed_right: int
  atgoal: int
  def __init__(self, speed_left, speed_right, atgoal):
    self.speed_left = speed_left
    self.speed_right = speed_right
    self.atgoal = atgoal

class OdometryGoto:
  track: OdometryTrack
  configuration: OdometryGotoConfiguration
  state: OdometryGotoState
  result: OdometryGotoResult
  def __init__(self, track):
    self.track = track
    self.configuration = OdometryGotoConfiguration(1.0)
    self.state = OdometryGotoState(0.0, 0.0, 0.0)
    self.result = OdometryGotoResult(0, 0, 1)

  def set_goal(self, x, y, theta):
    self.state.goal_x = x
    self.state.goal_y = y
    self.state.goal_theta = theta
    self.result.atgoal = 0

  def step(self):
    # get current position
    current = self.track.result
    goal = self.state

    dx = goal.goal_x - current.x
    dy = goal.goal_y - current.y

    # controller parameters, an initial choice for the values is given but might be changed
    RHO = 1.2
    ALPHA = 1.5
    BETA = -0.35

    # "v_c" is the robot's velocity in its longitudinal direction
    # the values range from -1000 to +1000
    # which corresponds approx. to max. 130mm/s
    v_adapt = 1000 / 0.13  # conversion-factor for speed in [m/s] to e-Puck speed units

    # "omega_c" is the robot's rotational speed around the vertical axis
    # (positive for turns in counter-clockwise direction)
    # the value is defined to range from -2000 to 2000
    # representing turn rates of max. 270°/s
    omega_adapt = 2000 / (270 * pi / 180) # conversion-factor for turn rate in [rad/s] to e-Puck speed units

    # calculate current distance and angles to goal position
    rho_c = sqrt(dx * dx + dy * dy)

    alpha_c = arctan2(dy, dx) - current.theta
    while (alpha_c > pi):
      alpha_c -= 2 * pi
    while (alpha_c < -pi):
      alpha_c += 2 * pi

    beta_c = -current.theta - alpha_c
    while (beta_c > pi):
      beta_c -= 2 * pi
    while (beta_c < -pi):
      beta_c += 2 * pi

    # control law
    v_c = RHO * rho_c
    omega_c = ALPHA * alpha_c + BETA * beta_c

    # adapt SI values to e-Puck units
    v_e = v_c * v_adapt
    omega_e= omega_c * omega_adapt

    # record motor speed
    self.result.speed_left = int(v_e - omega_e / 2)
    self.result.speed_right = int(v_e + omega_e / 2)

    # check speed limits (for accuracy reasons)
    if abs(self.result.speed_left) < self.configuration.speed_min:
      self.result.speed_left = 0
    if abs(self.result.speed_right) < self.configuration.speed_min:
      self.result.speed_right = 0

    # check if goal is reached
    if (self.result.speed_left == 0 and self.result.speed_right == 0) or rho_c < 0.002:
      self.result.atgoal = 1

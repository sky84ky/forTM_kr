import os
import math
from common.realtime import sec_since_boot, DT_MDL
from selfdrive.swaglog import cloudlog
from selfdrive.controls.lib.lateral_mpc import libmpc_py
from selfdrive.controls.lib.drive_helpers import MPC_COST_LAT
from selfdrive.controls.lib.lane_planner import LanePlanner
from selfdrive.config import Conversions as CV
from common.params import Params
from common.numpy_fast import interp
import cereal.messaging as messaging
from cereal import log
import common.log as trace1
from selfdrive.kyd_conf import kyd_conf

LaneChangeState = log.PathPlan.LaneChangeState
LaneChangeDirection = log.PathPlan.LaneChangeDirection

LOG_MPC = os.environ.get('LOG_MPC', False)

LANE_CHANGE_SPEED_MIN = 60 * CV.KPH_TO_MS
LANE_CHANGE_TIME_MAX = 10.

DESIRES = {
  LaneChangeDirection.none: {
    LaneChangeState.off: log.PathPlan.Desire.none,
    LaneChangeState.preLaneChange: log.PathPlan.Desire.none,
    LaneChangeState.laneChangeStarting: log.PathPlan.Desire.none,
    LaneChangeState.laneChangeFinishing: log.PathPlan.Desire.none,
    LaneChangeState.laneChangeDone: log.PathPlan.Desire.none,
  },
  LaneChangeDirection.left: {
    LaneChangeState.off: log.PathPlan.Desire.none,
    LaneChangeState.preLaneChange: log.PathPlan.Desire.none,
    LaneChangeState.laneChangeStarting: log.PathPlan.Desire.laneChangeLeft,
    LaneChangeState.laneChangeFinishing: log.PathPlan.Desire.laneChangeLeft,
    LaneChangeState.laneChangeDone: log.PathPlan.Desire.laneChangeLeft,
  },
  LaneChangeDirection.right: {
    LaneChangeState.off: log.PathPlan.Desire.none,
    LaneChangeState.preLaneChange: log.PathPlan.Desire.none,
    LaneChangeState.laneChangeStarting: log.PathPlan.Desire.laneChangeRight,
    LaneChangeState.laneChangeFinishing: log.PathPlan.Desire.laneChangeRight,
    LaneChangeState.laneChangeDone: log.PathPlan.Desire.laneChangeRight,
  },
}

def calc_states_after_delay(states, v_ego, steer_angle, curvature_factor, steer_ratio, delay):
  states[0].x = v_ego * delay
  states[0].psi = v_ego * curvature_factor * math.radians(steer_angle) / steer_ratio * delay
  return states

class PathPlanner():
  def __init__(self, CP):
    self.LP = LanePlanner()

    self.last_cloudlog_t = 0
    self.steer_rate_cost = CP.steerRateCost
    self.setup_mpc()
    self.solution_invalid_cnt = 0


    self.params = Params()
    kyd = kyd_conf(CP)
    if kyd.conf['steerRatio'] == "-1":
      self.steerRatio = CP.steerRatio
    else:
      self.steerRatio = float(kyd.conf['steerRatio'])
      
    if kyd.conf['steerRateCost'] == "-1":
      self.steer_rate_cost = CP.steerRateCost
    else:
      self.steer_rate_cost = float(kyd.conf['steerRateCost'])

    self.kyd_steerRatio = None
    self.sRBP = [0., 0.]
    self.sRBoost = [0., 0.]


    # Lane change 
    self.lane_change_enabled = self.params.get('LaneChangeEnabled') == b'1'
    self.lane_change_auto_delay = self.params.get_OpkrAutoLanechangedelay()  #int( self.params.get('OpkrAutoLanechangedelay') )

    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.lane_change_run_timer = 0.0
    self.lane_change_wait_timer = 0.0
    self.lane_change_ll_prob = 1.0
    self.prev_one_blinker = False

    self.param_OpkrEnableLearner = int(self.params.get('OpkrEnableLearner'))

  def setup_mpc(self):
    self.libmpc = libmpc_py.libmpc
    self.libmpc.init(MPC_COST_LAT.PATH, MPC_COST_LAT.LANE, MPC_COST_LAT.HEADING, self.steer_rate_cost)

    self.mpc_solution = libmpc_py.ffi.new("log_t *")
    self.cur_state = libmpc_py.ffi.new("state_t *")
    self.cur_state[0].x = 0.0
    self.cur_state[0].y = 0.0
    self.cur_state[0].psi = 0.0
    self.cur_state[0].delta = 0.0

    self.angle_steers_des = 0.0
    self.angle_steers_des_mpc = 0.0
    self.angle_steers_des_prev = 0.0
    self.angle_steers_des_time = 0.0

  def update(self, sm, pm, CP, VM):

    v_ego = sm['carState'].vEgo
    angle_steers = sm['carState'].steeringAngle
    active = sm['controlsState'].active

    angle_offset = sm['liveParameters'].angleOffset


    if not self.param_OpkrEnableLearner:
      kyd = kyd_conf()
      #self.steer_rate_cost = float(kyd.conf['steerRateCost'])
      self.sRBP = kyd.conf['sR_BP']
      self.sRBoost = kyd.conf['sR_Boost']
      boost_rate = interp(abs(angle_steers), self.sRBP, self.sRBoost)
      self.kyd_steerRatio = self.steerRatio + boost_rate

      self.sR_Cost = [1.5,1.32,1.15,0.99,0.84,0.72,0.62,0.53,0.36,0.275,0.20,0.175,0.15,0.11,0.05]
      #self.sR_Cost = [1.0,0.90,0.81,0.73,0.66,0.60,0.54,0.48,0.36,0.275,0.20,0.175,0.15,0.11,0.05] 
      #self.sR_Cost = [0.75,0.67,0.60,0.54,0.48,0.425,0.37,0.32,0.24,0.19,0.15,0.14,0.13,0.11,0.05]
      #self.sR_Cost = [0.50,0.46,0.425,0.395,0.37,0.34,0.315,0.29,0.23,0.185,0.15,0.14,0.13,0.11,0.05]
      #steerRatio = 10.0
      #"sR_BP": [0.0,2.0,4.0,6.0,8.0,10.0,12.0,14.0,20.0,27.0,35.0,40.0,45.0,60.0,100],
      #"sR_Boost": [0.0,0.7,1.3,1.9,2.5,3.05,3.55,4.0,5.0,5.7,6.2,6.35,6.4,6.5,8.0],
      self.steer_rate_cost = interp(abs(angle_steers), self.sRBP, self.sR_Cost)


    # Run MPC
    self.angle_steers_des_prev = self.angle_steers_des_mpc

    # Update vehicle model
    x = max(sm['liveParameters'].stiffnessFactor, 0.1)
    sr = max(sm['liveParameters'].steerRatio, 0.1)
    #VM.update_params(0.8, CP.steerRatio) # 타이어 강성계수 고정,조향비율 고정 계산
    VM.update_params(0.8, sr)

    curvature_factor = VM.curvature_factor(v_ego)

    self.LP.parse_model(sm['model'])

    # Lane change logic
    one_blinker = sm['carState'].leftBlinker != sm['carState'].rightBlinker
    below_lane_change_speed = v_ego < LANE_CHANGE_SPEED_MIN

    if sm['carState'].leftBlinker:
      self.lane_change_direction = LaneChangeDirection.left
    elif sm['carState'].rightBlinker:
      self.lane_change_direction = LaneChangeDirection.right

    if (not active) or (self.lane_change_run_timer > LANE_CHANGE_TIME_MAX) or (not one_blinker) or (not self.lane_change_enabled):
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
    else:
      torque_applied = sm['carState'].steeringPressed and \
                       ((sm['carState'].steeringTorque > 0 and self.lane_change_direction == LaneChangeDirection.left) or
                        (sm['carState'].steeringTorque < 0 and self.lane_change_direction == LaneChangeDirection.right))

      blindspot_detected = ((sm['carState'].leftBlindspot and self.lane_change_direction == LaneChangeDirection.left) or
                            (sm['carState'].rightBlindspot and self.lane_change_direction == LaneChangeDirection.right))

      lane_change_prob = self.LP.l_lane_change_prob + self.LP.r_lane_change_prob

      # State transitions
      # off
      if self.lane_change_state == LaneChangeState.off and one_blinker and not self.prev_one_blinker and not below_lane_change_speed:
        self.lane_change_state = LaneChangeState.preLaneChange
        self.lane_change_ll_prob = 1.0
        self.lane_change_wait_timer = 0

      # pre
      elif self.lane_change_state == LaneChangeState.preLaneChange:
        self.lane_change_wait_timer += DT_MDL

        if not one_blinker or below_lane_change_speed:
          self.lane_change_state = LaneChangeState.off
        elif not blindspot_detected and (torque_applied or (self.lane_change_auto_delay and self.lane_change_wait_timer > self.lane_change_auto_delay)):
          self.lane_change_state = LaneChangeState.laneChangeStarting

      # starting
      elif self.lane_change_state == LaneChangeState.laneChangeStarting:
        # fade out over .5s
        self.lane_change_ll_prob = max(self.lane_change_ll_prob - 1.5*DT_MDL, 0.0)
        # 98% certainty
        if lane_change_prob < 0.02 and self.lane_change_ll_prob < 0.01:
          self.lane_change_state = LaneChangeState.laneChangeFinishing

      # finishing
      elif self.lane_change_state == LaneChangeState.laneChangeFinishing:
        # fade in laneline over 1s
        self.lane_change_ll_prob = min(self.lane_change_ll_prob + DT_MDL, 1.0)
        if one_blinker and self.lane_change_ll_prob > 0.99:
          self.lane_change_state = LaneChangeState.laneChangeDone

      # done
      elif self.lane_change_state == LaneChangeState.laneChangeDone:
        if not one_blinker:
          self.lane_change_state = LaneChangeState.off

    if self.lane_change_state in [LaneChangeState.off, LaneChangeState.preLaneChange]:
      self.lane_change_run_timer = 0.0
    else:
      self.lane_change_run_timer += DT_MDL

    self.prev_one_blinker = one_blinker

    desire = DESIRES[self.lane_change_direction][self.lane_change_state]

    # Turn off lanes during lane change
    if desire == log.PathPlan.Desire.laneChangeRight or desire == log.PathPlan.Desire.laneChangeLeft:
      self.LP.l_prob *= self.lane_change_ll_prob
      self.LP.r_prob *= self.lane_change_ll_prob
    self.LP.update_d_poly(v_ego)

    # account for actuation delay
    if self.param_OpkrEnableLearner:
      self.cur_state = calc_states_after_delay(self.cur_state, v_ego, angle_steers - angle_offset, curvature_factor, VM.sR, CP.steerActuatorDelay)
    else:
      self.cur_state = calc_states_after_delay(self.cur_state, v_ego, angle_steers - angle_offset, curvature_factor, self.kyd_steerRatio, CP.steerActuatorDelay)
    
    v_ego_mpc = max(v_ego, 5.0)  # avoid mpc roughness due to low speed
    self.libmpc.run_mpc(self.cur_state, self.mpc_solution,
                        list(self.LP.l_poly), list(self.LP.r_poly), list(self.LP.d_poly),
                        self.LP.l_prob, self.LP.r_prob, curvature_factor, v_ego_mpc, self.LP.lane_width)

    # reset to current steer angle if not active or overriding
    if active:
      delta_desired = self.mpc_solution[0].delta[1]
      if self.param_OpkrEnableLearner:
        rate_desired = math.degrees(self.mpc_solution[0].rate[0] * VM.sR)
      else:
        rate_desired = math.degrees(self.mpc_solution[0].rate[0] * self.kyd_steerRatio)
    else:
      if self.param_OpkrEnableLearner:
        delta_desired = math.radians(angle_steers - angle_offset) / VM.sR
      else:
        delta_desired = math.radians(angle_steers - angle_offset) / self.kyd_steerRatio
      rate_desired = 0.0

    self.cur_state[0].delta = delta_desired
    if self.param_OpkrEnableLearner:
      self.angle_steers_des_mpc = float(math.degrees(delta_desired * VM.sR) + angle_offset)
    else:
      self.angle_steers_des_mpc = float(math.degrees(delta_desired * self.kyd_steerRatio) + angle_offset)
    #  Check for infeasable MPC solution
    mpc_nans = any(math.isnan(x) for x in self.mpc_solution[0].delta)
    t = sec_since_boot()
    if mpc_nans:
      self.libmpc.init(MPC_COST_LAT.PATH, MPC_COST_LAT.LANE, MPC_COST_LAT.HEADING, self.steer_rate_cost)
      if self.param_OpkrEnableLearner:
        self.cur_state[0].delta = math.radians(angle_steers - angle_offset) / VM.sR
      else:
        self.cur_state[0].delta = math.radians(angle_steers - angle_offset) / self.kyd_steerRatio

      if t > self.last_cloudlog_t + 5.0:
        self.last_cloudlog_t = t
        cloudlog.warning("Lateral mpc - nan: True")

    if self.mpc_solution[0].cost > 20000. or mpc_nans:   # TODO: find a better way to detect when MPC did not converge
      self.solution_invalid_cnt += 1
    else:
      self.solution_invalid_cnt = 0
    plan_solution_valid = self.solution_invalid_cnt < 3

    plan_send = messaging.new_message('pathPlan')
    plan_send.valid = sm.all_alive_and_valid(service_list=['carState', 'controlsState', 'liveParameters', 'model'])
    plan_send.pathPlan.laneWidth = float(self.LP.lane_width)
    plan_send.pathPlan.dPoly = [float(x) for x in self.LP.d_poly]
    plan_send.pathPlan.lPoly = [float(x) for x in self.LP.l_poly]
    plan_send.pathPlan.lProb = float(self.LP.l_prob)
    plan_send.pathPlan.rPoly = [float(x) for x in self.LP.r_poly]
    plan_send.pathPlan.rProb = float(self.LP.r_prob)

    plan_send.pathPlan.angleSteers = float(self.angle_steers_des_mpc)
    plan_send.pathPlan.rateSteers = float(rate_desired)
    plan_send.pathPlan.angleOffset = float(sm['liveParameters'].angleOffset)
    plan_send.pathPlan.mpcSolutionValid = bool(plan_solution_valid)
    plan_send.pathPlan.paramsValid = bool(sm['liveParameters'].valid)

    plan_send.pathPlan.desire = desire
    plan_send.pathPlan.laneChangeState = self.lane_change_state
    plan_send.pathPlan.laneChangeDirection = self.lane_change_direction

    if not self.param_OpkrEnableLearner:
      plan_send.pathPlan.steerRatio = float(self.kyd_steerRatio)

    pm.send('pathPlan', plan_send)

    if LOG_MPC:
      dat = messaging.new_message('liveMpc')
      dat.liveMpc.x = list(self.mpc_solution[0].x)
      dat.liveMpc.y = list(self.mpc_solution[0].y)
      dat.liveMpc.psi = list(self.mpc_solution[0].psi)
      dat.liveMpc.delta = list(self.mpc_solution[0].delta)
      dat.liveMpc.cost = self.mpc_solution[0].cost
      pm.send('liveMpc', dat)

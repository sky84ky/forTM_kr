from cereal import car, log
from common.realtime import DT_CTRL
from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.car.hyundai.hyundaican import create_lkas11, create_clu11, create_mdps12, create_lfa_mfa
from selfdrive.car.hyundai.values import Buttons, SteerLimitParams, CAR, FEATURES
from opendbc.can.packer import CANPacker
from selfdrive.config import Conversions as CV
from common.numpy_fast import interp

# speed controller
from selfdrive.car.hyundai.spdcontroller  import SpdController
from selfdrive.car.hyundai.spdctrlSlow  import SpdctrlSlow
from selfdrive.car.hyundai.spdctrlNormal  import SpdctrlNormal
from selfdrive.car.hyundai.spdctrlFast  import SpdctrlFast

from common.params import Params
from selfdrive.kyd_conf import kyd_conf
import common.log as trace1
import common.CTime1000 as tm

VisualAlert = car.CarControl.HUDControl.VisualAlert
LaneChangeState = log.PathPlan.LaneChangeState

class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.apply_steer_last = 0
    self.car_fingerprint = CP.carFingerprint
    self.packer = CANPacker(dbc_name)
    self.steer_rate_limited = False
    self.resume_cnt = 0
    self.last_resume_frame = 0
    self.last_lead_distance = 0
    self.lanechange_manual_timer = 0
    self.emergency_manual_timer = 0
    self.driver_steering_torque_above_timer = 0
    self.mode_change_timer = 0
    self.turning_signal_timer = 0 # 상시조향코드 추가로 인한 추가

    self.steer_mode = ""
    self.mdps_status = ""
    self.lkas_switch = ""

    self.lkas11_cnt = 0

    self.nBlinker = 0

    self.dRel = 0
    self.yRel = 0
    self.vRel = 0

    self.timer1 = tm.CTime1000("time")
    self.model_speed = 0
    self.model_sum = 0
    
    # hud
    self.hud_timer_left = 0
    self.hud_timer_right = 0

    self.command_cnt = 0
    self.command_load = 0
    self.params = Params()

    # param
    self.param_preOpkrAccelProfile = -1
    self.param_OpkrAccelProfile = 0
    self.param_OpkrAutoResume = 0
    self.param_OpkrEnableLearner = 0

    self.SC = None
    self.traceCC = trace1.Loger("CarController")

    self.res_cnt = 7
    self.res_delay = 0

    kyd = kyd_conf()
    self.driver_steering_torque_above = float(kyd.conf['driverSteeringTorqueAbove'])

    self.params = Params()
    self.mode_change_switch = int(self.params.get('CruiseStatemodeSelInit'))

  def process_hud_alert(self, enabled, CC):
    visual_alert = CC.hudControl.visualAlert
    left_lane = CC.hudControl.leftLaneVisible
    right_lane = CC.hudControl.rightLaneVisible

    sys_warning = (visual_alert == VisualAlert.steerRequired)

    if left_lane:
      self.hud_timer_left = 100

    if right_lane:
      self.hud_timer_right = 100

    if self.hud_timer_left:
      self.hud_timer_left -= 1

    if self.hud_timer_right:
      self.hud_timer_right -= 1

    # initialize to no line visible
    sys_state = 1
    if self.hud_timer_left and self.hud_timer_right or sys_warning:  # HUD alert only display when LKAS status is active
      if enabled or sys_warning:
        sys_state = 3
      else:
        sys_state = 4
    elif self.hud_timer_left:
      sys_state = 5
    elif self.hud_timer_right:
      sys_state = 6

    return sys_warning, sys_state

  def param_load(self ):
    self.command_cnt += 1
    if self.command_cnt > 100:
      self.command_cnt = 0

    if self.command_cnt % 10:
      return

    self.command_load += 1
    if self.command_load == 1:
      self.param_OpkrAccelProfile = int(self.params.get('OpkrAccelProfile')) 
    elif self.command_load == 2:
      self.param_OpkrAutoResume = int(self.params.get('OpkrAutoResume'))
    else:
      self.command_load = 0
    
    self.param_OpkrEnableLearner = int(self.params.get('OpkrEnableLearner'))

    # speed controller
    if self.param_preOpkrAccelProfile != self.param_OpkrAccelProfile:
      self.param_preOpkrAccelProfile = self.param_OpkrAccelProfile
      if self.param_OpkrAccelProfile == 1:
        self.SC = SpdctrlSlow()
      elif self.param_OpkrAccelProfile == 2:
        self.SC = SpdctrlNormal()
      else:
        self.SC = SpdctrlFast()


  #아톰님 보간함수 참조
  def cV_tune( self, v_ego, cv_value ):  # cV(곡률에 의한 변화)
    kyd = kyd_conf()
    self.sRKPHV = [9., 22.]
    self.cVBPV = kyd.conf['cvBPV']   # 곡률
    self.cvSteerMaxV1  = kyd.conf['cvSteerMaxV1']
    self.cvSteerDeltaUpV1 = kyd.conf['cvSteerDeltaUpV1']
    self.cvSteerDeltaDnV1 = kyd.conf['cvSteerDeltaDnV1']
    self.cvSteerMaxV2 = kyd.conf['cvSteerMaxV2']
    self.cvSteerDeltaUpV2 = kyd.conf['cvSteerDeltaUpV2']
    self.cvSteerDeltaDnV2 = kyd.conf['cvSteerDeltaDnV2']

    cv_BPV = self.cVBPV   # 곡률
    # Max
    self.steerMax1 = interp( cv_value, cv_BPV, self.cvSteerMaxV1 )
    self.steerMax2 = interp( cv_value, cv_BPV, self.cvSteerMaxV2 )
    self.steerMaxV = [ float(self.steerMax1), float(self.steerMax2) ]
    self.MAX = interp( v_ego, self.sRKPHV, self.steerMaxV )  

    # Up
    self.steerUP1 = interp( cv_value, cv_BPV, self.cvSteerDeltaUpV1 )
    self.steerUP2 = interp( cv_value, cv_BPV, self.cvSteerDeltaUpV2 )
    self.steerUPV = [ float(self.steerUP1), float(self.steerUP2) ]
    self.UP = interp( v_ego, self.sRKPHV, self.steerUPV )

    # Dn
    self.steerDN1 = interp( cv_value, cv_BPV, self.cvSteerDeltaDnV1 )
    self.steerDN2 = interp( cv_value, cv_BPV, self.cvSteerDeltaDnV2 )    
    self.steerDNV = [ float(self.steerDN1), float(self.steerDN2) ]
    self.DN = interp( v_ego, self.sRKPHV, self.steerDNV )


  def update(self, CC, CS, frame, sm, CP ):

    if self.CP != CP:
      self.CP = CP

    self.param_load()
    
    enabled = CC.enabled
    actuators = CC.actuators
    pcm_cancel_cmd = CC.cruiseControl.cancel
    
    self.dRel, self.yRel, self.vRel = SpdController.get_lead( sm )

    if self.SC is not None:
      self.model_speed, self.model_sum = self.SC.calc_va(  sm, CS.out.vEgo  )
    else:
      self.model_speed = self.model_sum = 0

    # Steering Torque
    if self.param_OpkrEnableLearner:
      new_steer = actuators.steer * SteerLimitParams.STEER_MAX
      apply_steer = apply_std_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, SteerLimitParams)
      self.steer_rate_limited = new_steer != apply_steer
    else:
      path_plan = sm['pathPlan']
      self.cV_tune( CS.out.vEgo, self.model_speed )
      param = SteerLimitParams()
      param.STEER_MAX = min( param.STEER_MAX, self.MAX )
      param.STEER_DELTA_UP = min( param.STEER_DELTA_UP, self.UP )
      param.STEER_DELTA_DOWN = min( param.STEER_DELTA_DOWN, self.DN )
      new_steer = actuators.steer * param.STEER_MAX
      apply_steer = apply_std_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, param)    
      self.steer_rate_limited = new_steer != apply_steer


    # disable if steer angle reach 90 deg, otherwise mdps fault in some models
    lkas_active = enabled and abs(CS.out.steeringAngle) < 90. #TenesiDel -> and self.lkas_button_on

    # fix for Genesis hard fault at low speed 테네시 추가
    if CS.out.vEgo < 60 * CV.KPH_TO_MS and self.car_fingerprint == CAR.GENESIS and not CS.mdps_bus:
      lkas_active = False

#    if (( CS.out.leftBlinker and not CS.out.rightBlinker) or ( CS.out.rightBlinker and not CS.out.leftBlinker)) and CS.out.vEgo > 60 * CV.KPH_TO_MS: # 깜빡이 작동시에도 상히조향 유지 수정해보기
#      self.lanechange_manual_timer = 10
#    if CS.out.leftBlinker and CS.out.rightBlinker:
#      self.emergency_manual_timer = 10
#    if abs(CS.out.steeringTorque) > self.driver_steering_torque_above and CS.out.vEgo > 60: # 깜빡이 작동시에도 상히조향 유지 수정해보기
#      self.driver_steering_torque_above_timer = 30
#    if self.lanechange_manual_timer or self.driver_steering_torque_above_timer:
#      lkas_active = 0
#    if self.lanechange_manual_timer > 0:
#      self.lanechange_manual_timer -= 1
#    if self.emergency_manual_timer > 0:
#      self.emergency_manual_timer -= 1
#    if self.driver_steering_torque_above_timer > 0:
#      self.driver_steering_torque_above_timer -= 1

    # Disable steering while turning blinker on and speed below 60 kph #201011 상시조향 작업 작동성공
    if CS.out.leftBlinker or CS.out.rightBlinker:
      if self.car_fingerprint not in [CAR.K5, CAR.K5_HEV]: # 테네시 추가 OPTIMA -> K5
        self.turning_signal_timer = 100  # Disable for 1.0 Seconds after blinker turned off
      elif CS.left_blinker_flash or CS.right_blinker_flash:  # Optima has blinker flash signal only
        self.turning_signal_timer = 100
    if self.turning_signal_timer and CS.out.vEgo > 70 * CV.KPH_TO_MS:  # TenesiADD Blinker tune 시속60미만에서는 상시조향
      lkas_active = 0
    if self.turning_signal_timer:  # TenesiADD
      self.turning_signal_timer -= 1 #201011 상시조향 작업 작동성공

    if not lkas_active:
      apply_steer = 0

    steer_req = 1 if apply_steer else 0      

    self.apply_steer_last = apply_steer

    sys_warning, sys_state = self.process_hud_alert( lkas_active, CC )

    clu11_speed = CS.clu11["CF_Clu_Vanz"]
    enabled_speed = 38 if CS.is_set_speed_in_mph  else 55
    if clu11_speed > enabled_speed:
      enabled_speed = clu11_speed

    can_sends = []
    if frame == 0: # initialize counts from last received count signals
      self.lkas11_cnt = CS.lkas11["CF_Lkas_MsgCount"] + 1
    self.lkas11_cnt %= 0x10

    can_sends.append(create_lkas11(self.packer, self.lkas11_cnt, self.car_fingerprint, apply_steer, steer_req,
                                   CS.lkas11, sys_warning, sys_state, CC, enabled, 0 ))
    if CS.mdps_bus or CS.scc_bus == 1: # send lkas11 bus 1 if mdps is on bus 1                               
      can_sends.append(create_lkas11(self.packer, self.lkas11_cnt, self.car_fingerprint, apply_steer, steer_req,
                                   CS.lkas11, sys_warning, sys_state, CC, enabled, 1 ))

    if CS.mdps_bus: # send clu11 to mdps if it is not on bus 0                                   
    #if frame % 2 and CS.mdps_bus == 1: # send clu11 to mdps if it is not on bus 0                                   
      can_sends.append(create_clu11(self.packer, frame, CS.mdps_bus, CS.clu11, Buttons.NONE, enabled_speed))
    
    #if CS.mdps_bus:
    can_sends.append(create_mdps12(self.packer, frame, CS.mdps12))                                   

    str_log1 = '곡률={:05.1f}/{:=+06.3f}  토크={:=+04.0f}/{:=+04.0f}'.format(  self.model_speed, self.model_sum, new_steer, CS.out.steeringTorque )
    if self.param_OpkrEnableLearner:
      str_log2 = '프레임율={:03.0f}  STMAX={:03.0f}'.format( self.timer1.sampleTime(), SteerLimitParams.STEER_MAX, )
    else:
      str_log2 = '프레임율={:03.0f}  ST={:03.0f}/{:01.0f}/{:01.0f}  SR={:05.2f}'.format( self.timer1.sampleTime(), self.MAX, self.UP, self.DN, path_plan.steerRatio )
    trace1.printf( '{}  {}'.format( str_log1, str_log2 ) )

    if CS.out.cruiseState.modeSel == 0 and self.mode_change_switch == 4:
      self.mode_change_timer = 50
      self.mode_change_switch = 0
    elif CS.out.cruiseState.modeSel == 1 and self.mode_change_switch == 0:
      self.mode_change_timer = 50
      self.mode_change_switch = 1
    elif CS.out.cruiseState.modeSel == 2 and self.mode_change_switch == 1:
      self.mode_change_timer = 50
      self.mode_change_switch = 2
    elif CS.out.cruiseState.modeSel == 3 and self.mode_change_switch == 2:
      self.mode_change_timer = 50
      self.mode_change_switch = 3
    elif CS.out.cruiseState.modeSel == 4 and self.mode_change_switch == 3:
      self.mode_change_timer = 50
      self.mode_change_switch = 4
    if self.mode_change_timer > 0:
      self.mode_change_timer -= 1

    run_speed_ctrl = self.param_OpkrAccelProfile and CS.acc_active and self.SC != None and (CS.out.cruiseState.modeSel == 1 or CS.out.cruiseState.modeSel == 2 or CS.out.cruiseState.modeSel == 3)
    if not run_speed_ctrl:
      if CS.out.cruiseState.modeSel == 0:
        self.steer_mode = "오파모드"
      elif CS.out.cruiseState.modeSel == 1:
        self.steer_mode = "차간+커브"
      elif CS.out.cruiseState.modeSel == 2:
        self.steer_mode = "차간ONLY"
      elif CS.out.cruiseState.modeSel == 3:
        self.steer_mode = "자동RES"
      elif CS.out.cruiseState.modeSel == 4:
        self.steer_mode = "순정모드"
      if CS.out.steerWarning == 0:
        self.mdps_status = "정상"
      elif CS.out.steerWarning == 1:
        self.mdps_status = "오류"
      if CS.lkas_button_on == 0:
        self.lkas_switch = "OFF"
      elif CS.lkas_button_on == 1:
        self.lkas_switch = "ON"
      else:
        self.lkas_switch = "-"
      
      if CS.out.cruiseState.modeSel == 3:
        str_log2 = '주행모드={:s}  MDPS상태={:s}  LKAS버튼={:s}  AUTORES=(VS:{:03.0f}/CN:{:01.0f}/RD:{:03.0f}/BK:{})'.format( self.steer_mode, self.mdps_status, self.lkas_switch, CS.VSetDis, self.res_cnt, self.res_delay, CS.out.brakeLights )
      else:
        str_log2 = '주행모드={:s}  MDPS상태={:s}  LKAS버튼={:s}'.format( self.steer_mode, self.mdps_status, self.lkas_switch )
      trace1.printf2( '{}'.format( str_log2 ) )

    #print( 'st={} cmd={} long={}  steer={} req={}'.format(CS.out.cruiseState.standstill, pcm_cancel_cmd, self.CP.openpilotLongitudinalControl, apply_steer, steer_req ) )


    if pcm_cancel_cmd and self.CP.openpilotLongitudinalControl:
      can_sends.append(create_clu11(self.packer, frame, CS.scc_bus, CS.clu11, Buttons.CANCEL, clu11_speed))
    elif CS.out.cruiseState.standstill and not self.car_fingerprint == CAR.NIRO_EV:
      # run only first time when the car stopped
      if self.last_lead_distance == 0 or not self.param_OpkrAutoResume:
        # get the lead distance from the Radar
        self.last_lead_distance = CS.lead_distance
        self.resume_cnt = 0
      # when lead car starts moving, create 6 RES msgs
      elif CS.lead_distance != self.last_lead_distance and (frame - self.last_resume_frame) > 5:
        can_sends.append(create_clu11(self.packer, frame, CS.scc_bus, CS.clu11, Buttons.RES_ACCEL, clu11_speed))
        self.resume_cnt += 1
        # interval after 6 msgs
        if self.resume_cnt > 5:
          self.last_resume_frame = frame
          self.resume_cnt = 0
    elif CS.out.cruiseState.standstill and self.car_fingerprint == CAR.NIRO_EV:
      if CS.lead_distance > 3.7 and (frame - self.last_resume_frame)*DT_CTRL > 0.2 and self.param_OpkrAutoResume:
        can_sends.append(create_clu11(self.packer, frame, CS.scc_bus, CS.clu11, Buttons.RES_ACCEL, clu11_speed))
        self.last_resume_frame = frame

    # reset lead distnce after the car starts moving
    elif self.last_lead_distance != 0:
      self.last_lead_distance = 0
    elif run_speed_ctrl and self.SC != None:
      is_sc_run = self.SC.update( CS, sm, self )
      if is_sc_run:
        can_sends.append(create_clu11(self.packer, self.resume_cnt, CS.scc_bus, CS.clu11, self.SC.btn_type, self.SC.sc_clu_speed ))
        self.resume_cnt += 1
      else:
        self.resume_cnt = 0
    
    if CS.out.cruiseState.modeSel == 3:
      if CS.out.brakeLights and CS.VSetDis > 30:
        self.res_cnt = 0
        self.res_delay = 50
      elif self.res_delay:
        self.res_delay -= 1
      elif not self.res_delay and self.res_cnt < 6 and CS.VSetDis > 30 and CS.out.vEgo > 30 * CV.KPH_TO_MS:
        if self.res_cnt < 1:
          can_sends.append(create_clu11(self.packer, frame, CS.scc_bus, CS.clu11, Buttons.CANCEL, clu11_speed))
        can_sends.append(create_clu11(self.packer, frame, CS.scc_bus, CS.clu11, Buttons.RES_ACCEL, clu11_speed))
        self.res_cnt += 1
      else:
        self.res_cnt = 7
        self.res_delay = 0

    # 20 Hz LFA MFA message
    if frame % 5 == 0 and self.car_fingerprint in FEATURES["send_lfa_mfa"]:
      can_sends.append(create_lfa_mfa(self.packer, frame, enabled))

    self.lkas11_cnt += 1
    return can_sends

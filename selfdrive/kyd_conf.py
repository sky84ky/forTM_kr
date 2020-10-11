import json
import os

class kyd_conf():
  def __init__(self, CP=None):
    self.conf = self.read_config()
    if CP is not None:
      self.init_config(CP)

  def init_config(self, CP):
    write_conf = False
    if self.conf['EnableLiveTune'] != "1":
      self.conf['EnableLiveTune'] = str(1)
      write_conf = True

    # only fetch Kp, Ki, Kf sR and sRC from interface.py if it's a PID controlled car
    if CP.lateralTuning.which() == 'pid':
      if self.conf['Kp'] == "-1":
        self.conf['Kp'] = str(round(CP.lateralTuning.pid.kpV[0],3))
        write_conf = True
      if self.conf['Ki'] == "-1":
        self.conf['Ki'] = str(round(CP.lateralTuning.pid.kiV[0],3))
        write_conf = True
      if self.conf['Kf'] == "-1":
        self.conf['Kf'] = str('{:f}'.format(CP.lateralTuning.pid.kf))
        write_conf = True
    elif CP.lateralTuning.which() == 'indi':
      if self.conf['outerLoopGain'] == "-1":
        self.conf['outerLoopGain'] = str(round(CP.lateralTuning.indi.outerLoopGain,2))
        write_conf = True
      if self.conf['innerLoopGain'] == "-1":
        self.conf['innerLoopGain'] = str(round(CP.lateralTuning.indi.innerLoopGain,2))
        write_conf = True
      if self.conf['timeConstant'] == "-1":
        self.conf['timeConstant'] = str(round(CP.lateralTuning.indi.timeConstant,2))
        write_conf = True
      if self.conf['actuatorEffectiveness'] == "-1":
        self.conf['actuatorEffectiveness'] = str(round(CP.lateralTuning.indi.actuatorEffectiveness,2))
        write_conf = True
    elif CP.lateralTuning.which() == 'lqr':
      if self.conf['scale'] == "-1":
        self.conf['scale'] = str(round(CP.lateralTuning.lqr.scale,2))
        write_conf = True
      if self.conf['ki'] == "-1":
        self.conf['ki'] = str(round(CP.lateralTuning.lqr.ki,3))
        write_conf = True
      if self.conf['dc_gain'] == "-1":
        self.conf['dc_gain'] = str('{:f}'.format(CP.lateralTuning.lqr.dcGain))
        write_conf = True
    
    if self.conf['steerRatio'] == "-1":
      self.conf['steerRatio'] = str(round(CP.steerRatio,3))
      write_conf = True
    
    if self.conf['steerRateCost'] == "-1":
      self.conf['steerRateCost'] = str(round(CP.steerRateCost,3))
      write_conf = True

    if write_conf:
      self.write_config(self.config)

  def read_config(self):
    self.element_updated = False

    if os.path.isfile('/data/kyd.json'):
      with open('/data/kyd.json', 'r') as f:
        self.config = json.load(f)
        
      if "EnableLiveTune" not in self.config:
        self.config.update({"EnableLiveTune":"1"})
        self.element_updated = True

      if "cameraOffset" not in self.config:
        self.config.update({"cameraOffset":"0.06"})
        self.element_updated = True
        
      if "steerAngleCorrection" not in self.config:
        self.config.update({"steerAngleCorrection":"0.0"})
        self.element_updated = True

      if "Kp" not in self.config:
        self.config.update({"Kp":"-1"})
        self.config.update({"Ki":"-1"})
        self.config.update({"Kf":"-1"})
        self.element_updated = True
	
      if "steerRatio" not in self.config:
        self.config.update({"steerRatio":"-1"})
        self.config.update({"steerRateCost":"-1"})
        self.config.update({"deadzone":"0.0"})
        self.element_updated = True
	
      if "sR_Boost" not in self.config:
        self.config.update({"sR_Boost":[0.0, 0.0]})
        self.config.update({"sR_BP":[0.0, 0.0]})
        self.element_updated = True
        
      if "outerLoopGain" not in self.config:
        self.config.update({"outerLoopGain":"-1"})
        self.config.update({"innerLoopGain":"-1"})
        self.config.update({"timeConstant":"-1"})
        self.config.update({"actuatorEffectiveness":"-1"})
        self.element_updated = True

      if "scale" not in self.config:
        self.config.update({"scale":"-1"})
        self.config.update({"ki":"-1"})
        self.config.update({"dc_gain":"-1"})
        self.element_updated = True

      if "steerMax" not in self.config:
        self.config.update({"steerMax":"255"})
        self.config.update({"steerDeltaUp":"3"})
        self.config.update({"steerDeltaDown":"7"})
        self.element_updated = True

      if "cvBPV" not in self.config:
        self.config.update({"cvBPV":[50,255]})
        self.config.update({"cvSteerMaxV1":[300,230]})
        self.config.update({"cvSteerMaxV2":[409,255]})
        self.config.update({"cvSteerDeltaUpV1":[3,3]})
        self.config.update({"cvSteerDeltaUpV2":[4,3]})
        self.config.update({"cvSteerDeltaDnV1":[6,4]})
        self.config.update({"cvSteerDeltaDnV2":[8,4]})
        self.element_updated = True

      if "SteerActuatorDelay" not in self.config:
        self.config.update({"SteerActuatorDelay":"0.15"})
        self.config.update({"SteerLimitTimer":"0.8"})
        self.element_updated = True

      if "driverSteeringTorqueAbove" not in self.config:
        self.config.update({"driverSteeringTorqueAbove":"200"})
        self.element_updated = True

      if self.element_updated:
        print("updated")
        self.write_config(self.config)

    else:
      self.config = {"EnableLiveTune":"1", "steerMax":"384", "steerDeltaUp":"3", "steerDeltaDown":"7", \
      	             "steerAngleCorrection":"0.0", "cameraOffset":"0.06", "SteerActuatorDelay":"0.15", "SteerLimitTimer":"0.8", \
      	             "Kp":"-1", "Ki":"-1", "Kf":"-1", "driverSteeringTorqueAbove":"200", \
      	             "outerLoopGain":"-1", "innerLoopGain":"-1", "timeConstant":"-1", "actuatorEffectiveness":"-1", \
                     "scale":"-1", "ki":"-1", "dc_gain":"-1", \
                     "steerRatio":"-1", "steerRateCost":"-1", "deadzone":"0.0", \
                     "sR_Boost":[0.0, 0.0], "sR_BP":[0.0, 0.0], \
                     "cvBPV":[50,255], "cvSteerMaxV1":[300,230], "cvSteerMaxV2":[409,255], "cvSteerDeltaUpV1":[3,3], "cvSteerDeltaUpV2":[4,3], "cvSteerDeltaDnV1":[6,4], "cvSteerDeltaDnV2":[8,4]}
      self.write_config(self.config)
    return self.config

  def write_config(self, config):
    try:
      with open('/data/kyd.json', 'w') as f:
        json.dump(self.config, f, indent=2, sort_keys=True)
        os.chmod("/data/kyd.json", 0o764)
    except IOError:
      os.mkdir('/data')
      with open('/data/kyd.json', 'w') as f:
        json.dump(self.config, f, indent=2, sort_keys=True)
        os.chmod("/data/kyd.json", 0o764)

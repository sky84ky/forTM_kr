from cereal import car, log

# Priority
class Priority:
  LOWEST = 0
  LOW_LOWEST = 1
  LOW = 2
  MID = 3
  HIGH = 4
  HIGHEST = 5

AlertSize = log.ControlsState.AlertSize
AlertStatus = log.ControlsState.AlertStatus
AudibleAlert = car.CarControl.HUDControl.AudibleAlert
VisualAlert = car.CarControl.HUDControl.VisualAlert

class Alert():
  def __init__(self,
               alert_type,
               alert_text_1,
               alert_text_2,
               alert_status,
               alert_size,
               alert_priority,
               visual_alert,
               audible_alert,
               duration_sound,
               duration_hud_alert,
               duration_text,
               alert_rate=0.):

    self.alert_type = alert_type
    self.alert_text_1 = alert_text_1
    self.alert_text_2 = alert_text_2
    self.alert_status = alert_status
    self.alert_size = alert_size
    self.alert_priority = alert_priority
    self.visual_alert = visual_alert
    self.audible_alert = audible_alert

    self.duration_sound = duration_sound
    self.duration_hud_alert = duration_hud_alert
    self.duration_text = duration_text

    self.start_time = 0.
    self.alert_rate = alert_rate

    # typecheck that enums are valid on startup
    tst = car.CarControl.new_message()
    tst.hudControl.visualAlert = self.visual_alert

  def __str__(self):
    return self.alert_text_1 + "/" + self.alert_text_2 + " " + str(self.alert_priority) + "  " + str(
      self.visual_alert) + " " + str(self.audible_alert)

  def __gt__(self, alert2):
    return self.alert_priority > alert2.alert_priority

ALERTS = [
  Alert(
      "turningIndicatorOn",
      "방향지시등 동작시에는 핸들조향 꺼짐",
      "",
      AlertStatus.userPrompt, AlertSize.small,
      Priority.MID, VisualAlert.none, AudibleAlert.none, 0., 0., .1),
  Alert(
      "lkasButtonOff",
      "LKAS 버튼 꺼짐",
      "",
      AlertStatus.userPrompt, AlertSize.small,
      Priority.HIGH, VisualAlert.none, AudibleAlert.chimeLkas, 2.1, 0., .1),

  # Miscellaneous alerts
  Alert(
      "enable",
      "활성화",
      "",
      AlertStatus.normal, AlertSize.none,
      Priority.MID, VisualAlert.none, AudibleAlert.chimeEngage, 1.9, 0., 0.),

  Alert(
      "disable",
      "비활성화",
      "",
      AlertStatus.normal, AlertSize.none,
      Priority.MID, VisualAlert.none, AudibleAlert.chimeDisengage, 2.0, 0., 0.),

  Alert(
      "fcw",
      "브레이크!",
      "충돌 위험",
      AlertStatus.critical, AlertSize.full,
      Priority.HIGHEST, VisualAlert.fcw, AudibleAlert.chimeWarningRepeat, 1., 2., 2.),

  Alert(
      "fcwStock",
      "브레이크!",
      "충돌 위험",
      AlertStatus.critical, AlertSize.full,
      Priority.HIGHEST, VisualAlert.fcw, AudibleAlert.none, 1., 2., 2.),  # no EON chime for stock FCW

  Alert(
      "steerSaturated",
      "핸들을 조작하세요",
      "핸들조향 제한을 초과함",
      AlertStatus.userPrompt, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, 1., 2., 3.),

  Alert(
      "steerTempUnavailable",
      "핸들을 조작하세요",
      "핸들조향 일시적으로 사용불가",
      AlertStatus.userPrompt, AlertSize.mid,
      Priority.LOW, VisualAlert.steerRequired, AudibleAlert.chimeWarning1, .4, 2., 3.),

  Alert(
      "steerTempUnavailableMute",
      "핸들을 조작하세요",
      "핸들조향 일시적으로 사용불가",
      AlertStatus.userPrompt, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .2, .2, .2),

  Alert(
      "preDriverDistracted",
      "도로를 주시하세요 : 사용자 도로주시 불안",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.chimeDistracted, 4.0, .1, .1, alert_rate=1),

  Alert(
      "promptDriverDistracted",
      "도로를 주시하세요",
      "사용자 도로주시 불안",
      AlertStatus.userPrompt, AlertSize.mid,
      Priority.MID, VisualAlert.steerRequired, AudibleAlert.chimeWarning2, .1, .1, .1),

  Alert(
      "driverDistracted",
      "즉시 해제하세요",
      "사용자 주의 불안",
      AlertStatus.critical, AlertSize.full,
      Priority.HIGH, VisualAlert.steerRequired, AudibleAlert.chimeWarningRepeat, .1, .1, .1),

  Alert(
      "preDriverUnresponsive",
      "핸들을 조작하세요 : 얼굴 인식 불가",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.steerRequired, AudibleAlert.none, .0, .1, .1, alert_rate=1),

  Alert(
      "promptDriverUnresponsive",
      "핸들을 조작하세요",
      "사용자가 응답하지않음",
      AlertStatus.userPrompt, AlertSize.mid,
      Priority.MID, VisualAlert.steerRequired, AudibleAlert.chimeWarning2, .1, .1, .1),

  Alert(
      "driverUnresponsive",
      "즉시 해제하세요",
      "사용자가 응답하지않음",
      AlertStatus.critical, AlertSize.full,
      Priority.HIGH, VisualAlert.steerRequired, AudibleAlert.chimeWarningRepeat, .1, .1, .1),

  Alert(
      "driverMonitorLowAcc",
      "운전자 모니터링 확인",
      "운전자 모니터링이 비정상적입니다",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.steerRequired, AudibleAlert.none, .4, 0., 1.),

  Alert(
      "geofence",
      "오픈파일럿을 해제하세요",
      "지오펜스 영역이 아닙니다",
      AlertStatus.userPrompt, AlertSize.mid,
      Priority.HIGH, VisualAlert.steerRequired, AudibleAlert.chimeWarningRepeat, .1, .1, .1),

  Alert(
      "startup",
      "오픈파일럿 사용준비 완료",
      "항상 핸들을 잡고 도로를 주시하세요",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW_LOWEST, VisualAlert.none, AudibleAlert.chimeStartup, 4.7, 0., 5.),

  Alert(
      "startupMaster",
      "테스트되지않은 브랜치",
      "항상 핸들을 잡고 도로를 주시하세요",
      AlertStatus.userPrompt, AlertSize.mid,
      Priority.LOW_LOWEST, VisualAlert.none, AudibleAlert.none, 0., 0., 5.),

  Alert(
      "startupNoControl",
      "대시캠 모드",
      "항상 핸들을 잡고 도로를 주시하세요",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW_LOWEST, VisualAlert.none, AudibleAlert.none, 0., 0., 15.),

  Alert(
      "startupNoCar",
      "대시캠 모드 : 호환되지않는 차량",
      "항상 핸들을 잡고 도로를 주시하세요",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW_LOWEST, VisualAlert.none, AudibleAlert.none, 0., 0., 15.),

  Alert(
      "ethicalDilemma",
      "핸들을 즉시 조작하세요",
      "도덕적인 딜레마 탐지",
      AlertStatus.critical, AlertSize.full,
      Priority.HIGHEST, VisualAlert.steerRequired, AudibleAlert.chimeWarningRepeat, 1., 3., 3.),

  Alert(
      "steerTempUnavailableNoEntry",
      "오픈파일럿 사용불가",
      "핸들조향 일시적으로 사용불가",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.chimeError, .4, 0., 3.),

  Alert(
      "manualRestart",
      "핸들을 조작하세요",
      "수동으로 재활성화하세요",
      AlertStatus.userPrompt, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, 0., 0., .2),

  Alert(
      "resumeRequired",
      "멈춤",
      "이동하려면 계속을 누르세요",
      AlertStatus.userPrompt, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, 0., 0., .2),

  Alert(
      "belowSteerSpeed",
      "핸들을 조작하세요",
      "사용할수없는 자동조향 : 속도 ",
      AlertStatus.userPrompt, AlertSize.small,
      Priority.MID, VisualAlert.none, AudibleAlert.none, 0., 0.4, .3),

  Alert(
      "debugAlert",
      "디버그 경고",
      "",
      AlertStatus.userPrompt, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .1, .1, .1),
  Alert(
      "preLaneChangeLeft",
      "자동차선변경",
      "좌측차선의 차량을 확인하세요",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .0, .1, .1, alert_rate=0.75),

  Alert(
      "preLaneChangeRight",
      "자동차선변경",
      "우측차선의 차량을 확인하세요",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .0, .1, .1, alert_rate=0.75),

  Alert(
      "laneChange",
      "차선을 변경합니다",
      "차선의 차량을 확인하세요",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .0, .1, .1),
  
    Alert(
      "rightLCAbsm",
      "우측차선 차량감지",
      "우측차선에 차량이 감지되니 대기하세요",
      AlertStatus.userPrompt, AlertSize.mid,
      Priority.MID, VisualAlert.none, AudibleAlert.none, 0., 0.4, .3),
  
  Alert(
      "leftLCAbsm",
      "좌측차선 차량감지",
      "좌측차선에 차량이 감지되니 대기하세요",
      AlertStatus.userPrompt, AlertSize.mid,
      Priority.MID, VisualAlert.none, AudibleAlert.none, 0., 0.4, .3),
  
  Alert(
      "preventLCA",
      "핸들을 조작하세요",
      "차선변경 취소 , 차선이 안전하지않습니다",
      AlertStatus.critical, AlertSize.full,
      Priority.HIGH, VisualAlert.steerRequired, AudibleAlert.chimeWarningRepeat, .4, 3., 3.,),

  Alert(
      "posenetInvalid",
      "핸들을 조작하세요",
      "비전 모델 출력 불확실",
      AlertStatus.userPrompt, AlertSize.mid,
      Priority.LOW, VisualAlert.steerRequired, AudibleAlert.chimeVision, 3.5, 2., 3.),

  # Non-entry only alerts
  Alert(
      "wrongCarModeNoEntry",
      "오픈파일럿 사용불가",
      "메인 스위치 꺼짐",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.chimeError, .4, 0., 3.),

  Alert(
      "dataNeededNoEntry",
      "오픈파일럿 사용불가",
      "캘리브레이션에 필요한 데이터가 없음 , 주행후 재시도하세요",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.chimeError, .4, 0., 3.),

  Alert(
      "outOfSpaceNoEntry",
      "오픈파일럿 사용불가",
      "저장공간 부족",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.chimeError, .4, 0., 3.),

  Alert(
      "pedalPressedNoEntry",
      "오픈파일럿 사용불가",
      "활성화시도중 브레이크 감지됨",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, "brakePressed", AudibleAlert.chimeError, .4, 2., 3.),

  Alert(
      "speedTooLowNoEntry",
      "오픈파일럿 사용불가",
      "속도가 너무 낮음",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.chimeError, .4, 2., 3.),

  Alert(
      "brakeHoldNoEntry",
      "오픈파일럿 사용불가",
      "브레이크 감지됨",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.chimeError, .4, 2., 3.),

  Alert(
      "parkBrakeNoEntry",
      "오픈파일럿 사용불가",
      "주차 브레이크 채결됨",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.chimeError, .4, 2., 3.),

  Alert(
      "lowSpeedLockoutNoEntry",
      "오픈파일럿 사용불가",
      "크루즈 에러 : 차량을 다시 시작하세요",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.chimeError, .4, 2., 3.),

  Alert(
      "lowBatteryNoEntry",
      "오픈파일럿 사용불가",
      "배터리 부족",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.chimeError, .4, 2., 3.),

  Alert(
      "sensorDataInvalidNoEntry",
      "오픈파일럿 사용불가",
      "EON센서에 데이터가 없음",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.chimeError, .4, 2., 3.),

  Alert(
      "soundsUnavailableNoEntry",
      "오픈파일럿 사용불가",
      "스피커가 없습니다",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.chimeError, .4, 2., 3.),

  Alert(
      "tooDistractedNoEntry",
      "오픈파일럿 사용불가",
      "방해 수준이 너무높음",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.chimeError, .4, 2., 3.),

  # Cancellation alerts causing soft disabling
  Alert(
      "overheat",
      "핸들을 즉시 조작하세요",
      "시스템 과열",
      AlertStatus.critical, AlertSize.full,
      Priority.MID, VisualAlert.steerRequired, AudibleAlert.chimeWarningRepeat, .1, 2., 2.),

  Alert(
      "wrongGear",
      "핸들을 즉시 조작하세요",
      "기어가 [D] 상태가 아님",
      AlertStatus.critical, AlertSize.full,
      Priority.MID, VisualAlert.steerRequired, AudibleAlert.chimeWarningRepeat, .1, 2., 2.),

  Alert(
      "calibrationInvalid",
      "핸들을 즉시 조작하세요",
      "캘리브레이션 에러 : EON 위치변경 후 재시도하세요",
      AlertStatus.critical, AlertSize.full,
      Priority.MID, VisualAlert.steerRequired, AudibleAlert.chimeWarningRepeat, .1, 2., 2.),

  Alert(
      "calibrationIncomplete",
      "핸들을 즉시 조작하세요",
      "캘리브레이션 진행중",
      AlertStatus.critical, AlertSize.full,
      Priority.MID, VisualAlert.steerRequired, AudibleAlert.chimeWarningRepeat, .1, 2., 2.),

  Alert(
      "doorOpen",
      "핸들을 즉시 조작하세요",
      "도어 열림",
      AlertStatus.critical, AlertSize.full,
      Priority.MID, VisualAlert.steerRequired, AudibleAlert.chimeWarningRepeat, .1, 2., 2.),

  Alert(
      "seatbeltNotLatched",
      "핸들을 즉시 조작하세요",
      "안전벨트 미채결",
      AlertStatus.critical, AlertSize.full,
      Priority.MID, VisualAlert.steerRequired, AudibleAlert.chimeWarningRepeat, .1, 2., 2.),

  Alert(
      "espDisabled",
      "핸들을 즉시 조작하세요",
      "ESP 꺼짐",
      AlertStatus.critical, AlertSize.full,
      Priority.MID, VisualAlert.steerRequired, AudibleAlert.chimeWarningRepeat, .1, 2., 2.),

  Alert(
      "lowBattery",
      "핸들을 즉시 조작하세요",
      "배터리 부족",
      AlertStatus.critical, AlertSize.full,
      Priority.MID, VisualAlert.steerRequired, AudibleAlert.chimeWarningRepeat, .1, 2., 2.),

  Alert(
      "commIssue",
      "핸들을 즉시 조작하세요",
      "프로세스간 통신문제",
      AlertStatus.critical, AlertSize.full,
      Priority.MID, VisualAlert.steerRequired, AudibleAlert.chimeWarningRepeat, .1, 2., 2.),

  Alert(
      "radarCommIssue",
      "핸들을 즉시 조작하세요",
      "레이더 통신문제",
      AlertStatus.critical, AlertSize.full,
      Priority.MID, VisualAlert.steerRequired, AudibleAlert.chimeWarningRepeat, .1, 2., 2.),

  Alert(
      "radarCanError",
      "핸들을 즉시 조작하세요",
      "레이더 에러 : 차량을 다시 시작하세요",
      AlertStatus.critical, AlertSize.full,
      Priority.MID, VisualAlert.steerRequired, AudibleAlert.chimeWarningRepeat, .1, 2., 2.),

  Alert(
      "radarFault",
      "핸들을 즉시 조작하세요",
      "레이더 에러 : 차량을 다시 시작하세요",
      AlertStatus.critical, AlertSize.full,
      Priority.MID, VisualAlert.steerRequired, AudibleAlert.chimeWarningRepeat, .1, 2., 2.),

  Alert(
      "lowMemory",
      "핸들을 즉시 조작하세요",
      "메모리 부족 : EON을 재부팅하세요",
      AlertStatus.critical, AlertSize.full,
      Priority.MID, VisualAlert.steerRequired, AudibleAlert.chimeWarningRepeat, .1, 2., 2.),

  # Cancellation alerts causing immediate disabling
  Alert(
      "controlsFailed",
      "핸들을 즉시 조작하세요",
      "컨트롤 실패",
      AlertStatus.critical, AlertSize.full,
      Priority.HIGHEST, VisualAlert.steerRequired, AudibleAlert.chimeWarningRepeat, 2.2, 3., 4.),

  Alert(
      "controlsMismatch",
      "핸들을 즉시 조작하세요",
      "컨트롤 불일치",
      AlertStatus.critical, AlertSize.full,
      Priority.HIGHEST, VisualAlert.steerRequired, AudibleAlert.chimeWarningRepeat, 2.2, 3., 4.),

  Alert(
      "canError",
      "핸들을 즉시 조작하세요",
      "CAN 에러 : 연결상태를 확인하세요",
      AlertStatus.critical, AlertSize.full,
      Priority.HIGHEST, VisualAlert.steerRequired, AudibleAlert.chimeWarningRepeat, 2.2, 3., 4.),

  Alert(
      "steerUnavailable",
      "핸들을 즉시 조작하세요",
      "LKAS 에러 : 차량을 다시시작하세요",
      AlertStatus.critical, AlertSize.full,
      Priority.HIGHEST, VisualAlert.steerRequired, AudibleAlert.chimeWarningRepeat, 2.2, 3., 4.),

  Alert(
      "brakeUnavailable",
      "핸들을 즉시 조작하세요",
      "크루즈 에러 : 차량을 다시시작하세요",
      AlertStatus.critical, AlertSize.full,
      Priority.HIGHEST, VisualAlert.steerRequired, AudibleAlert.chimeWarningRepeat, 2.2, 3., 4.),

  Alert(
      "gasUnavailable",
      "핸들을 즉시 조작하세요",
      "Gas 에러 : 차량을 다시시작하세요",
      AlertStatus.critical, AlertSize.full,
      Priority.HIGHEST, VisualAlert.steerRequired, AudibleAlert.chimeWarningRepeat, 2.2, 3., 4.),

  Alert(
      "reverseGear",
      "핸들을 즉시 조작하세요",
      "기어 [R] 상태",
      AlertStatus.critical, AlertSize.full,
      Priority.HIGHEST, VisualAlert.none, AudibleAlert.none, 2.2, 3., 4.),

  Alert(
      "cruiseDisabled",
      "핸들을 즉시 조작하세요",
      "크루즈 꺼짐",
      AlertStatus.critical, AlertSize.full,
      Priority.HIGHEST, VisualAlert.steerRequired, AudibleAlert.chimeWarningRepeat, 2.2, 3., 4.),

  Alert(
      "plannerError",
      "핸들을 즉시 조작하세요",
      "플래너 솔루션 에러",
      AlertStatus.critical, AlertSize.full,
      Priority.HIGHEST, VisualAlert.steerRequired, AudibleAlert.chimeWarningRepeat, 2.2, 3., 4.),

  # not loud cancellations (user is in control)
  Alert(
      "noTarget",
      "오픈파일럿 취소됨",
      "근접 앞차량이 없습니다",
      AlertStatus.normal, AlertSize.mid,
      Priority.HIGH, VisualAlert.none, AudibleAlert.chimeDisengage, .4, 2., 3.),

  Alert(
      "speedTooLow",
      "오픈파일럿 취소됨",
      "속도가 너무낮음",
      AlertStatus.normal, AlertSize.mid,
      Priority.HIGH, VisualAlert.none, AudibleAlert.chimeDisengage, .4, 2., 3.),

  # Cancellation alerts causing non-entry
  Alert(
      "overheatNoEntry",
      "오픈파일럿 사용불가",
      "시스템 과열",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.chimeError, .4, 2., 3.),

  Alert(
      "wrongGearNoEntry",
      "오픈파일럿 사용불가",
      "기어가 [D] 상태가 아님",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.chimeError, .4, 2., 3.),

  Alert(
      "calibrationInvalidNoEntry",
      "오픈파일럿 사용불가",
      "캘리브레이션 에러 : EON 위치변경 및 캘리브레이션 재시도",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.chimeError, .4, 2., 3.),

  Alert(
      "calibrationIncompleteNoEntry",
      "오픈파일럿 사용불가",
      "캘리브레이션 진행중",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.chimeError, .4, 2., 3.),

  Alert(
      "doorOpenNoEntry",
      "오픈파일럿 사용불가",
      "도어 열림",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.chimeError, .4, 2., 3.),

  Alert(
      "seatbeltNotLatchedNoEntry",
      "오픈파일럿 사용불가",
      "안전벨트 미채결",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.chimeError, .4, 2., 3.),

  Alert(
      "espDisabledNoEntry",
      "오픈파일럿 사용불가",
      "ESP 꺼짐",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.chimeError, .4, 2., 3.),

  Alert(
      "geofenceNoEntry",
      "오픈파일럿 사용불가",
      "지오펜스 영역이 아닙니다",
      AlertStatus.normal, AlertSize.mid,
      Priority.MID, VisualAlert.none, AudibleAlert.chimeError, .4, 2., 3.),

  Alert(
      "radarCanErrorNoEntry",
      "오픈파일럿 사용불가",
      "레이더 에러 : 차량을 다시 시작하세요",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.chimeError, .4, 2., 3.),

  Alert(
      "radarFaultNoEntry",
      "오픈파일럿 사용불가",
      "레이더 에러 : 차량을 다시 시작하세요",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.chimeError, .4, 2., 3.),

  Alert(
      "posenetInvalidNoEntry",
      "오픈파일럿 사용불가",
      "비전 모델 출력 불확실",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.chimeVision, 3.5, 2., 3.),

  Alert(
      "controlsFailedNoEntry",
      "오픈파일럿 사용불가",
      "컨트롤 실패",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.chimeError, .4, 2., 3.),

  Alert(
      "canErrorNoEntry",
      "오픈파일럿 사용불가",
      "CAN 에러 : 연결상태를 확인하세요",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.chimeError, .4, 2., 3.),

  Alert(
      "steerUnavailableNoEntry",
      "오픈파일럿 사용불가",
      "LKAS 에러 : 차량을 다시 시작하세요",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.chimeError, .4, 2., 3.),

  Alert(
      "brakeUnavailableNoEntry",
      "오픈파일럿 사용불가",
      "크루즈 에러 : 차량을 다시 시작하세요",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.chimeError, .4, 2., 3.),

  Alert(
      "gasUnavailableNoEntry",
      "오픈파일럿 사용불가",
      "Gas 에러 : 차량을 다시 시작하세요",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.chimeError, .4, 2., 3.),

  Alert(
      "reverseGearNoEntry",
      "오픈파일럿 사용불가",
      "기어 [R] 상태",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.chimeError, .4, 2., 3.),

  Alert(
      "cruiseDisabledNoEntry",
      "오픈파일럿 사용불가",
      "크루즈 오프 상태",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.chimeError, .4, 2., 3.),

  Alert(
      "noTargetNoEntry",
      "오픈파일럿 사용불가",
      "근접 앞차량이 없습니다",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.chimeError, .4, 2., 3.),

  Alert(
      "plannerErrorNoEntry",
      "오픈파일럿 사용불가",
      "플래너 솔루션 에러",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.chimeError, .4, 2., 3.),

  Alert(
      "commIssueNoEntry",
      "오픈파일럿 사용불가",
      "프로세스간 통신문제",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.chimeDisengage, .4, 2., 3.),

  Alert(
      "radarCommIssueNoEntry",
      "오픈파일럿 사용불가",
      "레이더 통신문제",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.chimeDisengage, .4, 2., 3.),

  Alert(
      "internetConnectivityNeededNoEntry",
      "오픈파일럿 사용불가",
      "네트워크를 연결하세요",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.chimeDisengage, .4, 2., 3.),

  Alert(
      "lowMemoryNoEntry",
      "오픈파일럿 사용불가",
      "메모리 적음 : EON을 재부팅하세요",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.chimeDisengage, .4, 2., 3.),

  # permanent alerts
  Alert(
      "steerUnavailablePermanent",
      "LKAS 에러 : 차량을 다시 시작하고 재시도하세요",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW_LOWEST, VisualAlert.none, AudibleAlert.none, 0., 0., .2),

  Alert(
      "brakeUnavailablePermanent",
      "크루즈 에러 : 차량을 다시 시작하고 재시도하세요",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW_LOWEST, VisualAlert.none, AudibleAlert.none, 0., 0., .2),

  Alert(
      "lowSpeedLockoutPermanent",
      "크루즈 에러 : 차량을 다시 시작하고 재시도하세요",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW_LOWEST, VisualAlert.none, AudibleAlert.none, 0., 0., .2),

  Alert(
      "calibrationIncompletePermanent",
      "캘리브레이션 진행중 : ",
      "속도 유지 ",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOWEST, VisualAlert.none, AudibleAlert.none, 0., 0., .2),

  Alert(
      "invalidGiraffeToyotaPermanent",
      "giraffe 환경 지원되지않음",
      "Visit comma.ai/tg",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW_LOWEST, VisualAlert.none, AudibleAlert.none, 0., 0., .2),

  Alert(
      "internetConnectivityNeededPermanent",
      "네트워크를 연결해 주세요",
      "사용하려면 업데이트 점검이 필요합니다",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW_LOWEST, VisualAlert.none, AudibleAlert.none, 0., 0., .2),

  Alert(
      "communityFeatureDisallowedPermanent",
      "커뮤니티 기능이 탐지됨",
      "개발자설정에서 커뮤니티 기능을 사용하세요",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, 0., 0., .2),  # LOW priority to overcome Cruise Error

  Alert(
      "sensorDataInvalidPermanent",
      "EON센서에 데이터가 없음",
      "EON을 재부팅하세요",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW_LOWEST, VisualAlert.none, AudibleAlert.none, 0., 0., .2),

  Alert(
      "soundsUnavailablePermanent",
      "스피커가 없습니다",
      "EON을 재부팅하세요",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW_LOWEST, VisualAlert.none, AudibleAlert.none, 0., 0., .2),

  Alert(
      "lowMemoryPermanent",
      "메모리 매우 낮음",
      "EON을 재부팅하세요",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW_LOWEST, VisualAlert.none, AudibleAlert.none, 0., 0., .2),

  Alert(
      "carUnrecognizedPermanent",
      "대시캠 모드",
      "차량 인식 불가",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW_LOWEST, VisualAlert.none, AudibleAlert.none, 0., 0., 10.),

  Alert(
      "vehicleModelInvalid",
      "차량 매개변수 식별 실패",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOWEST, VisualAlert.steerRequired, AudibleAlert.none, .0, .0, .1),

  # offroad alerts
  Alert(
      "ldwPermanent",
      "핸들을 조작하세요",
      "차선이탈 감지됨",
      AlertStatus.userPrompt, AlertSize.mid,
      Priority.LOW, VisualAlert.steerRequired, AudibleAlert.chimePrompt, 1., 2., 3.),
]

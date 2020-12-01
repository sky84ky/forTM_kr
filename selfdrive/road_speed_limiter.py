import os
import fcntl
import signal
import json
import time

LIMIT_PATH = '/data/data/com.neokii.oproadlimit/'
LIMIT_FILE = '/data/data/com.neokii.oproadlimit/files/oproadlimit.json'

current_milli_time = lambda: int(round(time.time() * 1000))


class RoadSpeedLimiter:
  def __init__(self):
    self.json = None
    self.last_updated = 0

    try:
      os.remove(LIMIT_FILE)
    except:
      pass

    try:
      signal.signal(signal.SIGIO, self.handler)
      fd = os.open(LIMIT_PATH, os.O_RDONLY)
      fcntl.fcntl(fd, fcntl.F_SETSIG, 0)
      fcntl.fcntl(fd, fcntl.F_NOTIFY, fcntl.DN_MODIFY | fcntl.DN_CREATE | fcntl.DN_MULTISHOT)
    except Exception as ex:
      print("exception", ex)
      pass

  def handler(self, signum, frame):
    try:
      self.json = None
      if os.path.isfile(LIMIT_FILE):
        with open(LIMIT_FILE, 'r') as f:
          self.json = json.load(f)
          self.last_updated = current_milli_time()

          print(self.json)

    except Exception as ex:
      print("exception", ex)
      pass

  def get_max_speed(self, CS, v_cruise_kph):

    if current_milli_time() - self.last_updated > 1000 * 20:  # 최종 업데이트 시간이 20초가 지났으면 유효하지 않음
      return v_cruise_kph

    try:

      #car_speed_kph = CS.vEgo * CV.MS_TO_KPH

      cam_limit_speed_left_dist = self.json['cam_limit_speed_left_dist']
      cam_limit_speed = self.json['cam_limit_speed']

      section_limit_speed = self.json['section_limit_speed']
      # section_avg_speed = self.json['section_avg_speed']
      section_left_dist = self.json['section_left_dist']
      # section_left_time = self.json['section_left_time']

      # 과속카메라 남은 거리가 0보다 크고 제한속도가 50 ~ 130 이면
      if cam_limit_speed_left_dist is not None and cam_limit_speed is not None \
          and cam_limit_speed_left_dist > 0 and 50 <= cam_limit_speed <= 130:

        # 제한속도로 10초후의 거리보다 남은 거리가 작으면, 100km/h 일 경우 약 278미터
        if cam_limit_speed_left_dist < cam_limit_speed * 15:
          return min(v_cruise_kph, cam_limit_speed)

      # 구간단속중이면
      elif section_left_dist is not None and section_limit_speed is not None and \
          section_left_dist > 0:

        return min(v_cruise_kph, section_limit_speed)

    except:
      pass

    return v_cruise_kph


road_speed_limiter = None

def road_speed_limiter_get_max_speed(CS, v_cruise_kph):
  global road_speed_limiter
  if road_speed_limiter is None:
    road_speed_limiter = RoadSpeedLimiter()

  return road_speed_limiter.get_max_speed(CS, v_cruise_kph)

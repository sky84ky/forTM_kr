from selfdrive.kyd_conf import kyd_conf
import subprocess
import json
import os
BASEDIR = os.path.abspath(os.path.join(os.path.dirname(os.path.realpath(__file__)), "../"))

letters = { "a":[ "###", "# #", "###", "# #", "# #"], "b":[ "###", "# #", "###", "# #", "###"], "c":[ "###", "#", "#", "#", "###"], "d":[ "##", "# #", "# #", "# #", "##"], "e":[ "###", "#", "###", "#", "###"], "f":[ "###", "#", "###", "#", "#"], "g":[ "###", "# #", "###", "  #", "###"], "h":[ "# #", "# #", "###", "# #", "# #"], "i":[ "###", " #", " #", " #", "###"], "j":[ "###", " #", " #", " #", "##"], "k":[ "# #", "##", "#", "##", "# #"], "l":[ "#", "#", "#", "#", "###"], "m":[ "# #", "###", "###", "# #", "# #"], "n":[ "###", "# #", "# #", "# #", "# #"], "o":[ "###", "# #", "# #", "# #", "###"], "p":[ "###", "# #", "###", "#", "#"], "q":[ "###", "# #", "###", "  #", "  #"], "r":[ "###", "# #", "##", "# #", "# #"], "s":[ "###", "#", "###", "  #", "###"], "t":[ "###", " #", " #", " #", " #"], "u":[ "# #", "# #", "# #", "# #", "###"], "v":[ "# #", "# #", "# #", "# #", " #"], "w":[ "# #", "# #", "# #", "###", "###"], "x":[ "# #", " #", " #", " #", "# #"], "y":[ "# #", "# #", "###", "  #", "###"], "z":[ "###", "  #", " #", "#", "###"], " ":[ " "], "1":[ " #", "##", " #", " #", "###"], "2":[ "###", "  #", "###", "#", "###"], "3":[ "###", "  #", "###", "  #", "###"], "4":[ "#", "#", "# #", "###", "  #"], "5":[ "###", "#", "###", "  #", "###"], "6":[ "###", "#", "###", "# #", "###"], "7":[ "###", "  # ", " #", " #", "#"], "8":[ "###", "# #", "###", "# #", "###"], "9":[ "###", "# #", "###", "  #", "###"], "0":[ "###", "# #", "# #", "# #", "###"], "!":[ " # ", " # ", " # ", "   ", " # "], "?":[ "###", "  #", " ##", "   ", " # "], ".":[ "   ", "   ", "   ", "   ", " # "], "]":[ "   ", "   ", "   ", "  #", " # "], "/":[ "  #", "  #", " # ", "# ", "# "], ":":[ "   ", " # ", "   ", " # ", "   "], "@":[ "###", "# #", "## ", "#  ", "###"], "'":[ " # ", " # ", "   ", "   ", "   "], "#":[ " # ", "###", " # ", "###", " # "], "-":[ "  ", "  ","###","   ","   "] }
# letters stolen from here: http://www.stuffaboutcode.com/2013/08/raspberry-pi-minecraft-twitter.html

def print_letters(text):
    bigletters = []
    for i in text:
        bigletters.append(letters.get(i.lower(),letters[' ']))
    output = ['']*5
    for i in range(5):
        for j in bigletters:
            temp = ' '
            try:
                temp = j[i]
            except:
                pass
            temp += ' '*(5-len(temp))
            temp = temp.replace(' ',' ')
            temp = temp.replace('#','@')
            output[i] += temp
    return '\n'.join(output)
import sys, termios, tty, os, time

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

button_delay = 0.2

kyd = kyd_conf()
kyd.conf['EnableLiveTune'] = "1"
#kyd.write_config(kyd.conf)

#param = ["cameraOffset", "Kp", "Ki", "Kf", "steerRatio", "sR_boost", "sR_BP0", \
#           "sR_BP1", "sR_time", "steerRateCost"]
param = ["cameraOffset", "outerLoopGain", "innerLoopGain", "timeConstant", "actuatorEffectiveness"]

#  param = ["cameraOffset", "scale", "ki", "dc_gain", "steerRatio", "sR_boost", "sR_BP0", \
#           "sR_BP1", "sR_time", "steerRateCost"]

#param = ["Kp", "Ki", "Kf", "steerRatio", "sR_boost", "sR_BP0", \
#         "sR_BP1", "sR_time", "steerRateCost", "deadzone", "slowOnCurves", \
#         "1barBP0", "1barBP1", "1barMax", "2barBP0", "2barBP1", \
#         "2barMax", "3barBP0", "3barBP1", "3barMax", \
#         "1barHwy", "2barHwy", "3barHwy"]

j = 0
while True:
  print ("")
  print (print_letters(param[j][0:9]))
  print ("")
  print (print_letters(kyd.conf[param[j]]))
  print ("")
  print ("w: +1   1: +0.1   3: +0.05   5: +0.01   7: +0.001   r: +0.00001")
  print ("x: -1   a: -0.1   d: -0.05   g: -0.01   j: -0.001   v: -0.00001")
#  print ("w,1,3,5,7,r to incr 1,0.1,0.05,0.01,0.001,0.00001")
#  print ("x,a,d,g,j,v to decr 1,0.1,0.05,0.01,0.001,0.00001")
  print ("0: 0   L: 1")
#  print ("0 / L to make the value 0 / 1")
  print ("SPACE: next   m: prev")
  print ("z: quit")
  print ("p: reboot")

  char  = getch()
  write_json = False
  if (char == "v"):
    kyd.conf[param[j]] = str(round((float(kyd.conf[param[j]]) - 0.00001),5))
    write_json = True

  if (char == "r"):
    kyd.conf[param[j]] = str(round((float(kyd.conf[param[j]]) + 0.00001),5))
    write_json = True
    
  if (char == "7"):
    kyd.conf[param[j]] = str(round((float(kyd.conf[param[j]]) + 0.001),5))
    write_json = True

  if (char == "5"):
    kyd.conf[param[j]] = str(round((float(kyd.conf[param[j]]) + 0.01),5))
    write_json = True

  elif (char == "3"):
    kyd.conf[param[j]] = str(round((float(kyd.conf[param[j]]) + 0.05),5))
    write_json = True

  elif (char == "1"):
    kyd.conf[param[j]] = str(round((float(kyd.conf[param[j]]) + 0.1),5))
    write_json = True

  elif (char == "w"):
    kyd.conf[param[j]] = str(round((float(kyd.conf[param[j]]) + 1),5))
    write_json = True

  elif (char == "j"):
    kyd.conf[param[j]] = str(round((float(kyd.conf[param[j]]) - 0.001),5))
    write_json = True

  elif (char == "g"):
    kyd.conf[param[j]] = str(round((float(kyd.conf[param[j]]) - 0.01),5))
    write_json = True

  elif (char == "d"):
    kyd.conf[param[j]] = str(round((float(kyd.conf[param[j]]) - 0.05),5))
    write_json = True

  elif (char == "a"):
    kyd.conf[param[j]] = str(round((float(kyd.conf[param[j]]) - 0.1),5))
    write_json = True

  elif (char == "x"):
    kyd.conf[param[j]] = str(round((float(kyd.conf[param[j]]) - 1),5))
    write_json = True

  elif (char == "0"):
    kyd.conf[param[j]] = "0"
    write_json = True

  elif (char == "l"):
    kyd.conf[param[j]] = "1"
    write_json = True

  elif (char == " "):
    if j < len(param) - 1:
      j = j + 1
    else:
      j = 0

  elif (char == "m"):
    if j > 0:
      j = j - 1
    else:
      j = len(param) - 1

  elif (char == "p"):
    print ("")
    print ("system will reboot. continue?(y/n)")
    char2 = getch()
    if (char2 == "y"):
      os.system('LD_LIBRARY_PATH="" svc power reboot')
    else:
      process.kill()

  elif (char == "z"):
    process.kill()
    break


  if float(kyd.conf['EnableLiveTune']) != 1 and float(kyd.conf['EnableLiveTune']) != 0:
    kyd.conf['EnableLiveTune'] = "1"

  if write_json:
    kyd.write_config(kyd.conf)

  time.sleep(button_delay)

else:
  process.kill()

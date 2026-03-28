from lerobot.robots.aloha_mini_follower import AlohaMiniFollowerConfig, AlohaMiniFollower
import time
import sys
import math
from AlohaMiniRobot import *
import select
import termios
import tty

def kbhit():
    return sys.stdin in select.select([sys.stdin], [], [], 0)[0]

def getch():
    return sys.stdin.read(1)

fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)
tty.setcbreak(fd)

LyKinematics = AlohaMiniKinematics()

config = AlohaMiniFollowerConfig(
    port="/dev/ttyACM1",
    id="my_awesome_follower_arm",
)

follower = AlohaMiniFollower(config)
follower.connect(calibrate=False)

try:
    while True:
        # print("running...")
        for i in range(0,80):
            b, s, e, w, r, g = LyKinematics.inverse_kinematics(def_x-50, def_y, def_z-100, (80-i)*0.01, 0, 0)

            follower.send_action(
            {
                "shoulder_pan.pos": int(b),
                "shoulder_lift.pos": int(s),
                "elbow_flex.pos": int(e),
                "wrist_flex.pos": int(w),
                "wrist_roll.pos": int(r),
                "gripper.pos": int(g),
            })
            time.sleep(0.02)
        time.sleep(1)
        for i in range(0,80):
            b, s, e, w, r, g = LyKinematics.inverse_kinematics(def_x-50, def_y, def_z-100, i*0.01, 0, 0)

            follower.send_action(
            {
                "shoulder_pan.pos": int(b),
                "shoulder_lift.pos": int(s),
                "elbow_flex.pos": int(e),
                "wrist_flex.pos": int(w),
                "wrist_roll.pos": int(r),
                "gripper.pos": int(g),
            })
            time.sleep(0.02)
            
        time.sleep(1)

        if kbhit():
            key = getch()
            if key == 's':
                print("Stop!")
                break
finally:
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    follower.disconnect()
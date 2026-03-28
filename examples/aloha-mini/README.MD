# aloha_mini
## simple use aloha_mini arm

Only change the end effector pitch

```bash
python3 AlohaMini_arm_ctrl.py
```
input 's' or 'ctrl + c' stop run

<span style="color:red">
Please note that at the end of the program, the robotic arm will become disabled. Please support it in advance and ensure protective measures are in place.
</span>

code context 

```bash
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
```

use this function when the stepper motor misses steps

  ```bash
  follower.stepper_init()
  ```

use this function when the stepper motor misses steps

  ```bash
  b, s, e, w, r, g = AMKinematics.inverse_kinematics(x, y, z, w, r, g)
  ```
  Calculate inverse kinematics for aloha mini robotic arm
  
  Parameters:

      x: End effector x coordinate mm
      y: End effector y coordinate mm
      z: End effector z coordinate mm
      w: End effector pitch rad
      r: End effector yaw rad
      g: End effector gripper rad
      
  Returns:
  
      [shoulder_pan,shoulder_lift,elbow_flex,wrist_flex,wrist_roll,gripper]
      pos in degrees

## prepare
```bash
cp -r aloha_mini_follower/ ~/.cache/huggingface/lerobot/calibration/robots/
```

## calibrate
aloha_mini_follower

```bash
lerobot-calibrate \
  --robot.type=aloha_mini_follower \
  --robot.port=/dev/ttyACM0 \
  --robot.id=my_awesome_follower_arm
```

so101_leader
```bash
lerobot-calibrate \
  --teleop.type=so101_leader \
  --teleop.port=/dev/ttyACM1 \
  --teleop.id=my_awesome_leader_arm
```
## teleoperate

```bash
lerobot-teleoperate \
  --robot.type=aloha_mini_follower \
  --robot.port=/dev/ttyACM0 \
  --robot.cameras="{}" \
  --robot.id=my_awesome_follower_arm \
  --teleop.type=so101_leader \
  --teleop.port=/dev/ttyACM1 \
  --teleop.id=my_awesome_leader_arm \
  --display_data=false
```
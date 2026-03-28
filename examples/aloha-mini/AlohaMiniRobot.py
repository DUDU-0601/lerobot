import math
import numpy as np
from typing import List, Union, Tuple

SERVO_ONE_CIRCLE = 4096
SERVO_RAD = 6.2831854

STEPPER_ONE_CIRCLE = 3200
RATIO = 72/15
STEPPER_RAD = 6.2831854

"""
# # =================================
# # Arm linkage Config - SO101
# # =================================
# #          	  
# #             -------L3A-----------O==L2B===\
# #             |                    ^       ||
# #            L3B                   |       ||
# #  ----L4-----|              ELBOW_JOINT   ||
# #                                         L2A
# #                                          ||
# #                                          ||
# #                                          ||
# #                        SHOULDER_JOINT -> OO < - ZERO
# #                                          ||--L1--[O] <- BASE_JOINT

# l1 = 31.1569
# l2A = 112.3221
# l2B = 28.897
# l2 = math.sqrt(l2A * l2A + l2B * l2B)
# t2rad = math.atan2(l2B, l2A)
# l3A = 134.4873
# l3B = -4.1052
# l3 = math.sqrt(l3A * l3A + l3B * l3B)
# t3rad = math.atan2(l3B, l3A)
# l4 = 165.8818

# def_x = l1 + l2B + l3A + l4
# def_y = 0
# def_z = l2A - l3B

# # coefficient for bus servos
# rad_to_step_coefficient = SERVO_ONE_CIRCLE / SERVO_RAD


# # coefficient for bus servos
# rad_to_step_coefficient_stepper = STEPPER_ONE_CIRCLE * RATIO / STEPPER_RAD

"""

# =================================
# Arm linkage Config - Stepper-A
# =================================
#          	  
#             -------L3A-----------O==L2B===\
#             |                    ^       ||
#            L3B                   |       ||
#  ----L4-----|              ELBOW_JOINT   ||
#                                         L2A
#                                          ||
#                                          ||
#                                          ||
#                        SHOULDER_JOINT -> OO < - ZERO
#                                          ||--L1--[O] <- BASE_JOINT

l1 = 28.8042
l2A = 200
l2B = 0
l2 = math.sqrt(l2A * l2A + l2B * l2B)
t2rad = math.atan2(l2B, l2A)
l3A = 183.4981
l3B = 0
l3 = math.sqrt(l3A * l3A + l3B * l3B)
t3rad = math.atan2(l3B, l3A)
l4 = 166.

def_x = l1 + l2B + l3A + l4
def_y = 0
def_z = l2A - l3B

# coefficient for bus servos
rad_to_step_coefficient = SERVO_ONE_CIRCLE / SERVO_RAD

# coefficient for bus servos
rad_to_step_coefficient_stepper = STEPPER_ONE_CIRCLE * RATIO / STEPPER_RAD

"""
# =================================
# Arm linkage Config - Stepper-B
# =================================
#          	  
#             -------L3A-----------O==L2B===\
#             |                    ^       ||
#            L3B                   |       ||
#  ----L4-----|              ELBOW_JOINT   ||
#                                         L2A
#                                          ||
#                                          ||
#                                          ||
#                        SHOULDER_JOINT -> OO < - ZERO
#                                          ||--L1--[O] <- BASE_JOINT

l1 = 28.8042
l2A = 150
l2B = 0
l2 = math.sqrt(l2A * l2A + l2B * l2B)
t2rad = math.atan2(l2B, l2A)
l3A = 133.4981
l3B = 0
l3 = math.sqrt(l3A * l3A + l3B * l3B)
t3rad = math.atan2(l3B, l3A)
l4 = 166.

def_x = l1 + l2B + l3A + l4
def_y = 0
def_z = l2A - l3B

# coefficient for bus servos
rad_to_step_coefficient = SERVO_ONE_CIRCLE / SERVO_RAD

# coefficient for bus servos
rad_to_step_coefficient_stepper = STEPPER_ONE_CIRCLE * RATIO / STEPPER_RAD
"""
def rad_to_step(rad_input: float) -> int:
    """Convert joint radians to servo steps."""
    return int(round(rad_input * rad_to_step_coefficient))

def _clamp(x: float, lo: float = -1.0, hi: float = 1.0) -> float:
    """Clamp value into [lo, hi] to avoid acos domain errors caused by float drift."""
    return max(lo, min(hi, x))

def simple_plane_ik(
    LA: float,
    LB: float,
    a: float,
    b: float,
    t2rad: float = 0.0,
    t3rad: float = 0.0,
    eps: float = 1e-6,
):
    """
    Parameters
    ----------
    LA : float
        第一段连杆长度（肩到肘）
    LB : float
        第二段连杆长度（肘到末端）
    a : float
        末端目标点 x 坐标（相对基座）
    b : float
        末端目标点 y 坐标（相对基座）
    t2rad : float, optional
        肩关节机械零位补偿（弧度），对应原 C 代码中的 t2rad
    t3rad : float, optional
        肘关节机械零位补偿（弧度），对应原 C 代码中的 t3rad
    eps : float, optional
        小阈值，用于避免除零和数值问题

    Returns
    -------
    alpha : float
        肩关节角（弧度）
    beta : float
        肘关节角（弧度）
    delta : float
        末端相对地平面的角度（弧度）
    """

    L2C = a * a + b * b
    LC = math.sqrt(L2C)

    if LC < eps:
        raise ValueError("Target is too close to the base origin; IK is singular.")

    # Check ik
    if LC > LA + LB + eps or LC < abs(LA - LB) - eps:
        raise ValueError(
            f"Target ({a}, {b}) is unreachable for link lengths LA={LA}, LB={LB}."
        )
    lam = math.atan2(b, a)
    cos_psi = _clamp((LA * LA + L2C - LB * LB) / (2.0 * LA * LC))
    psi = math.acos(cos_psi) + t2rad

    # 肩关节角 alpha
    alpha = math.pi / 2.0 - lam - psi

    cos_omega = _clamp((LB * LB + L2C - LA * LA) / (2.0 * LC * LB))
    omega = math.acos(cos_omega)

    # 肘关节角 beta
    beta = psi + omega - t3rad

    # 末端相对地平面的角度
    delta = math.pi / 2.0 - alpha - beta

    if any(math.isnan(v) for v in (alpha, beta, delta)):
        raise ValueError("IK result contains NaN.")

    return alpha, beta, delta

def rotate_end(theta, l):
    x = -l * math.cos(theta)
    y = -l * math.sin(theta)
    return x, y

def get_wrist(xA, yA, len):
    distance = math.sqrt(xA * xA + yA * yA)
    if distance - len <= 1e-6:
        return [0, 0]
    else:
        ratio = (distance - len) / distance
        return [xA * ratio, yA * ratio]

def cartesian_to_polar(xA, yA):
    """
    input a coordinate point (x, y). The function returns two values:

    The distance from this coordinate point to the origin of the coordinate system.
    The angle, in radians, between the line connecting this point and the origin 
    of the coordinate system and the positive direction of the x-axis. 
    The angle should be in the range (-π, π).
    """
    return math.sqrt(xA * xA + yA * yA), math.atan2(yA, xA)
        

class AlohaMiniKinematics:
    """
    A class to represent the kinematics of a AlohaMini robot arm.
    All public methods use degrees for input/output.
    """

    def __init__(self):
        print("init")

    def ik_ctrl(self,x, y, z, t, r, g):
        y = -y
        end_delta_x, end_delta_y = rotate_end((t-math.pi), l4)
        wrist_x, wrist_y = get_wrist(x, y, end_delta_x)
        wrist_distance, base_rad = cartesian_to_polar(wrist_x, wrist_y)
        shoulder_rad, elbow_rad, wrist_rad = simple_plane_ik(l2, l3, (wrist_distance - l1), (z + end_delta_y), t2rad, t3rad)
        return base_rad, shoulder_rad, elbow_rad, wrist_rad + t, r, g

    def inverse_kinematics(self, x, y, z, w, r, g):
        """
        Calculate inverse kinematics for lygion robotic arm
        
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
        """
        b, s, e, w, r, g = self.ik_ctrl(x, y, z, w, r, g)
        b = b*180/3.14
        s = s*180/3.14
        e = (e-1.5708)*180/3.14
        w = w*180/3.14
        r = -r*180/3.14
        g = g*180/3.14

        return [b, s, e, w, r, g]
    
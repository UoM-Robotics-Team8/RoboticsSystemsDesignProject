import sys

from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import numpy as np
from numpy import sin, cos, arctan2, arccos, pi
from scipy.optimize import minimize



### Define Link Transformations ###
"""
DH Parameters

Link | theta         | d(mm) |    a     | alpha
_____________________________________________
1    | q1            | 0.093 | 0        | -pi/2
2    | q2 - offset   |  0    | 0.10595  | 0
3    | q3 + offset   |  0    | 0.1      | 0
4    | q4            |  0    | 0.113575 | 0
"""

### Trying to Use RoboticsToolbox ###

# class PX100(DHRobot):

#     def __init__(self):
#         super().__init__(
#             [
#                 RevoluteDH(a=0, alpha=-np.pi/2, d=0.093, offset=0),
#                 RevoluteDH(a=0.10595, alpha=0, d=0, offset=1.234127782),
#                 RevoluteDH(a=0.1, alpha=0, d=0, offset=-1.234127782),
#                 RevoluteDH(a=0.113575, alpha=0, d=0, offset=0)
#             ], name="px100"
        # )

def forward_kinematics(q1,q2,q3,q4):
    
    ### RViz DH Paramters ###
    alpha1 = -pi/2
    l1 = 0.0931
    lh = 0.1059481005
    l3 = 0.1
    l4 = 0.113575
    offset = 1.234127782

    ### Docs DH Paramters ###
    # alpha1 = -pi/2
    # l1 = 0.08945
    # lh = 0.10595
    # l3 = 0.1
    # l4 = 0.113575
    # offset = 1.234127782
    
    
    H_waist_shoulder = np.array([
            [cos(q1), -sin(q1), 0.0, 0.0],
            [sin(q1),  cos(q1), 0.0, 0.0],
            [    0.0,      0.0, 1.0, 0.0],
            [    0.0,      0.0, 0.0, 1.0]
        ]) @ np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0,  l1],
            [0.0, 0.0, 0.0, 1.0]
        ]) @ np.array([
            [1.0,         0.0,          0.0, 0.0],
            [0.0, cos(alpha1), -sin(alpha1), 0.0],
            [0.0, sin(alpha1),  cos(alpha1), 0.0],
            [0.0,         0.0,          0.0, 1.0]
        ])

    H_shoulder_elbow = np.array([
            [cos(q2 - offset), -sin(q2 - offset), 0.0, 0.0],
            [sin(q2 - offset),  cos(q2 - offset), 0.0, 0.0],
            [             0.0,               0.0, 1.0, 0.0],
            [             0.0,               0.0, 0.0, 1.0]
        ]) @ np.array([
            [1.0, 0.0, 0.0,  lh],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ])

    H_elbow_wrist = np.array([
            [cos(q3 + offset), -sin(q3 + offset), 0.0, 0.0],
            [sin(q3 + offset),  cos(q3 + offset), 0.0, 0.0],
            [             0.0,               0.0, 1.0, 0.0],
            [             0.0,               0.0, 0.0, 1.0]
        ]) @ np.array([
            [1.0, 0.0, 0.0,  l3],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ])

    H_wrist_ee = np.array([
            [cos(q4), -sin(q4), 0.0, 0.0],
            [sin(q4),  cos(q4), 0.0, 0.0],
            [    0.0,      0.0, 1.0, 0.0],
            [    0.0,      0.0, 0.0, 1.0]
        ]) @ np.array([
            [1.0, 0.0, 0.0,  l4],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ])

    H_ee = H_waist_shoulder @ H_shoulder_elbow @ H_elbow_wrist @ H_wrist_ee
    # roll = np.arctan2(H_ee[2, 1], H_ee[2, 2])  # atan2(R32, R33)
    pitch = np.arcsin(-H_ee[2, 0])          # arcsin(-R31)
    x, y, z, phi = H_ee[0,3], H_ee[1,3], H_ee[2,3], pitch 
    return np.array([x, y, z, phi])

# def inverse_kinematics(x,y,z,phi):

#     ## DH Parameters

#     alpha1 = -pi/2
#     l1 = 0.0931
#     lh = 0.1059481005
#     l3 = 0.1
#     l4 = 0.113575
#     offset = 1.234127782

#     # Joint limits (Min, Max)
#     joint_limits = {
#         "Waist": (-pi, pi),                      # Joint 1 (q1)
#         "Shoulder": (-1.9373, 1.8675),           # Joint 2 (q2)
#         "Elbow": (-2.1118, 1.6057),              # Joint 3 (q3)
#         "Wrist_Angle": (-1.7453, 2.1468)         # Joint 4 (q4)
#     }

#     x3 = x - l4*cos(phi)
#     z3 = z - l4*sin(phi)

#     c3 = (x3**2 + z3**2 - lh**2 - l3**2) / (2 * lh * l3)

#     k1 = lh + l3 * c3

#     s1 = -np.sqrt(1 - (c3**2))
#     k2 = l3 * s1


#     theta3 = arctan2(s1,c3)
#     theta2 = arctan2(z3,x3) - arctan2(k2,k1)
#     theta4 = phi - theta3 - theta2

#     q1 = arctan2(y,x)
#     q2 = pi/2 - theta2 - offset
#     q3 = pi/2 - theta3 - offset
#     q4 = theta4

#     # q1 = arctan2(y,x)
#     # q2 = theta2
#     # q3 = theta3
#     # q4 = theta4

#     # Check joint limits
#     joint_angles = [q1, q2, q3, q4]
#     joint_names = ["Waist", "Shoulder", "Elbow", "Wrist_Angle"]

#     for i, (joint, angle) in enumerate(zip(joint_names, joint_angles)):
#         min_limit, max_limit = joint_limits[joint]
#         if not (min_limit <= angle <= max_limit):
#             print(f"⚠️ Warning: {joint} angle {np.degrees(angle):.2f}° is out of range ({np.degrees(min_limit):.2f}° to {np.degrees(max_limit):.2f}°).")

    
#     return joint_angles

# def ik_different(x,y,z,pitch):
    ## Link Lengths

    l1 = 0.0931
    lm = 0.035
    l2 = 0.1
    lh = np.sqrt(lm**2 + l2**2)
    l3 = 0.1
    l4 = 0.113575

    # Joint limits (Min, Max)
    joint_limits = {
        "Waist": (-pi, pi),                      # Joint 1 (q1)
        "Shoulder": (-1.9373, 1.8675),           # Joint 2 (q2)
        "Elbow": (-2.1118, 1.6057),              # Joint 3 (q3)
        "Wrist_Angle": (-1.7453, 2.1468)         # Joint 4 (q4)
    }

    q1 = arctan2(y,x)

    x3 = x - l4*cos(pitch)
    y3 = y - l4*sin(q1)*cos(pitch)
    z3 = z - l4*sin(pitch)

    r = np.sqrt(x3**2 + y3**2)
    h = z3 - l1
    c = np.sqrt(r**2 + h**2)

    theta_a = arctan2(np.sqrt(x**2 + y**2), z)
    phi = arccos((c**2 - l3**2 - lh**2) / (-2 * l3 * lh))
    alpha = arccos((l3**2 - lh**2 - c**2) / (-2 * lh * c))
    beta = arctan2(lm,l2)
    gamma = arctan2(h,r)
    psi = pi/2 - beta

    ### Elbow up ### 

    q2 = pi - alpha - beta - gamma
    q3 = pi - psi - phi
    q4 = theta_a - q2 - q3 - pi/2

    ### Elbow down ###
    
    # q2 = pi/2 - (gamma - alpha + beta)
    # q3 = -pi + (phi - psi)
    # q4 = theta_a - q2 - q3 - pi/2

    # Check joint limits
    joint_angles = [q1, q2, q3, q4]
    joint_names = ["Waist", "Shoulder", "Elbow", "Wrist_Angle"]

    for i, (joint, angle) in enumerate(zip(joint_names, joint_angles)):
        min_limit, max_limit = joint_limits[joint]
        if not (min_limit <= angle <= max_limit):
            print(f"⚠️ Warning: {joint} angle {np.degrees(angle):.2f}° is out of range ({np.degrees(min_limit):.2f}° to {np.degrees(max_limit):.2f}°).")

    
    return joint_angles

def main():
    bot = InterbotixManipulatorXS(
        robot_model='px100',
        group_name='arm',
        gripper_name='gripper',
    )

    robot_startup()

    #     # Instantiate the robot
    # px100 = PX100_Robot()
    # print(px100)
    
    ## Testing Kinematics ###
    q1 = 0
    q2 = np.radians(0)
    q3 = np.radians(0)
    q4 = np.radians(90)
    joint_positions = [q1,q2,q3,q4]

    # joint_positions = ik_correct(x=0.135, y=0.0, z=0.079525, pitch=pi/2)
    # q1, q2, q3, q4 = joint_positions
    # print(joint_positions)
    bot.arm.set_joint_positions([0.0, 0.0, 0.0, 0.0])
    bot.arm.set_joint_positions(joint_positions)
    # bot.arm.set_joint_positions([0.0, 0.0, 0.0, 0.0])
    # x, y, z, phi = forward_kinematics(q1, q2, q3, q4)
    # print(f"End-effector position: x={x:.4f}, y={y:.4f}, z={z:.4f}, phi={np.degrees(phi):.1f}")

    robot_shutdown()


if __name__ == '__main__':
    main()

import sys

from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import numpy as np
from numpy import sin, cos, pi
from time import sleep

"""
    testing
"""


def main():
    bot = InterbotixManipulatorXS(
        robot_model='px100',
        group_name='arm',
        gripper_name='gripper',
    )

    robot_startup()

    if (bot.arm.group_info.num_joints < 4):
        bot.get_node().logfatal(
            'This demo requires the robot to have at least 4 joints!'
        )
        robot_shutdown()
        sys.exit()
        
    ######## TESTING AREA ########
    x, y, z = 0.2, 0.0, 0.1
    r = np.hypot(x, y)  # Equivalent to sqrt(x^2 + y^2)
    theta = np.arctan2(y, x)  # Angle in radians
    x_traj = r - 0.2458
    z_traj = z - 0.193

    # bot.gripper.release(2.0)
    # bot.gripper.grasp(2.0)

    ### Best Practices ###

    bot.arm.go_to_home_pose()
    bot.arm.set_single_joint_position(joint_name='waist', position=theta)
    bot.gripper.release(1.0)
    bot.arm.set_ee_cartesian_trajectory(x=x_traj, z=z_traj)
    bot.gripper.grasp(1.0)
    bot.arm.set_ee_cartesian_trajectory(x=-x_traj, z=-z_traj)
    bot.arm.set_single_joint_position(joint_name='waist', position=pi/2)
    bot.arm.set_single_joint_position(joint_name="wrist_angle", position=pi/2)
    bot.arm.set_ee_cartesian_trajectory(z=-0.07)
    bot.gripper.release(1.0)
    bot.arm.set_ee_cartesian_trajectory(z=0.07)
    bot.arm.go_to_home_pose()
    bot.arm.go_to_sleep_pose()

    #######################
    # bot.arm.set_single_joint_position(joint_name='wrist_angle', position=0.0)
    # bot.arm.set_ee_pose_components(pitch=0.157)
    # bot.arm.go_to_home_pose()
    # joint_positions = [0.0, 0.0, 0.0, 0.0]
    # bot.arm.set_joint_positions([0.0, pi/2, -pi/2, 0.0])
    # bot.arm.set_joint_positions([pi/4,pi/6,pi/6,pi/6])
    # bot.arm.set_joint_positions([0.0, 0.0, 0.0, 0.0])
    # bot.arm.go_to_sleep_pose()

    # sleep(2)
    # bot.gripper.grasp()
    # bot.arm.set_single_joint_position(joint_name='wrist_angle', position=0.0)
    # # bot.arm.set_ee_cartesian_trajectory(x=-r, z=-z)
    # bot.arm.set_single_joint_position(joint_name='waist', position=0.0)
    # bot.arm.go_to_home_pose()
    # bot.arm.go_to_sleep_pose()
    ###### z=0.0931 shoulder height
    ###### x=0.15 perfect 90 deg angles ... wrist and shoulder in line
    ### what trajectories are valid???
    # bot.arm.go_to_home_pose()
    # bot.arm.set_ee_cartesian_trajectory(x=0.02)
    # bot.arm.set_ee_pose_components(x=0.32, z=0.0931)
    # bot.arm.set_ee_cartesian_trajectory

    # ### try to grasp block
    # H_base_block = np.array([[1, 0, 0,  0.1],
    #                          [0, 1, 0,  0.0],
    #                          [0, 0, 1, -0.1],
    #                          [0, 0, 0,    1]])
    # H_base_ee = bot.arm.get_ee_pose()
    # H_ee_base = np.linalg.inv(H_base_ee)
    # H_ee_block = H_ee_base @ H_base_block
    # bot.arm.go_to_home_pose()

    # bot.arm.set_ee_cartesian_trajectory()



    ##############################

   ### Giving coordinates w.r.t base_link
    # bot.arm.set_ee_pose_components(x=0.1, y=0.0, z=-0.2, pitch=pi/4)
    # sleep(2)

#     ### Giving transformation from base_link to ee_gripper_link
#     T_sd = np.array([
#         [ cos(pi), -sin(pi), 0.0, -0.248575],
#         [-sin(pi),  cos(pi), 0.0,       0.0],
#         [     0.0,      0.0, 1.0,    0.1931],
#         [     0.0,      0.0, 0.0,       1.0]
#     ])
#     bot.arm.set_ee_pose_matrix(T_sd)
#     sleep(2)

#     bot.arm.go_to_home_pose()
#     sleep(2)

    ### Giving linear displacement of end-effector
    # bot.arm.set_ee_cartesian_trajectory(x=0.15)

    ### Commanding a single joint
    # bot.arm.set_single_joint_position(joint_name="wrist_angle", position=-1.57)

    ### Upright then spin
    # bot.arm.set_ee_pose_components(x=0.0, y=0.0, z=0.4105, pitch=-pi/2)
    # bot.arm.set_ee_cartesian_trajectory(roll=pi/4)


    # print(bot.arm.get_joint_positions())
    # sleep(2)
    # bot.arm.go_to_sleep_pose()
    
    robot_shutdown()


if __name__ == '__main__':
    main()

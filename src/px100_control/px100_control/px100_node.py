import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_srvs.srv import Trigger
import numpy as np
from numpy import pi
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

class PX100Node(Node):
    """A ROS2 Node that represents the manipulator."""

    def __init__(self):
        super().__init__("px100_node")

        self.arm_on_server = self.create_service(
            srv_type=Trigger,
            srv_name="/leo_rover/turn_arm_on",
            callback=self.turn_on_callback
        )

        self.arm_off_server = self.create_service(
            srv_type=Trigger,
            srv_name="/leo_rover/turn_arm_off",
            callback=self.turn_off_callback
        )

        self.pose_subscriber = self.create_subscription(
            msg_type=Point,
            topic="/camera/block_poses",
            callback=self.pose_callback,
            qos_profile=1
        )

        self.turned_on: bool = True
        self.bot = InterbotixManipulatorXS(
                robot_model='px100',
                group_name='arm',
                gripper_name='gripper',
            )
        robot_startup()


    def turn_on_callback(self,
                         request: Trigger.Request,
                         response: Trigger.Response
                         ) -> Trigger.Response:
        if not self.turned_on:
            # Turn Robot On
            robot_startup()
            self.turned_on = True

            # Assign to response
            response.success = True
            response.message = "Robot has been turned on"

        else:
            # Assign to response
            response.success = False
            response.message = "Robot is already on"

        # Display INFO
        self.get_logger().info(response.message)

        return response

    def turn_off_callback(self,
                         request: Trigger.Request,
                         response: Trigger.Response
                         ) -> Trigger.Response:
        if self.turned_on:
            # Turn Robot Off
            robot_shutdown()
            self.turned_on = False

            # Assign to response
            response.success = True
            response.message = "Robot has been turned off"

        else:
            # Assign to response
            response.success = False
            response.message = "Robot is already off"

        # Display INFO
        self.get_logger().info(response.message)

        return response

    def pose_callback(self, msg: Point):
        """Method called when msg is received by node"""
        if self.turned_on:
            x = msg.x
            y = msg.y
            z = msg.z
            r = np.hypot(x, y) 
            theta = np.arctan2(y, x)
            x_traj = r - 0.2458
            z_traj = z - 0.193

            ### Best Practices ###

            self.bot.arm.go_to_home_pose()
            self.bot.arm.set_single_joint_position(joint_name='waist', position=theta)
            self.bot.gripper.release()
            # self.bot.arm.set_ee_cartesian_trajectory(x=x_traj, z=z_traj)
            self.bot.arm.set_ee_pose_components(x=x, z=z, pitch=0.0, blocking=True)
            # self.bot.arm.set_single_joint_position(joint_name="wrist_angle", position=pi/2, blocking=True)


            self.bot.gripper.grasp()
            # self.bot.arm.set_ee_cartesian_trajectory(x=-x_traj, z=-z_traj)
            self.bot.arm.go_to_home_pose(moving_time=1.5, blocking=True)
            # self.bot.arm.set_single_joint_position(joint_name='waist', position=pi/2)
            # self.bot.arm.set_single_joint_position(joint_name="wrist_angle", position=pi/2)
            # self.bot.arm.set_ee_cartesian_trajectory(z=-0.07)
            # self.bot.gripper.release()
            # self.bot.arm.set_ee_cartesian_trajectory(z=0.07)
            self.bot.arm.go_to_home_pose()
            self.bot.arm.go_to_sleep_pose()
        else:
            self.get_logger().info("Robot is turned off")


def main(args=None):
    try:
        rclpy.init(args=args)

        px100_node = PX100Node()

        rclpy.spin(px100_node)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()

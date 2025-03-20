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
            srv_name="/leo_rover/pick_object",
            callback=self.pick_callback
        )

        self.arm_off_server = self.create_service(
            srv_type=Trigger,
            srv_name="/leo_rover/place_object",
            callback=self.place_callback
        )

        self.pose_subscriber = self.create_subscription(
            msg_type=Point,
            topic="/object_center_distance",
            callback=self.pose_callback,
            qos_profile=1
        )

        self.turned_on: bool = True
        self.H_base_cam = np.array([[0, 0, 1,  0.06],
                                     [1, 0, 0,   0.0],
                                     [0, 1, 0, 0.015],
                                     [0, 0, 0,     1]])
        self.H_cam_obj = np.zeros(4)
        self.H_base_obj = np.zeros(4)
        self.bot = InterbotixManipulatorXS(
                robot_model='px100',
                group_name='arm',
                gripper_name='gripper',
            )
        robot_startup()


    def pick_callback(self,
                         request: Trigger.Request,
                         response: Trigger.Response
                         ) -> Trigger.Response:
        """A method that is called to order the manipulator to pick the object."""
        if self.turned_on:
            self.H_base_obj = self.H_base_cam @ self.H_cam_obj

            x = self.H_base_obj[0, 3]
            y = self.H_base_obj[1, 3]
            z = self.H_base_obj[2, 3]
            r = np.hypot(x, y) 
            theta = np.arctan2(y, x)
            x_traj = r - 0.2458
            z_traj = z - 0.193

            ### Best Practices ###

            self.bot.arm.go_to_home_pose()
            self.bot.arm.set_single_joint_position(joint_name='waist', position=theta)
            self.bot.gripper.release()
            self.bot.arm.set_ee_pose_components(x=x, z=z, pitch=0.0, blocking=True)
            self.bot.gripper.grasp()
            self.bot.arm.go_to_home_pose(moving_time=1.5, blocking=True)
            self.bot.arm.go_to_sleep_pose()

        else:
            # Assign to response
            response.success = False
            response.message = "Robot is already on"

        # Display INFO
        self.get_logger().info(response.message)

        return response

    def place_callback(self,
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

            self.H_cam_obj = np.array([[1, 0, 0, msg.x],
                                       [0, 1, 0, msg.y],
                                       [0, 0, 1, msg.z],
                                       [0, 0, 0,     1]])
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

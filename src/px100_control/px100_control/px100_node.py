import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger
import numpy as np
from numpy import pi
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

class PX100Node(Node):
    """A ROS2 Node that represents the manipulator."""

    def __init__(self):
        super().__init__("px100_node")

        self.pick_object_server = self.create_service(
            srv_type=Trigger,
            srv_name="/leo_rover/pick_object",
            callback=self.pick_callback
        )

        self.place_object_server = self.create_service(
            srv_type=Trigger,
            srv_name="/leo_rover/drop_object",
            callback=self.drop_callback
        )

        self.pose_subscriber = self.create_subscription(
            msg_type=Float32MultiArray,
            topic="/object_center_distance",
            callback=self.pose_callback,
            qos_profile=1
        )

        self.H_base_cam = np.array([[ 0, 0, 1,  0.062],
                                    [-1, 0, 0, -0.066],  ### This is the value when camera is in position 1
                                    [ 0, 1, 0,  0.006],
                                    [ 0, 0, 0,      1]])
        
        # self.H_base_cam = np.array([[ 0, 0, 1,  0.066], ### This is the values when camera is offset (position 2)
        #                             [-1, 0, 0, -0.013],  
        #                             [ 0, 1, 0,   0.07],
        #                             [ 0, 0, 0,      1]])
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

        self.H_base_obj = self.H_base_cam @ self.H_cam_obj

        x = self.H_base_obj[0, 3] + 0.0075
        y = self.H_base_obj[1, 3]
        z = self.H_base_obj[2, 3]
        # r = np.hypot(x, y) + 0.015
        # theta = np.arctan2(y, x)
        # x_traj = r - 0.2458
        # z_traj = z - 0.193

        self.bot.arm.go_to_home_pose()
        # self.bot.arm.set_single_joint_position(joint_name='waist', position=theta)
        self.bot.gripper.release()
        self.bot.arm.set_ee_pose_components(x=x, y=y, z=z, pitch=0.0, blocking=True)
        self.bot.gripper.grasp()
        self.bot.arm.go_to_home_pose(moving_time=1.5, blocking=True)
        self.bot.arm.go_to_sleep_pose()


        # Display INFO
        response.success = True
        response.message = "Robot has picked object"
        self.get_logger().info(response.message)

        return response

    def drop_callback(self,
                         request: Trigger.Request,
                         response: Trigger.Response
                         ) -> Trigger.Response:

        self.bot.arm.go_to_home_pose()
        self.bot.arm.set_joint_positions([0.0, pi/4, -pi/4, 0.0])
        self.bot.gripper.release()
        self.bot.arm.go_to_home_pose()
        self.bot.arm.go_to_sleep_pose()

        # Assign to response
        response.success = True
        response.message = "Robot has placed object"

        # Display INFO
        self.get_logger().info(response.message)

        return response

    def pose_callback(self, msg: Point):
        """Method called when msg is received by node"""
        x_msg, y_msg, z_msg = msg.data


        self.H_cam_obj = np.array([[1, 0, 0, x_msg],
                                       [0, 1, 0, y_msg],
                                       [0, 0, 1, z_msg],
                                       [0, 0, 0,     1]])


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

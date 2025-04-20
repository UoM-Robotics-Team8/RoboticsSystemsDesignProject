import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray, Bool
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped


class ObjectDetectListener(Node):
   """Node will listen to the distance to detected object, pause m-explore
      then use Nav2 to navigate towards the detcted object""" 
   
   def __init__(self):
      super().__init__("object_detect_listener_node")

      self.object_detected: bool = False
      self.dist_msg: Float32MultiArray = None
      self.object_x: float = None
      self.object_y: float = None

      self.object_distance_subscriber = self.create_subscription(
         msg_type=Float32MultiArray,
         topic='/object_center_distance',
         callback=self.object_callback,
         qos_profile=10
      )

      self.pause_publshier = self.create_publisher(
         msg_type=Bool,
         topic='/explore/resume',
         qos_profile=1
      )

      self.nav2_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

   def object_callback(self, msg):
      self.dist_msg = msg

      self.distance_to_object()

   def distance_to_object(self):
      if self.dist_msg is None:
         self.get_logger().info("Waiting for data on /object_center_distance...")
      else:

         # pause explore, set goal pose, nav to goal pose
         self.pause_explore()
         
         self.object_x = self.dist_msg[0]
         self.object_y = self.dist_msg[1]

         self.nav_to_object()

   def pause_explore(self):
      msg = Bool()
      msg.data = False
      self.pause_publshier.publish(msg)
   
   def nav_to_object(self):
      goal = NavigateToPose()

      # set the goal pose frame
      goal.header.frame_id = "map"

      # time is needed in the header
      goal.header.stamp = self.get_clock().now().to_msg()

      # set the goal position
      goal.pose.position.x = self.object_x
      goal.pose.position.y = self.object_y
      goal.pose.position.z = 0 # 2D

      self.send_goal(goal)

   def send_goal(self, goal):
      # wait for nav2 to become available
      self.nav2_client.wait_for_server()

      # send goal to nav2 
      self.nav2_client.send_goal_async(goal)
      

def main(args=None):
      try:
         rclpy.init(args=args)

         object_detection_listener_node = ObjectDetectListener()

         rclpy.spin(object_detection_listener_node)

      except KeyboardInterrupt:
         pass
      except Exception as e:
         print(e)


if __name__ == '__main__':
   main()
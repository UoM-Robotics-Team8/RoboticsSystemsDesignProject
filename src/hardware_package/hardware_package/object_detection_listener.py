import rclpy
from rclpy.node import Node
from rclpy.task import Future
from std_msgs.msg import String, Float32MultiArray, Bool
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped


class ObjectDetectListener(Node):
   """Node will listen to the distance to detected object, pause m-explore
      then use Nav2 to navigate towards the detcted object""" 
   
   def __init__(self):
      super().__init__("object_detect_listener_node")

      self.objects_detected: int = 0
      self.no_of_objects: int = 0
      self.dist_msg: Float32MultiArray = None
      self.object_x: float = None
      self.object_y: float = None

      self.object_distance_subscriber = self.create_subscription(
         msg_type=Float32MultiArray,
         topic='/object_center_distance',
         callback=self.object_callback,
         qos_profile=10
      )

      # self.object_grabbed_subscriber = self.create_subscription(
      #    msg_type=Bool,
      #    topic='/object_grabbed',
      #    callback=self.object_grabbed_callback,
      #    qos_profile=1
      # )

      self.explore_publisher = self.create_publisher(
         msg_type=Bool,
         topic='/explore/resume',
         qos_profile=1
      )

      self.nav2_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

      self.future: Future = None

   def object_callback(self, msg):
      self.dist_msg = msg

      self.distance_to_object()

   # def object_grabbed_callback(self, msg):
   #    if msg.data == True:
   #       self.no_of_objects += 1
   #       if self.no_of_objects == 3:
   #          self.go_home()
   #       else:
   #          self.resume_explore()


   def distance_to_object(self):
      if self.dist_msg is None:
         self.get_logger().info("Waiting for data on /object_center_distance...")
      else:
         if self.objects_detected == 30:

            # pause explore, set goal pose, nav to goal pose
            self.pause_explore()
            
            self.object_x = self.dist_msg[0]
            self.object_y = self.dist_msg[1]

            self.nav_to_object()
         else:
            self.objects_detected += 1

   def pause_explore(self):
      msg = Bool()
      msg.data = False
      self.explore_publisher.publish(msg)

   def resume_explore(self):
      msg = Bool()
      msg.data = True
      self.explore_publisher.publish(msg)
   
   def nav_to_object(self):
      goal = NavigateToPose.Goal()
      
      # expects this message type
      goal.pose = PoseStamped()

      # set the goal pose frame
      goal.pose.header.frame_id = "map"

      # time is needed in the header
      goal.pose.header.stamp = self.get_clock().now().to_msg()

      # set the goal position (quaternion)
      goal.pose.pose.position.x = self.object_x
      goal.pose.pose.position.y = self.object_y
      goal.pose.pose.position.z = 0 # 2D
      goal.pose.pose.orientation.w = 1.0 # facing forward

      self.send_goal(goal)

   def send_goal(self, goal):
      # wait for nav2 to become available
      self.nav2_client.wait_for_server()

      # send goal to nav2 
      self.future = self.nav2_client.send_goal_async(goal)

      self.future.add_done_callback(self.navigation_result_callback)

   def navigation_result_callback(self, future: Future):
      result = future.result()

      if result.status == 3:
         # navigation successful, so pick up object
         self.get_logger().info("Navigation Succeeded!!")
      else:
         # navigation unsuccesful - bad!
         self.get_logger().info("ERROR! NAVIGATION FAILED")
         # resume explore and hope it'll not fail next time
         self.resume_explore()

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
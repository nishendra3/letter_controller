"""
ROS2 node for drawing letter N with the iRobot Create 3
Uses four coordinates for the trajectory of the letter N
Controller used : PID ( simple-pid library )

author: Nishendra Singh
date: 2023-05-25
"""
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from irobot_create_msgs.action import Dock, Undock, NavigateToPosition
import math
import time
from simple_pid import PID




class TrajectoryFollower(Node):

    def __init__(self):
        super().__init__('letter_controller')


        # publisher for the cmd_vel topic
        self.pub = self.create_publisher(Twist, '/diffdrive_controller/cmd_vel_unstamped', 10)

        # subscriber for the odometry
        self.sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # action clients for docking, undocking and homing
        self.dock_client = ActionClient(self, Dock, '/dock')
        self.undock_client = ActionClient(self, Undock, '/undock')
        self.homing_client = ActionClient(self, NavigateToPosition, '/navigate_to_position')

        # The current pose & home of the robot
        self.current_pose = [0, 0, 0]
        self.home = (-0.2, 0.0)  
        
        # Trajectory for the letter N
        self.waypoints = [ (-1, 0), (-2, 0), (-1, 1), (-2, 1)]  

        # Addition of home position to the trajectory
        self.waypoints.append(self.home)  
        
        # Initialize PID controllers
        self.distance_pid = PID(0.7, 0.7, 0.05, setpoint=0)
        self.angle_pid    = PID(1., 0.2, 0.05, setpoint=0)


        self.get_logger().info("init successful")
    

    def odom_callback(self, msg):
        """
        Input: Odometry message
        Output: None
        Use: Decides the next waypoint to navigate to
        """
        self.current_pose[0] = msg.pose.pose.position.x
        self.current_pose[1] = msg.pose.pose.position.y
        self.current_pose[2] = euler_from_quaternion(msg.pose.pose.orientation.x,
                                                        msg.pose.pose.orientation.y,
                                                        msg.pose.pose.orientation.z,
                                                        msg.pose.pose.orientation.w
                                                        )[2]

        if len(self.waypoints) > 0:
            # Navigate to the next waypoint
            self.navigate_to_next_waypoint()
        else:
            # Reached the end of the trajectory, start homing & docking
            self.get_logger().info("Reached the end of the trajectory")
            f1 = self.home_robot()
            time.sleep(4)
            f2 = self.dock_robot()
            rclpy.spin_until_future_complete(self, f2)



    def navigate_to_next_waypoint(self):
        """
        Input: None
        Output: None
        Use: Navigates to the next waypoint in the trajectory using PID
        """

        
        self.get_logger().info("navigating to the next waypoint")

        # error calculations
        goal = self.waypoints[0]
        error_x = goal[0] - self.current_pose[0]
        error_y = goal[1] - self.current_pose[1]
        distance = math.hypot(error_x, error_y)
        target_angle = math.atan2(error_y, error_x)
        angle_error = self.normalize_angle(target_angle - self.current_pose[2])

        # Twist message from the controller
        control_msg = Twist()
        control_msg.linear.x = self.distance_pid(distance)
        control_msg.angular.z = self.angle_pid(angle_error)
        self.pub.publish(control_msg)
        
        # Check if waypoint is reached
        if distance < 0.2:  # Close enough, go to the next waypoint
            self.waypoints.pop(0)
            self.distance_pid.reset()
            self.angle_pid.reset()
            return

    def normalize_angle(self,angle):
        """Normalize an angle to [-pi, pi)"""
        return (angle + math.pi) % (2 * math.pi) - math.pi


    # Util functions for docking, undocking and homing

    def undock_robot(self):
        if not self.undock_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available!')
            return

        goal_msg = Undock.Goal()
        return self.undock_client.send_goal_async(goal_msg)

    def dock_robot(self):
        if not self.dock_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available!')
            return

        goal_msg = Dock.Goal()
        return self.dock_client.send_goal_async(goal_msg)
    
    def home_robot(self):
        if not self.homing_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available!')
            return
        
        goal_msg = NavigateToPosition.Goal()
        goal_msg.goal_pose.pose.position.x = self.home[0]
        goal_msg.goal_pose.pose.position.y = self.home[1]
        goal_msg.goal_pose.pose.orientation.w = 0.8
        return self.homing_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryFollower()

    # Undock the robot
    future = node.undock_robot()
    rclpy.spin_until_future_complete(node, future)
    time.sleep(5)
    
    # Start the trajectory
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

# helper functions
def euler_from_quaternion(x, y, z, w):
        """
        source: https://automaticaddison.com/how-to-convert-a-quaternion-to-a-euler-angle/
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians
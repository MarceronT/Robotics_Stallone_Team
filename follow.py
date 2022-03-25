import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from geometry_msgs.msg import PoseStamped

from robot_navigator import BasicNavigator, NavigationResult

import time
import sys
import math

class Follow(Node):

    def __init__(self):
        super().__init__('follow')

        self.time_movement = time.time()
        self.state = "FOLLOW_ME" # ETAT = "FOLLOW_ME" / "GO_BACK"
        self.dist_max = 0.60 # 1.0
        self.dist_min = 0.20 # 0.20
        self.x_centre = (self.dist_max + self.dist_min) / 2
        self.y_centre = 0
        self.v_lin = 0
        self.v_rot = 0
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        if (self.state == "FOLLOW_ME"):
            msg_twist = Twist()
            sum_x = 0
            sum_y = 0
            i = -1
            tour = 0
            for r in msg.ranges :
                i += 1
                theta = msg.angle_min +msg.angle_increment * i
                theta_degrees = math.degrees(theta)
                if (330 < theta_degrees or theta_degrees < 30) :
                    if (r > self.dist_min and r < self.dist_max):
                        print('Theta ={}, r ={}'.format(theta_degrees,r))
                        x = r * math.cos(theta)
                        y = r * math.sin(theta)
                        sum_x += x
                        sum_y += y 
                        tour += 1
            if (tour == 0):
                self.v_lin = 0.0
                self.v_rot = 0.0
                # print(time.time())
                # print(self.time_movement)
                if ((time.time() - self.time_movement) > 3):
                    self.state = "GO_BACK"
                    self.nav_auto() #appelle ma fonction goto
            else:
                self.time_movement = time.time()
                # self.timer = True
                # print('sum_x ={}, sum_y ={}'.format(self.sum_x, self.sum_y))      
                bari_x = sum_x / tour
                bari_y = sum_y / tour
                # print('bari_x ={}, bari_y ={}'.format(bari_x, bari_y))

                delta_x = bari_x - self.x_centre
                delta_y = bari_y

                self.v_lin = delta_x * 0.5
                print(self.v_lin)
                self.v_rot = delta_y * 1.8
                print(self.v_rot)

            msg_twist.linear.x = self.v_lin # should be between 0.01 and 0.1
            msg_twist.angular.z = self.v_rot
            self.publisher.publish(msg_twist)

    def nav_auto(self):
        navigator = BasicNavigator()

        # Set our demo's initial pose
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = 3.45
        initial_pose.pose.position.y = 2.15
        initial_pose.pose.orientation.z = 1.0
        initial_pose.pose.orientation.w = 0.0
        navigator.setInitialPose(initial_pose)

        navigator.waitUntilNav2Active()

        navigator.changeMap('/home/thomas/Documents/Robotique/map_autonav.yaml')

        # You may use the navigator to clear or obtain costmaps
        # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
        # global_costmap = navigator.getGlobalCostmap()
        # local_costmap = navigator.getLocalCostmap()

        # Go to our demos first goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = -2.0
        goal_pose.pose.position.y = -0.5
        goal_pose.pose.orientation.w = 1.0

        # sanity check a valid path exists
        # path = navigator.getPath(initial_pose, goal_pose)

        navigator.goToPose(goal_pose)

        i = 0
        while not navigator.isNavComplete():
            ################################################
            #
            # Implement some code here for your application!
            #
            ################################################

            # Do something with the feedback
            i = i + 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Estimated time of arrival: ' + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                    + ' seconds.')

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    navigator.cancelNav()

                # Some navigation request change to demo preemption
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
                    goal_pose.pose.position.x = -3.0
                    navigator.goToPose(goal_pose)

        # Do something depending on the return code
        result = navigator.getResult()
        if result == NavigationResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == NavigationResult.CANCELED:
            print('Goal was canceled!')
        elif result == NavigationResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')

        navigator.lifecycleShutdown()

        self.state = "FOLLOW_ME"

def follow():
    rclpy.init()

    minimal_subscriber = Follow()
    print("Dans le follow")
    rclpy.spin(minimal_subscriber)
    # rclpy.spin_once(minimal_subscriber)
    
    print("Après le follow")
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    
    rclpy.shutdown()


# if __name__ == '__main__':
#     follow()
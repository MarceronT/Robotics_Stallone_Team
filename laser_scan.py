import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

import math

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.dist_max = 2.5
        self.dist_min = 0.5
        # self.sum_x = 0
        # self.sum_y = 0
        self.x_centre = (self.dist_max - self.dist_min) / 2
        self.y_centre = 0
        self.v_lin = 0
        self.v_rot = 0
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg): 
        self.get_logger().info("time_increment : {}".format(msg.time_increment))
        self.get_logger().info("scan_time : {}".format(msg.scan_time))
        msg_twist = Twist()
        sum_x = 0
        sum_y = 0
        i = -1
        tour = 0
        for r in msg.ranges :
            i += 1
            theta = msg.angle_min +msg.angle_increment * i
            theta_degrees = math.degrees(theta)
            if (300 < theta_degrees or theta_degrees < 60) :
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
        else:
            # print('sum_x ={}, sum_y ={}'.format(self.sum_x, self.sum_y))      
            bari_x = sum_x / tour
            bari_y = sum_y / tour
            # print('bari_x ={}, bari_y ={}'.format(bari_x, bari_y))

            delta_x = bari_x - self.x_centre
            delta_y = bari_y

            self.v_lin = delta_x * 1.2
            self.v_rot = delta_y * 2

        msg_twist.linear.x = self.v_lin
        msg_twist.angular.z = self.v_rot
        self.publisher.publish(msg_twist)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
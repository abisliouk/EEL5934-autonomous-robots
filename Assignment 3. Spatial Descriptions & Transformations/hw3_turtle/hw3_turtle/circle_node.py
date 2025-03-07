# /usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CircleDraw(Node):
    def __init__(self):
        super().__init__('circle_node')
        self.get_logger().info('Circle Draw Node Started')
        self.vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.vel = Twist()
        self.radius = 2.0
        self.linear_velocity = 1.0 # Constant forward speed

        # Set angular velocity based on the radius
        self.angular_velocity = self.linear_velocity / self.radius

        # Set initial movement direction
        self.vel.linear.x = self.linear_velocity
        self.vel.angular.z = self.angular_velocity

        # create a time with a publish rate
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.publish)

    def publish(self):
        # Publish velocity command to move the turtle
        self.vel_pub.publish(self.vel)

def main(args=None):
    rclpy.init(args=args)

    node = CircleDraw()
    
    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
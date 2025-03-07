# /usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SpiralDraw(Node):
    def __init__(self):
        super().__init__('spiral_node')
        self.get_logger().info('Spiral Draw Node Started')
        self.vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.move_spiral)
        self.linear_speed = 0.2
        self.angular_speed = 2.0

    def move_spiral(self):
        msg = Twist()
        msg.linear.x = self.linear_speed
        msg.angular.z = self.angular_speed 
        self.vel_pub.publish(msg)
        self.linear_speed += 0.02
        
        self.get_logger().info(f'Coordinates: linear.x={msg.linear.x}, angular.z={msg.angular.z}')

def main(args=None):
    rclpy.init(args=args)

    node = SpiralDraw()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
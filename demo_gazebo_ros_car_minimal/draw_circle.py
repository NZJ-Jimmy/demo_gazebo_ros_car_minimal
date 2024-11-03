from shutil import move
import rclpy
import rclpy.clock
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time

class CircleDraw(Node):
    def __init__(self):
        super().__init__('circleDraw')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.speed = 2.0
        self.radius = 5.0
        self.timer_period_sec = 0.1
        while True:
            self.move_circle()

    def move_circle(self):
        angle_covered = 0.0
        while angle_covered < 2 * math.pi:
            msg = Twist()
            msg.linear.x = self.speed
            msg.angular.z = self.speed / self.radius  # ω = v / r
            self.publisher_.publish(msg)
            time.sleep(self.timer_period_sec)
            angle_covered += self.speed / self.radius * self.timer_period_sec
        self.stop_move()
        
    def stop_move(self):
        msg = Twist()
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CircleDraw()
    rclpy.spin(node)
    node.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()

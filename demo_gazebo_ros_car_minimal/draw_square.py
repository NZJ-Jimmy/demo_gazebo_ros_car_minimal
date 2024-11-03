from re import T
import rclpy
import rclpy.clock
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time

class SquareTurtle(Node):
    def __init__(self):
        super().__init__('square_turtle')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.speed = 1.0
        self.sideLength = 5.0
        self.timer_period_sec = 0.1
        while True:
            self.draw_square()

    def draw_square(self):
        for i in range(4):
            self.draw_line()
            self.turn_left()
            
    def draw_line(self):
        length_covered = 0.0
        while length_covered < self.sideLength:
            msg = Twist()
            msg.linear.x = self.speed
            self.publisher_.publish(msg)
            time.sleep(self.timer_period_sec)
            length_covered += self.speed * self.timer_period_sec
        self.stop_move()
    
    def turn_left(self):
        angle_covered = 0.0
        while angle_covered < math.pi / 2:
            msg = Twist()
            msg.angular.z = self.speed
            self.publisher_.publish(msg)
            time.sleep(self.timer_period_sec)
            angle_covered += self.speed * self.timer_period_sec
        self.stop_move()
        
    def stop_move(self):
        msg = Twist()
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SquareTurtle()
    rclpy.spin(node)
    node.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()

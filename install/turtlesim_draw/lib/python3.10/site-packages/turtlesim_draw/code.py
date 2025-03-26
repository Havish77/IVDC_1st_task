import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen, Spawn, Kill
import math

class TurtleDraw(Node):
    def __init__(self):
        super().__init__('turtle_draw')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        self.spawn_client = self.create_client(Spawn, 'spawn')
        self.kill_client = self.create_client(Kill, 'kill')    
        self.current_x = 0
        self.current_y = 0
        self.current_theta = 0.0  # radians, facing right
        self.pen_is_down = True

    def set_pen(self, r=255, g=255, b=255, width=2, off=False):
        while not self.pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for pen service...')
        req = SetPen.Request()
        req.r = r
        req.g = g
        req.b = b
        req.width = width
        req.off = off
        self.pen_is_down = not off
        self.pen_client.call_async(req)

    def draw_line(self, end_x, end_y, speed=1.0):
        """Draw line from current position to specified end coordinates"""
        dx = end_x - self.current_x
        dy = end_y - self.current_y
        distance = math.sqrt(math.pow(dx,2)+math.pow(dy,2))
        target_angle = math.atan2(dy, dx)
        
        # Calculate required rotation
        angle_diff = target_angle - self.current_theta
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi  # Normalize to [-π, π]
        
        # Rotate to face target
        if abs(angle_diff) > 1e-3:
            self._rotate(angle_diff)
        
        # Move forward
        self._move_straight(distance, speed)
        
        # Update position
        self.current_x = end_x
        self.current_y = end_y

    def draw_circle(self, center_x, center_y, radius=1, speed=1.0):
        """Draws a circle around the given center point with the specified radius."""
        dx = center_x - self.current_x
        dy = center_y - self.current_y
        
        # Check if starting at circumference
        distance_to_center = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))
        if abs(distance_to_center - radius) < 1e-3:  # Allow small floating-point error
            angle_diff = math.atan2(dy, dx) - self.current_theta
            angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi  # Normalize angle
            
            # Rotate to tangent direction
            angle_diff2 = math.atan2(-dx, dy) - math.atan2(dy, dx)
            if abs(angle_diff) > 1e-3:
                self._rotate(angle_diff)
            if abs(angle_diff2) > 1e-3:
                self._rotate(angle_diff2)
            
            # Start moving in circular path
            msg = Twist()
            msg.linear.x = speed
            msg.angular.z = speed / radius  # Angular speed for circular motion
            start_angle = self.current_theta
            
            while abs(self.current_theta - start_angle) < (2 * math.pi):
                self.publisher_.publish(msg)
                rclpy.spin_once(self)
                self.current_theta += msg.angular.z * 0.1  # Approximate angle update
                self.current_theta = (self.current_theta + math.pi) % (2 * math.pi) - math.pi
            
            # Stop movement after completing the circle
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            
            # Update position to approximate start point
            self.current_x = center_x + radius * math.cos(self.current_theta)
            self.current_y = center_y + radius * math.sin(self.current_theta)
        
        else:
            self.get_logger().info("Not starting at the circumference. Adjusting position...")
    
    def move_turtle(self, end_x, end_y, speed=1.0):
        self.set_pen(off=True)  
        self.draw_line(end_x, end_y, speed)
        self.set_pen(off=False)  

    def _rotate(self, angle):
        msg = Twist()
        msg.linear.x = 0.0  
        msg.angular.z = 0.1 
        target_angle = self.current_theta + angle
        target_angle = (target_angle + math.pi) % (2 * math.pi) - math.pi  
        while abs(self.current_theta - target_angle) > 1e-2:  
            self.publisher_.publish(msg)
            rclpy.spin_once(self)
        msg.angular.z = 0.0
        self.publisher_.publish(msg)

    def _move_straight(self, distance, speed):
        msg = Twist()
        msg.linear.x = speed
        time = distance/speed
        start_time = self.get_clock().now().seconds_nanoseconds()[0]
        while  (self.get_clock().now().seconds_nanoseconds()[0] - start_time) < time:
            self.publisher_.publish(msg)
            rclpy.spin_once(self)
        msg.linear.x =0
        self.publisher_.publish(msg)

    def _face_angle(self,angle):
        intial_angle = self.current_theta
        angle_diff = angle - intial_angle
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi
        self._rotate(angle_diff)

    def pen_up(self):
        self.set_pen(off=True)

    def pen_down(self):
        self.set_pen(off=False)

def main(args=None):
    rclpy.init(args=args)
    turtle_draw = TurtleDraw()
    # steps to draw the figure
    turtle_draw.move_turtle(5,7)
    turtle_draw.draw_line(7,5)
    turtle_draw.draw_line(5,3)
    turtle_draw.draw_line(3,5)
    turtle_draw.draw_line(7,5)
    turtle_draw.move_turtle(6,6)
    turtle_draw.draw_line(8,8)
    turtle_draw.move_turtle(4,4)
    turtle_draw.draw_line(2,2)
    turtle_draw.move_turtle(4,6)
    turtle_draw.draw_line(2,8)
    turtle_draw.move_turtle(6,4)
    turtle_draw.draw_line(8,2)
    turtle_draw.move_turtle(8,3)
    turtle_draw.draw_circle(8,2)
    turtle_draw.move_turtle(2,1)
    turtle_draw.draw_circle(2,2)
    turtle_draw.move_turtle(2,9)
    turtle_draw.draw_circle(2,8)
    turtle_draw.move_turtle(2,1)
    turtle_draw.draw_circle(2,2)

    

    turtle_draw.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
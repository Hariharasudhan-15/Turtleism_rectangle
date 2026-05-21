import math
import time
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from turtlesim.msg import Pose


class PerfectRectangleDrawer(Node):

    def __init__(self):
        super().__init__("perfect_rectangle_drawer")

        self.velocity_publisher = self.create_publisher(
            Twist, "/turtle1/cmd_vel", 10
        )
        self.pose_subscriber = self.create_subscription(
            Pose, "/turtle1/pose", self.pose_callback, 10
        )

        self.current_pose = None

        # Wait until we actually get pose data from Turtlesim
        while rclpy.ok() and self.current_pose is None:
            rclpy.spin_once(self)

        # Draw a perfect rectangle (Width = 3.5, Height = 2.0)
        self.draw_rectangle(width=3.5, height=2.0)

    def pose_callback(self, data):
        self.current_pose = data

    def normalize_angle(self, angle):
        """Keeps angles strictly between -pi and pi to prevent radian confusion"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def move_straight(self, distance, speed=1.5):
        """Moves forward accurately by evaluating distance to target"""
        msg = Twist()
        msg.linear.x = float(speed)

        start_x = self.current_pose.x
        start_y = self.current_pose.y
        distance_traveled = 0.0

        # Tolerable error limit (0.02 units)
        while (distance - distance_traveled) > 0.02 and rclpy.ok():
            self.velocity_publisher.publish(msg)
            rclpy.spin_once(self)
            distance_traveled = math.sqrt(
                (self.current_pose.x - start_x) ** 2
                + (self.current_pose.y - start_y) ** 2
            )

        self.stop_turtle()

    def turn_90_degrees_left(self):
        """Uses Proportional (P) control to slow down near 90 deg and snap perfectly"""
        msg = Twist()

        # Calculate exact target angle based on current direction
        target_theta = self.normalize_angle(self.current_pose.theta + (math.pi / 2.0))
        
        kp = 4.0  # Proportional gain (tweak higher to turn faster, lower to prevent overshoot)
        error = self.normalize_angle(target_theta - self.current_pose.theta)

        # Turn until the error is practically zero (less than ~0.3 degrees)
        while abs(error) > 0.005 and rclpy.ok():
            # Speed dynamically slows down as error approaches zero
            msg.angular.z = kp * error
            
            # Clamp maximum turning speed so it doesn't go crazy
            if msg.angular.z > 1.5: msg.angular.z = 1.5
            if msg.angular.z < -1.5: msg.angular.z = -1.5

            self.velocity_publisher.publish(msg)
            rclpy.spin_once(self)
            
            error = self.normalize_angle(target_theta - self.current_pose.theta)

        self.stop_turtle()

    def stop_turtle(self):
        """Instantly kills velocity and waits for simulation physics to catch up"""
        stop_msg = Twist()
        self.velocity_publisher.publish(stop_msg)
        time.sleep(0.4)  # Generous delay so it stops completely before the next step

    def draw_rectangle(self, width, height):
        self.get_logger().info("Executing clean, exact rectangle script...")
        for _ in range(2):
            self.move_straight(width)
            self.turn_90_degrees_left()
            self.move_straight(height)
            self.turn_90_degrees_left()
        self.get_logger().info("Rectangle completed successfully!")


def main(args=None):
    rclpy.init(args=args)
    node = PerfectRectangleDrawer()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
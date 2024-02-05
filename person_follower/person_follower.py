import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class PersonFollower(Node):

    def __init__(self):
        super().__init__('person_follower')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.prev_ranges = []  # Almacenamos las lecturas anteriores del láser

    def detect_person(self, ranges):
        # Tu lógica para determinar si hay una persona en los datos del láser
        # Ajusta esto según tu entorno
        min_range = min(ranges)
        person_threshold = 1.0  # Ajusta este umbral según tu entorno
        return min_range < person_threshold

    def has_frontal_variation(self, ranges, threshold=0.1):
        # Compara las lecturas anteriores con las actuales y devuelve True si hay variación significativa
        if not self.prev_ranges:
            self.prev_ranges = ranges
            return False

        for prev, current in zip(self.prev_ranges, ranges):
            if abs(current - prev) > threshold:
                return True

        return False

    def listener_callback(self, input_msg):
        angle_min = input_msg.angle_min
        angle_max = input_msg.angle_max
        angle_increment = input_msg.angle_increment
        ranges = input_msg.ranges

        # Check if a person is detected and there is a frontal variation
        if self.detect_person(ranges) and self.has_frontal_variation(ranges):
            # Find the index of the minimum range (closest obstacle)
            min_range_index = ranges.index(min(ranges))

            # Calculate the angle to the person
            angle_to_person = angle_min + min_range_index * angle_increment

            # Set linear and angular velocities
            vx = 0.2  # Constant forward velocity

            # Use the angle_to_person directly for angular velocity
            wz = 0.6 * angle_to_person

            # Adjust linear and angular velocities for a more responsive behavior
            vx *= 1.5
            wz *= 2.0
        else:
            # If no person is detected or no frontal variation, stop the robot
            vx = 0.0
            wz = 0.0

        # Almacenamos las lecturas actuales para la próxima iteración
        self.prev_ranges = ranges

        # Create Twist message and publish
        output_msg = Twist()
        output_msg.linear.x = vx
        output_msg.angular.z = wz
        self.publisher_.publish(output_msg)

def main(args=None):
    rclpy.init(args=args)
    person_follower = PersonFollower()
    rclpy.spin(person_follower)
    person_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

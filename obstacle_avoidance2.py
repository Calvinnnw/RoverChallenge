import rclpy
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

class ActuatorControl(Node):
    def __init__(self):
        super().__init__('micro_ros_arduino_node')

        # Enhanced QoS profile for faster and more reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=100  # Increase depth for higher buffering
        )

        # Publisher for actuator control
        self.actuator_publisher = self.create_publisher(Twist, '/actuator', qos_profile)

        # Subscriber for ultrasonic sensor readings with optimized QoS
        self.sensor_subscription = self.create_subscription(
            Int32,
            '/sensor/ultrasonic',
            self.sensor_callback,
            qos_profile
        )

        # Initialize actuator values
        self.servo_angle = 30.0  # Initial servo angle (degrees)
        self.motor_speed = 130.0  # Initial motor speed (arbitrary unit)
        self.obstacle_distance_threshold = 50  # Threshold distance to stop motor (cm)

        # Timer to publish actuator data periodically
        timer_period = 0.1  # Timer period in seconds (10 Hz frequency)
        self.timer = self.create_timer(timer_period, self.publish_actuator_data)

        # State to track whether reversing or turning
        self.reversing = False
        self.turning = False
        self.count = 0

    def publish_actuator_data(self):
        # Create and publish Twist message for actuator control
        twist_msg = Twist()
        if self.reversing:
            twist_msg.linear.x = -130.0  # Reverse speed
            twist_msg.angular.z = 30.0  # Keep straight while reversing
            self.get_logger().info('Reversing...')
        elif self.turning:
            twist_msg.linear.x = 130.0  # Stop linear motion while turning
            twist_msg.angular.z = 60.0  # Turn to the right (adjust angle as needed)
            self.get_logger().info('Turning right...')
        else:
            twist_msg.linear.x = self.motor_speed  # Forward speed
            twist_msg.angular.z = self.servo_angle  # Normal servo angle
        
        self.actuator_publisher.publish(twist_msg)

    def sensor_callback(self, msg):
        distance = msg.data
        self.get_logger().info(f'Ultrasonic sensor reading: {distance} cm')

        if distance < self.obstacle_distance_threshold:
            self.get_logger().warn(f'Obstacle detected! Initiating reverse and turn. Distance: {distance} cm')
            self.reversing = True
        
        if self.reversing:
            if distance >= self.obstacle_distance_threshold:
                self.reversing = False
                self.turning = True
         
        if self.turning:
            if distance < self.obstacle_distance_threshold:
                self.turning = False
                self.count = 0
            else:
                self.count += 1
                if self.count == 50:
                    self.turning = False
                    self.count = 0

def main(args=None):
    rclpy.init(args=args)

    actuator_control = ActuatorControl()

    try:
        rclpy.spin(actuator_control)
    except KeyboardInterrupt:
        pass
    finally:
        pass

if __name__ == '__main__':
    main()


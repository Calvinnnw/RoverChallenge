import rclpy
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
        self.motor_speed = 140.0  # Initial motor speed (arbitrary unit)
        self.obstacle_distance_threshold = 60  # Threshold distance to stop motor (cm)

        # Timer to publish actuator data periodically
        timer_period = 0.1  # Timer period in seconds (10 Hz frequency)
        self.timer = self.create_timer(timer_period, self.publish_actuator_data)

    def publish_actuator_data(self):
        # Create and publish Twist message for actuator control
        twist_msg = Twist()
        twist_msg.angular.z = self.servo_angle  # Map servo angle to angular.z
        twist_msg.linear.x = self.motor_speed  # Map motor speed to linear.x
        self.actuator_publisher.publish(twist_msg)
        self.get_logger().info(f'Published actuator data: Servo={self.servo_angle} degrees, Motor={self.motor_speed} units')

    def sensor_callback(self, msg):
        # Log the received ultrasonic sensor reading
        distance = msg.data
        self.get_logger().info(f'Ultrasonic sensor reading: {distance} cm')

        # Stop motor if obstacle is detected within threshold distance
        if distance < self.obstacle_distance_threshold:
            self.motor_speed = 0.0
            self.get_logger().warn(f'Obstacle detected! Stopping motor. Distance: {distance} cm')
        else:
            self.motor_speed = 140.0  # Resume motor speed if no obstacle

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


import rclpy
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from my_custom_msgs.msg import Ultrasonic, Actuator


class ActuatorControl(Node):
    def __init__(self):
        super().__init__('actuator_control_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=100
        )

        self.actuator_publisher = self.create_publisher(Actuator, '/actuator', qos_profile)
        self.sensor_subscription = self.create_subscription(
            Ultrasonic,
            '/sensor/ultrasonic',
            self.sensor_callback,
            qos_profile
        )

        self.current_sensor_data = Ultrasonic()

        # THRESHOLD DATA
        self.safe_threshold = 40
        self.minimum_threshold = 10

        # MOVEMENT / SPEED
        self.high_speed = 150
        self.slow_speed = 120
        self.stop_speed = 0
        self.reverse_speed = -120

        # ANGLE
        self.forward_angle = 30  # Straight
        self.turn_left_angle = 60
        self.turn_right_angle = 0

        # TURNING DATA
        self.turning_time = 1

        # TIMER DATA
        self.state_start_time = time.time()
        self.timer = self.create_timer(0.1, self.publish_actuator_data)
        self.last_safety_state = None

        # self.state = "FORWARD"

    def publish_actuator_data(self):
        now = time.time()
        
        msg = Actuator()
        front = self.current_sensor_data.front
        left = self.current_sensor_data.left
        right = self.current_sensor_data.right

        # Safety state decision
        if front < self.minimum_threshold or left < self.minimum_threshold or right < self.minimum_threshold:
            safety_state = "danger"
        elif front < self.safe_threshold or left < self.safe_threshold or right < self.safe_threshold:
            safety_state = "barely_safe"
        else:
            safety_state = "safe"

        self.get_logger().info(f"Sensors — F: {front}, L: {left}, R: {right} | State: {safety_state}")

        if safety_state != self.last_safety_state:
            self.state_start_time = now
            self.last_safety_state = safety_state
        
        elapsed = now - self.state_start_time


        # === LOGIC BASED ON SAFETY STATE ===

        # SAFETY STATE = "SAFE"
        if safety_state == "safe":

            if front >= self.safe_threshold + 20:
                msg.rpm = self.high_speed
                msg.angle = self.forward_angle
            else:
                if right < self.safe_threshold + 10:
                    if elapsed < self.turning_time:
                        msg.rpm = self.high_speed
                        msg.angle = self.turn_left_angle
                elif left < self.safe_threshold + 10:
                    if elapsed < self.turning_time:
                        msg.rpm = self.high_speed
                        msg.angle = self.turn_right_angle
                else:
                    msg.rpm = self.high_speed
                    msg.angle = self.forward_angle
                
        # SAFETY STATE = "BARELY_SAFE"
        elif safety_state == "barely_safe":

            #If front detect object
            if front < self.safe_threshold:
                #both side detect object
                #then compare
                if right < self.safe_threshold + 10 and left < self.safe_threshold + 10:
                    if right < left:
                        if elapsed < self.turning_time:
                            msg.rpm = self.slow_speed
                            msg.angle = self.turn_left_angle
                    else:
                        if elapsed < self.turning_time:
                            msg.rpm = self.slow_speed
                            msg.angle = self.turn_right_angle
                #either side object detects object
                #then compare
                elif right < self.safe_threshold + 10 or left < self.safe_threshold +10:
                    if right < self.safe_threshold + 10:
                        if elapsed < self.turning_time:
                            msg.rpm = self.slow_speed
                            msg.angle = self.turn_left_angle
                    elif left < self.safe_threshold + 10:
                        if elapsed < self.turning_time:
                            msg.rpm = self.slow_speed
                            msg.angle = self.turn_right_angle
            
            #if side sensor detect object
            elif right < self.safe_threshold or left < self.safe_threshold:

                #If front is safe
                if front > self.safe_threshold:
                    #both side have object
                    #then go forward
                    if right < self.safe_threshold and left < self.safe_threshold:
                        if elapsed < self.turning_time:
                            msg.rpm = self.slow_speed
                            msg.angle = self.forward_angle
                    #only right side have object
                    #then turn left
                    elif right < self.safe_threshold and left > self.safe_threshold:
                        if elapsed < self.turning_time:
                            msg.rpm = self.slow_speed
                            msg.angle = self.turn_left_angle
                    #only left side have object
                    #then turn right
                    elif left < self.safe_threshold and right > self.safe_threshold:
                        if elapsed < self.turning_time:
                            msg.rpm = self.slow_speed
                            msg.angle = self.turn_right_angle
                
                #If front not safe
                elif front < self.safe_threshold:
                    if right < self.safe_threshold and left > self.safe_threshold:
                        if elapsed < self.turning_time:
                            msg.angle = self.turn_left_angle
                            msg.rpm = self.slow_speed
                    elif left < self.safe_threshold and right > self.safe_threshold:
                        if elapsed < self.turning_time:
                            msg.angle = self.turn_right_angle
                            msg.rpm = self.slow_speed
                    elif right < self.safe_threshold and left < self.safe_threshold:
                        if right < left:
                            if elapsed < self.turning_time:
                                msg.angle = self.turn_left_angle
                                msg.rpm = self.slow_speed
                        else:
                            if elapsed < self.turning_time:
                                msg.angle = self.turn_right_angle
                                msg.rpm = self.slow_speed
                    else:
                        if elapsed < self.turning_time:
                            msg.angle = self.turn_right_angle
                            msg.rpm = self.slow_speed
            
            else:
                # Reset to safe
                msg.rpm = self.high_speed
                msg.angle = self.forward_angle

        # SAFETY STATE = "DANGER"
        elif safety_state == "danger":

            #stop for 1 sec
            if elapsed < 0.5:
                msg.rpm = self.stop_speed
            else:
                if front < self.minimum_threshold:
                    if elapsed < 2.5:
                        #TODO reverse for 2 sec
                        #angle straight
                        msg.rpm = self.reverse_speed
                        msg.angle = self.forward_angle
                    else:
                        msg.rpm = self.stop_speed
                        msg.angle = self.forward_angle

                elif front > self.minimum_threshold:
                    if right < self.minimum_threshold:
                        if elapsed < 1:
                            #TODO reverse to the right for 0.5 sec
                            msg.angle = self.turn_right_angle
                            msg.rpm = self.reverse_speed
                        else:
                            msg.rpm = self.stop_speed
                            msg.angle = self.forward_angle
                    elif left < self.minimum_threshold:
                        if elapsed < 1:
                            #TODO reverse to the left for 0.5 sec
                            msg.angle = self.turn_left_angle
                            msg.rpm = self.reverse_speed
                        else:
                            msg.rpm = self.stop_speed
                            msg.angle = self.forward_angle


        self.actuator_publisher.publish(msg)

    def sensor_callback(self, msg: Ultrasonic):
        self.current_sensor_data = msg

    def stop_robot(self):
        stop_msg = Actuator()
        stop_msg.rpm = 0
        stop_msg.angle = self.forward_angle
        self.actuator_publisher.publish(stop_msg)
        self.get_logger().info("Robot stopped safely on shutdown.")


def main(args=None):
    rclpy.init(args=args)
    node = ActuatorControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown requested by user.")
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

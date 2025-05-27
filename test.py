import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PubSubNode(Node):

    def __init__(self):
        super().__init__('pub_sub_node')
        
        # Publisher
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(0.5, self.publish_message)
        self.i = 0
        
        # Subscriber
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.subscribe_message,
            10)
        self.subscription  # prevent unused variable warning

    def publish_message(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

    def subscribe_message(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = PubSubNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
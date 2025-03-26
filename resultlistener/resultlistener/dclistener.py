import rclpy
from rclpy.node import Node
from detectionresult.msg import DetectionResult

class dclistener(Node):
    def __init__(self):
        super().__init__('dclistener')
        self.subscription = self.create_subscription(
            DetectionResult,
            '/detection_result',
            self.listener_callback,
            10)
        self.subscription  # 防止未使用警告

    def listener_callback(self, msg):
        self.get_logger().info(
            f'Received Detection: Class ID: {msg.class_id}, Confidence: {msg.confidence}, '
            f'Center: ({msg.center.x}, {msg.center.y}, {msg.center.z})'
        )

def main(args=None):
    rclpy.init(args=args)
    node = dclistener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
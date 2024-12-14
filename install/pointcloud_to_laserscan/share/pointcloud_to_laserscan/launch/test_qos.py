import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, LaserScan

class QoSCheckerNode(Node):
    def __init__(self):
        super().__init__('qos_checker_node')
        
        # Create a reliable QoS profile
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Subscribe to input point cloud
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            '/cloud_registered',
            self.cloud_callback,
            qos_profile=reliable_qos
        )
        
        # Publish laser scan
        self.scan_pub = self.create_publisher(
            LaserScan,
            '/scan',
            qos_profile=reliable_qos
        )
    
    def cloud_callback(self, msg):
        self.get_logger().info('Received point cloud with QoS profile')
        # Additional processing can be added here

def main(args=None):
    rclpy.init(args=args)
    node = QoSCheckerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
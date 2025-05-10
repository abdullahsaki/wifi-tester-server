import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import threading

class ROS2Client(Node):
    def __init__(self):
        super().__init__('django_ros2_client')
        self.position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        
        # Create publisher for cmd_vel
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Create subscriber for odometry
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.position_callback,
            10)
        
        print('ROS2 Client başlatıldı')

    def position_callback(self, msg):
        self.position['x'] = msg.pose.pose.position.x
        self.position['y'] = msg.pose.pose.position.y
        self.position['z'] = msg.pose.pose.position.z

    def publish_cmd_vel(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.publisher.publish(msg)

    def get_position(self):
        return self.position

# Global ROS2 client instance
ros2_client = None
ros2_thread = None

def init_ros2():
    global ros2_client, ros2_thread
    
    if ros2_client is None:
        rclpy.init()
        ros2_client = ROS2Client()
        
        def spin():
            rclpy.spin(ros2_client)
        
        ros2_thread = threading.Thread(target=spin)
        ros2_thread.daemon = True
        ros2_thread.start()
        
        return True
    return False

def get_position():
    if ros2_client:
        return ros2_client.position
    return {'x': 0.0, 'y': 0.0, 'z': 0.0} 
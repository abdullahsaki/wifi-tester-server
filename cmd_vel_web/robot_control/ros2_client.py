import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
import numpy as np
import threading
import time
import cv2


class ROS2Client(Node):
    """ROS2 iletişimi için ana sınıf."""
    
    def __init__(self):
        """ROS2 istemcisini başlatır."""
        super().__init__('web_interface_node')
        self.position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        
        # Publisher ve subscriber'ları oluştur
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.position_callback,
            10
        )
        
        self.image_subscriber = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        self.latest_image = None
        self.image_lock = threading.Lock()
        
        self.get_logger().info('ROS2 Client Düğümü başlatıldı')

    def position_callback(self, msg):
        """Pozisyon bilgisini günceller."""
        self.position['x'] = msg.pose.pose.position.x
        self.position['y'] = msg.pose.pose.position.y
        self.position['z'] = msg.pose.pose.position.z

    def publish_cmd_vel(self, linear_x, angular_z):
        """Hız komutunu yayınlar."""
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.publisher.publish(msg)

    def image_callback(self, msg):
        """Kamera görüntüsünü işler."""
        try:
            # ROS Image mesajını numpy dizisine dönüştür
            # Görüntünün RGB8 formatında olduğunu varsay
            height = msg.height
            width = msg.width
            data = np.array(msg.data, dtype=np.uint8)
            data = data.reshape((height, width, 3))  # RGB için 3 kanal
            
            with self.image_lock:
                self.latest_image = data
        except Exception as e:
            self.get_logger().error(f'Görüntü işleme hatası: {str(e)}')

    def get_latest_image(self):
        """En son görüntüyü JPEG formatında döndürür."""
        with self.image_lock:
            if self.latest_image is not None:
                try:
                    # Numpy dizisini JPEG baytlarına dönüştür
                    _, buffer = cv2.imencode('.jpg', self.latest_image)
                    return buffer.tobytes()
                except Exception as e:
                    self.get_logger().error(f'Görüntü kodlama hatası: {str(e)}')
                    return None
        return None

    def get_position(self):
        """Mevcut pozisyonu döndürür."""
        return self.position


class ROS2Manager:
    """ROS2 bağlantısını yöneten sınıf."""
    
    def __init__(self):
        """ROS2 yöneticisini başlatır."""
        self.client = None
        self.thread = None
    
    def initialize(self):
        """ROS2 bağlantısını başlatır."""
        if self.client is None:
            rclpy.init()
            self.client = ROS2Client()
            
            def spin():
                rclpy.spin(self.client)
            
            self.thread = threading.Thread(target=spin)
            self.thread.daemon = True
            self.thread.start()
            
            return True
        return False
    
    def get_position(self):
        """Robot pozisyonunu döndürür."""
        if self.client:
            return self.client.position
        return {'x': 0.0, 'y': 0.0, 'z': 0.0}


# Global ROS2 yönetici örneği
ros2_manager = ROS2Manager()
ros2_client = None


def init_ros2():
    """ROS2 bağlantısını başlatır ve istemciyi döndürür."""
    global ros2_client
    
    if ros2_client is None:
        if ros2_manager.initialize():
            ros2_client = ros2_manager.client
            return True
    return False


def get_position():
    """Robot pozisyonunu döndürür."""
    return ros2_manager.get_position() 
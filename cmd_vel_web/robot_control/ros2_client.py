import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, Imu, JointState
from tf2_msgs.msg import TFMessage
import numpy as np
import threading
import time
import cv2
import logging

# Loglama yapılandırması
logging.basicConfig(
    level=logging.WARNING,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)

logger = logging.getLogger('ROS2Client')

class ROS2Client(Node):
    """ROS2 iletişimi için ana sınıf."""
    
    def __init__(self):
        """ROS2 istemcisini başlatır."""
        try:
            super().__init__('web_interface_node')
            self.position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
            
            # Publisher ve subscriber'ları oluştur
            self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
            
            # cmd_vel topic'ini dinle
            self.cmd_vel_subscription = self.create_subscription(
                Twist,
                '/cmd_vel',
                self.cmd_vel_callback,
                10
            )
            
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
            
            # Topic subscribers
            self.create_subscription(Imu, '/imu', self.imu_callback, 10)
            
            # Yeni subscriber'lar
            self.joint_states_subscription = self.create_subscription(
                JointState,
                '/joint_states',
                self.joint_states_callback,
                10
            )
            
            self.tf_subscription = self.create_subscription(
                TFMessage,
                '/tf',
                self.tf_callback,
                10
            )
            
            self.latest_image = None
            self.image_lock = threading.Lock()
            
            # Store latest sensor data
            self.latest_imu_data = None
            self.latest_joint_states = None
            self.latest_tf_data = None
            
            # Store latest velocity data
            self.latest_velocity = {
                'linear_x': 0.0,
                'angular_z': 0.0
            }
            
            logger.info('ROS2 Client Düğümü başlatıldı')
            
            # Bağlantı durumunu kontrol et
            self.check_connection()
            
        except Exception as e:
            logger.error(f'ROS2 Client başlatma hatası: {str(e)}')
            raise

    def check_connection(self):
        """ROS2 bağlantısını kontrol eder."""
        try:
            # Topic listesini al
            topics = self.get_topic_names_and_types()
            
            # Gerekli topic'lerin varlığını kontrol et
            required_topics = ['/cmd_vel', '/odom', '/imu', '/joint_states', '/tf']
            available_topics = [topic[0] for topic in topics]
            
            missing_topics = [topic for topic in required_topics if topic not in available_topics]
            
            if missing_topics:
                logger.warning(f'Eksik topic\'ler: {missing_topics}')
            else:
                logger.info('Tüm gerekli topic\'ler mevcut')
                
            return len(missing_topics) == 0
            
        except Exception as e:
            logger.error(f'Bağlantı kontrolü hatası: {str(e)}')
            return False

    def cmd_vel_callback(self, msg):
        """cmd_vel topic'inden gelen hız verilerini günceller."""
        self.latest_velocity = {
            'linear_x': msg.linear.x,
            'angular_z': msg.angular.z
        }
        self.get_logger().debug(f'Yeni hız değerleri: linear_x={msg.linear.x:.2f}, angular_z={msg.angular.z:.2f}')

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
            # Görüntünün BGR8 formatında olduğunu varsay
            height = msg.height
            width = msg.width
            data = np.array(msg.data, dtype=np.uint8)
            data = data.reshape((height, width, 3))  # BGR için 3 kanal
            
            # BGR'den RGB'ye dönüştür
            data = cv2.cvtColor(data, cv2.COLOR_BGR2RGB)
            
            with self.image_lock:
                self.latest_image = data
        except Exception as e:
            self.get_logger().error(f'Görüntü işleme hatası: {str(e)}')

    def imu_callback(self, msg):
        """IMU verilerini işler."""
        try:
            logger.info("IMU verisi alındı")
            self.latest_imu_data = {
                'linear_acceleration': {
                    'x': msg.linear_acceleration.x,
                    'y': msg.linear_acceleration.y,
                    'z': msg.linear_acceleration.z
                },
                'angular_velocity': {
                    'x': msg.angular_velocity.x,
                    'y': msg.angular_velocity.y,
                    'z': msg.angular_velocity.z
                },
                'orientation': {
                    'x': msg.orientation.x,
                    'y': msg.orientation.y,
                    'z': msg.orientation.z,
                    'w': msg.orientation.w
                },
                'timestamp': msg.header.stamp.sec
            }
            logger.info(f'IMU verileri güncellendi: {self.latest_imu_data}')
        except Exception as e:
            logger.error(f'IMU veri işleme hatası: {str(e)}')

    def joint_states_callback(self, msg):
        """Eklem durumlarını günceller."""
        try:
            logger.info(f"Eklem durumu verisi alındı: {len(msg.name)} eklem")
            self.latest_joint_states = {
                'names': msg.name,
                'positions': msg.position,
                'velocities': msg.velocity,
                'efforts': msg.effort,
                'timestamp': msg.header.stamp.sec
            }
            logger.info(f'Eklem durumları güncellendi: {self.latest_joint_states}')
        except Exception as e:
            logger.error(f'Eklem durumu işleme hatası: {str(e)}')

    def tf_callback(self, msg):
        """TF verilerini günceller."""
        try:
            logger.info(f"TF verisi alındı: {len(msg.transforms)} transform")
            self.latest_tf_data = {
                transform.child_frame_id: {
                    'translation': {
                        'x': transform.transform.translation.x,
                        'y': transform.transform.translation.y,
                        'z': transform.transform.translation.z
                    },
                    'rotation': {
                        'x': transform.transform.rotation.x,
                        'y': transform.transform.rotation.y,
                        'z': transform.transform.rotation.z,
                        'w': transform.transform.rotation.w
                    }
                }
                for transform in msg.transforms
            }
            logger.info(f'TF verileri güncellendi: {self.latest_tf_data}')
        except Exception as e:
            logger.error(f'TF veri işleme hatası: {str(e)}')

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

    def get_latest_imu_data(self):
        """En son IMU verilerini döndürür."""
        return self.latest_imu_data

    def get_latest_velocity(self):
        """En son hız değerlerini döndürür."""
        return self.latest_velocity

    def get_latest_joint_states(self):
        """En son eklem durumlarını döndürür."""
        return self.latest_joint_states

    def get_latest_tf_data(self):
        """En son TF verilerini döndürür."""
        return self.latest_tf_data


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
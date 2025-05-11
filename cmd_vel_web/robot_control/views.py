# robot_control/views.py
from django.shortcuts import render
from django.http import JsonResponse, HttpResponse
from django.views.decorators.csrf import csrf_exempt
from .ros2_client import ros2_client, init_ros2, ros2_manager
import threading
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Imu
import json
from django.views.decorators.http import require_http_methods
import logging

# Loglama yapılandırması
logging.basicConfig(
    level=logging.WARNING,  # INFO'dan WARNING'e değiştirildi
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)

logger = logging.getLogger('RobotControl')


class RobotController:
    """Robot kontrolü için ana sınıf."""
    
    # TurtleBot3 hız limitleri
    BURGER_MAX_LIN_VEL = 0.22
    BURGER_MAX_ANG_VEL = 2.84
    WAFFLE_MAX_LIN_VEL = 0.26
    WAFFLE_MAX_ANG_VEL = 1.82
    
    # Hız artış/azalış adımları
    LIN_VEL_STEP_SIZE = 0.03  # Doğrusal hız artış adımı (m/s)
    ANG_VEL_STEP_SIZE = 0.1   # Açısal hız artış adımı (rad/s)
    
    def __init__(self):
        """Robot kontrolcüsünü başlatır."""
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0
        self.target_linear_x = 0.0
        self.target_angular_z = 0.0
    
    @staticmethod
    def constrain(input_vel, low_bound, high_bound):
        """Hız değerini belirli sınırlar içinde tutar."""
        if input_vel < low_bound:
            input_vel = low_bound
        elif input_vel > high_bound:
            input_vel = high_bound
        return input_vel
    
    def check_linear_limit_velocity(self, velocity):
        """Doğrusal hızı sınırlar içinde tutar."""
        return self.constrain(
            velocity,
            -self.BURGER_MAX_LIN_VEL,
            self.BURGER_MAX_LIN_VEL
        )
    
    def check_angular_limit_velocity(self, velocity):
        """Açısal hızı sınırlar içinde tutar."""
        return self.constrain(
            velocity,
            -self.BURGER_MAX_ANG_VEL,
            self.BURGER_MAX_ANG_VEL
        )
    
    def make_simple_profile(self, output_vel, input_vel, slop):
        """Hız profilini oluşturur."""
        if input_vel > output_vel:
            output_vel = min(input_vel, output_vel + slop)
        elif input_vel < output_vel:
            output_vel = max(input_vel, output_vel - slop)
        else:
            output_vel = input_vel
        return output_vel
    
    def update_velocities(self):
        """Hız değerlerini günceller."""
        self.current_linear_x = self.make_simple_profile(
            self.current_linear_x,
            self.target_linear_x,
            (self.LIN_VEL_STEP_SIZE / 2.0)
        )
        
        self.current_angular_z = self.make_simple_profile(
            self.current_angular_z,
            self.target_angular_z,
            (self.ANG_VEL_STEP_SIZE / 2.0)
        )


# Global robot kontrolcü örneği
robot_controller = RobotController()


def ensure_ros2_initialized():
    """ROS2 bağlantısının başlatıldığından emin olur."""
    global ros2_client
    
    if ros2_client is None:
        try:
            if ros2_manager.initialize():
                ros2_client = ros2_manager.client
                # Bağlantı durumunu kontrol et
                if ros2_client.check_connection():
                    logger.info('ROS2 bağlantısı başarıyla kuruldu')
                    return True
                else:
                    logger.error('ROS2 bağlantısı kurulamadı: Gerekli topic\'ler eksik')
                    return False
            else:
                logger.error('ROS2 bağlantısı başlatılamadı')
                return False
        except Exception as e:
            logger.error(f'ROS2 bağlantısı hatası: {str(e)}')
            return False
    return True


def home(request):
    """Ana sayfa görünümü."""
    try:
        ensure_ros2_initialized()
        return render(request, 'robot_control/base.html', {
            'robot_controller': robot_controller
        })
    except Exception as e:
        return HttpResponse(f"ROS2 başlatma hatası: {str(e)}", status=500)


@csrf_exempt
def send_command(request):
    """Robot komutlarını işler ve gönderir."""
    if not ensure_ros2_initialized():
        return JsonResponse({
            'status': 'error',
            'message': 'ROS2 bağlantısı kurulamadı'
        })
    if request.method == 'POST':
        try:
            data = request.POST
            command = data.get('command')
            if command == 'forward':
                robot_controller.target_linear_x = (
                    robot_controller.check_linear_limit_velocity(
                        robot_controller.target_linear_x + 
                        robot_controller.LIN_VEL_STEP_SIZE
                    )
                )
                robot_controller.target_angular_z = 0.0
            elif command == 'backward':
                robot_controller.target_linear_x = (
                    robot_controller.check_linear_limit_velocity(
                        robot_controller.target_linear_x - 
                        robot_controller.LIN_VEL_STEP_SIZE
                    )
                )
                robot_controller.target_angular_z = 0.0
            elif command == 'left':
                robot_controller.target_angular_z = (
                    robot_controller.check_angular_limit_velocity(
                        robot_controller.target_angular_z + 
                        robot_controller.ANG_VEL_STEP_SIZE
                    )
                )
                robot_controller.target_linear_x = 0.0
            elif command == 'right':
                robot_controller.target_angular_z = (
                    robot_controller.check_angular_limit_velocity(
                        robot_controller.target_angular_z - 
                        robot_controller.ANG_VEL_STEP_SIZE
                    )
                )
                robot_controller.target_linear_x = 0.0
            elif command == 'stop':
                # Tüm hızları sıfırla
                robot_controller.target_linear_x = 0.0
                robot_controller.target_angular_z = 0.0
                robot_controller.current_linear_x = 0.0
                robot_controller.current_angular_z = 0.0
                # Hemen sıfır hız komutunu gönder
                ros2_client.publish_cmd_vel(0.0, 0.0)
                return JsonResponse({
                    'status': 'success',
                    'message': 'Robot durduruldu',
                    'linear_velocity': 0.0,
                    'angular_velocity': 0.0
                })
            
            robot_controller.update_velocities()
            ros2_client.publish_cmd_vel(
                robot_controller.current_linear_x,
                robot_controller.current_angular_z
            )
            return JsonResponse({
                'status': 'success',
                'message': 'Komut gönderildi',
                'linear_velocity': robot_controller.current_linear_x,
                'angular_velocity': robot_controller.current_angular_z
            })
        except Exception as e:
            return JsonResponse({
                'status': 'error',
                'message': f'Hata: {str(e)}'
            })
    return JsonResponse({
        'status': 'error',
        'message': 'Geçersiz istek metodu'
    })


def get_position(request):
    """Robot pozisyonunu ve hızlarını döndürür."""
    if not ensure_ros2_initialized():
        return JsonResponse({
            'status': 'error',
            'message': 'ROS2 bağlantısı kurulamadı'
        })
    try:
        # cmd_vel topic'inden gelen hız değerlerini al
        velocity_data = ros2_client.get_latest_velocity()
        
        return JsonResponse({
            'status': 'success',
            'position': ros2_client.get_position(),
            'linear_velocity': velocity_data['linear_x'],
            'angular_velocity': velocity_data['angular_z']
        })
    except Exception as e:
        return JsonResponse({
            'status': 'error',
            'message': f'Hata: {str(e)}'
        })


def camera_feed(request):
    """Kamera görüntüsünü döndürür."""
    if not ensure_ros2_initialized():
        return HttpResponse("ROS2 bağlantısı kurulamadı", status=503)
    try:
        image_data = ros2_client.get_latest_image()
        if image_data is None:
            return HttpResponse("Görüntü alınamadı", status=204)
        return HttpResponse(image_data, content_type='image/jpeg')
    except Exception as e:
        return HttpResponse(
            f"Görüntü işleme hatası: {str(e)}",
            status=500
        )


@require_http_methods(["GET"])
def get_raw_data(request):
    """Get raw sensor data from ROS2 topics"""
    if not ensure_ros2_initialized():
        return JsonResponse({
            'status': 'error',
            'message': 'ROS2 bağlantısı kurulamadı'
        })
    try:
        # Verileri al ve kontrol et
        imu_data = ros2_client.get_latest_imu_data()
        joint_states = ros2_client.get_latest_joint_states()
        tf_data = ros2_client.get_latest_tf_data()
        
        # Numpy dizilerini listeye dönüştür ve veri yapısını düzenle
        formatted_data = {
            'status': 'success',
            'imu_data': None,
            'joint_states': None,
            'tf_data': None
        }
        
        if imu_data is not None:
            try:
                formatted_data['imu_data'] = {
                    'linear_acceleration': {
                        'x': float(imu_data['linear_acceleration']['x']),
                        'y': float(imu_data['linear_acceleration']['y']),
                        'z': float(imu_data['linear_acceleration']['z'])
                    },
                    'angular_velocity': {
                        'x': float(imu_data['angular_velocity']['x']),
                        'y': float(imu_data['angular_velocity']['y']),
                        'z': float(imu_data['angular_velocity']['z'])
                    },
                    'orientation': {
                        'x': float(imu_data['orientation']['x']),
                        'y': float(imu_data['orientation']['y']),
                        'z': float(imu_data['orientation']['z']),
                        'w': float(imu_data['orientation']['w'])
                    }
                }
            except Exception as e:
                logger.error(f"IMU veri formatlama hatası: {str(e)}")
        
        if joint_states is not None:
            try:
                formatted_data['joint_states'] = {
                    'names': joint_states['names'],
                    'positions': [float(p) for p in joint_states['positions']],
                    'velocities': [float(v) for v in joint_states['velocities']],
                    'efforts': [float(e) for e in joint_states['efforts']]
                }
            except Exception as e:
                logger.error(f"Eklem durumu formatlama hatası: {str(e)}")
        
        if tf_data is not None:
            try:
                formatted_data['tf_data'] = {
                    frame_id: {
                        'translation': {
                            'x': float(transform['translation']['x']),
                            'y': float(transform['translation']['y']),
                            'z': float(transform['translation']['z'])
                        },
                        'rotation': {
                            'x': float(transform['rotation']['x']),
                            'y': float(transform['rotation']['y']),
                            'z': float(transform['rotation']['z']),
                            'w': float(transform['rotation']['w'])
                        }
                    }
                    for frame_id, transform in tf_data.items()
                }
            except Exception as e:
                logger.error(f"TF veri formatlama hatası: {str(e)}")
        
        return JsonResponse(formatted_data)
    except Exception as e:
        logger.error(f"Ham veri alınırken hata oluştu: {str(e)}")
        return JsonResponse({
            'status': 'error',
            'message': str(e)
        })

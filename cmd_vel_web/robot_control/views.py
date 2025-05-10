# robot_control/views.py
from django.shortcuts import render
from django.http import JsonResponse, HttpResponse
from django.views.decorators.csrf import csrf_exempt
from .ros2_client import ros2_client, init_ros2
import threading
import time


class RobotController:
    """Robot kontrolü için ana sınıf."""
    
    # TurtleBot3 hız limitleri
    BURGER_MAX_LIN_VEL = 0.22
    BURGER_MAX_ANG_VEL = 2.84
    WAFFLE_MAX_LIN_VEL = 0.26
    WAFFLE_MAX_ANG_VEL = 1.82
    
    LIN_VEL_STEP_SIZE = 0.01
    ANG_VEL_STEP_SIZE = 0.1
    
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
    if ros2_client is None:
        return init_ros2()
    return True


def home(request):
    """Ana sayfa görünümü."""
    try:
        ensure_ros2_initialized()
        return render(request, 'robot_control/base.html')
    except Exception as e:
        return HttpResponse(f"ROS2 başlatma hatası: {str(e)}", status=500)


def teleop(request):
    """Teleoperasyon görünümü."""
    try:
        ensure_ros2_initialized()
        return render(request, 'robot_control/base.html')
    except Exception as e:
        return HttpResponse(f"ROS2 başlatma hatası: {str(e)}", status=500)


def raw_data(request):
    """Ham veri görünümü."""
    return render(request, 'robot_control/base.html')


def reports(request):
    """Raporlar görünümü."""
    return render(request, 'robot_control/base.html')


def about(request):
    """Hakkında görünümü."""
    return render(request, 'robot_control/base.html')


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
            elif command == 'backward':
                robot_controller.target_linear_x = (
                    robot_controller.check_linear_limit_velocity(
                        robot_controller.target_linear_x - 
                        robot_controller.LIN_VEL_STEP_SIZE
                    )
                )
            elif command == 'left':
                robot_controller.target_angular_z = (
                    robot_controller.check_angular_limit_velocity(
                        robot_controller.target_angular_z + 
                        robot_controller.ANG_VEL_STEP_SIZE
                    )
                )
            elif command == 'right':
                robot_controller.target_angular_z = (
                    robot_controller.check_angular_limit_velocity(
                        robot_controller.target_angular_z - 
                        robot_controller.ANG_VEL_STEP_SIZE
                    )
                )
            elif command == 'stop':
                robot_controller.target_linear_x = 0.0
                robot_controller.target_angular_z = 0.0
            
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
    """Robot pozisyonunu döndürür."""
    if not ensure_ros2_initialized():
        return JsonResponse({
            'status': 'error',
            'message': 'ROS2 bağlantısı kurulamadı'
        })
    
    try:
        return JsonResponse({
            'status': 'success',
            'position': ros2_client.get_position(),
            'linear_velocity': robot_controller.current_linear_x,
            'angular_velocity': robot_controller.current_angular_z
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


def move_forward(request):
    """Robotu ileri hareket ettirir."""
    if not ensure_ros2_initialized():
        return JsonResponse({
            'status': 'error',
            'message': 'ROS2 bağlantısı kurulamadı'
        })
    
    try:
        robot_controller.target_linear_x = (
            robot_controller.check_linear_limit_velocity(
                robot_controller.target_linear_x + 
                robot_controller.LIN_VEL_STEP_SIZE
            )
        )
        robot_controller.target_angular_z = 0.0
        robot_controller.current_linear_x = robot_controller.target_linear_x
        robot_controller.current_angular_z = robot_controller.target_angular_z
        
        ros2_client.publish_cmd_vel(
            robot_controller.current_linear_x,
            robot_controller.current_angular_z
        )
        
        return JsonResponse({
            'status': 'success',
            'message': 'İleri hareket komutu gönderildi',
            'linear_velocity': robot_controller.current_linear_x,
            'angular_velocity': robot_controller.current_angular_z
        })
    except Exception as e:
        return JsonResponse({
            'status': 'error',
            'message': f'Hata: {str(e)}'
        })


def move_backward(request):
    """Robotu geri hareket ettirir."""
    if not ensure_ros2_initialized():
        return JsonResponse({
            'status': 'error',
            'message': 'ROS2 bağlantısı kurulamadı'
        })
    
    try:
        robot_controller.target_linear_x = (
            robot_controller.check_linear_limit_velocity(
                robot_controller.target_linear_x - 
                robot_controller.LIN_VEL_STEP_SIZE
            )
        )
        robot_controller.target_angular_z = 0.0
        robot_controller.current_linear_x = robot_controller.target_linear_x
        robot_controller.current_angular_z = robot_controller.target_angular_z
        
        ros2_client.publish_cmd_vel(
            robot_controller.current_linear_x,
            robot_controller.current_angular_z
        )
        
        return JsonResponse({
            'status': 'success',
            'message': 'Geri hareket komutu gönderildi',
            'linear_velocity': robot_controller.current_linear_x,
            'angular_velocity': robot_controller.current_angular_z
        })
    except Exception as e:
        return JsonResponse({
            'status': 'error',
            'message': f'Hata: {str(e)}'
        })


def turn_left(request):
    """Robotu sola döndürür."""
    if not ensure_ros2_initialized():
        return JsonResponse({
            'status': 'error',
            'message': 'ROS2 bağlantısı kurulamadı'
        })
    
    try:
        robot_controller.target_linear_x = 0.0
        robot_controller.target_angular_z = (
            robot_controller.check_angular_limit_velocity(
                robot_controller.target_angular_z + 
                robot_controller.ANG_VEL_STEP_SIZE
            )
        )
        robot_controller.current_linear_x = robot_controller.target_linear_x
        robot_controller.current_angular_z = robot_controller.target_angular_z
        
        ros2_client.publish_cmd_vel(
            robot_controller.current_linear_x,
            robot_controller.current_angular_z
        )
        
        return JsonResponse({
            'status': 'success',
            'message': 'Sola dönüş komutu gönderildi',
            'linear_velocity': robot_controller.current_linear_x,
            'angular_velocity': robot_controller.current_angular_z
        })
    except Exception as e:
        return JsonResponse({
            'status': 'error',
            'message': f'Hata: {str(e)}'
        })


def turn_right(request):
    """Robotu sağa döndürür."""
    if not ensure_ros2_initialized():
        return JsonResponse({
            'status': 'error',
            'message': 'ROS2 bağlantısı kurulamadı'
        })
    
    try:
        robot_controller.target_linear_x = 0.0
        robot_controller.target_angular_z = (
            robot_controller.check_angular_limit_velocity(
                robot_controller.target_angular_z - 
                robot_controller.ANG_VEL_STEP_SIZE
            )
        )
        robot_controller.current_linear_x = robot_controller.target_linear_x
        robot_controller.current_angular_z = robot_controller.target_angular_z
        
        ros2_client.publish_cmd_vel(
            robot_controller.current_linear_x,
            robot_controller.current_angular_z
        )
        
        return JsonResponse({
            'status': 'success',
            'message': 'Sağa dönüş komutu gönderildi',
            'linear_velocity': robot_controller.current_linear_x,
            'angular_velocity': robot_controller.current_angular_z
        })
    except Exception as e:
        return JsonResponse({
            'status': 'error',
            'message': f'Hata: {str(e)}'
        })


def stop(request):
    """Robotu durdurur."""
    if not ensure_ros2_initialized():
        return JsonResponse({
            'status': 'error',
            'message': 'ROS2 bağlantısı kurulamadı'
        })
    
    try:
        robot_controller.target_linear_x = 0.0
        robot_controller.target_angular_z = 0.0
        robot_controller.current_linear_x = robot_controller.target_linear_x
        robot_controller.current_angular_z = robot_controller.target_angular_z
        
        ros2_client.publish_cmd_vel(
            robot_controller.current_linear_x,
            robot_controller.current_angular_z
        )
        
        return JsonResponse({
            'status': 'success',
            'message': 'Durma komutu gönderildi',
            'linear_velocity': robot_controller.current_linear_x,
            'angular_velocity': robot_controller.current_angular_z
        })
    except Exception as e:
        return JsonResponse({
            'status': 'error',
            'message': f'Hata: {str(e)}'
        })

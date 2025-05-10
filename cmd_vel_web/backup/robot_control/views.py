# robot_control/views.py
from django.shortcuts import render
from django.http import JsonResponse
from django.views.decorators.csrf import csrf_exempt
from .ros2_client import ros2_client, init_ros2
import asyncio
import threading

# Mevcut hız değerlerini tutmak için global değişkenler
current_linear_x = 0.0
current_angular_z = 0.0

def control_panel(request):
    init_ros2()
    return render(request, 'robot_control/control_panel.html')

@csrf_exempt
def send_command(request):
    global current_linear_x, current_angular_z
    data = request.POST
    linear_x = float(data.get('linear_x', current_linear_x))
    angular_z = float(data.get('angular_z', current_angular_z))
    current_linear_x = linear_x
    current_angular_z = angular_z
    ros2_client.publish_cmd_vel(linear_x, angular_z)
    return JsonResponse({'status': 'success', 'message': 'Komut gönderildi'})

@csrf_exempt
def move_forward(request):
    global current_linear_x, current_angular_z
    current_linear_x = 0.4
    ros2_client.publish_cmd_vel(current_linear_x, current_angular_z)
    return JsonResponse({'status': 'success', 'message': 'İleri hareket'})

@csrf_exempt
def move_backward(request):
    global current_linear_x, current_angular_z
    current_linear_x = -0.4
    ros2_client.publish_cmd_vel(current_linear_x, current_angular_z)
    return JsonResponse({'status': 'success', 'message': 'Geri hareket'})

def reset_angular_velocity():
    global current_angular_z
    current_angular_z = 0.0
    ros2_client.publish_cmd_vel(current_linear_x, current_angular_z)

@csrf_exempt
def turn_left(request):
    global current_linear_x, current_angular_z
    current_angular_z = 0.3
    ros2_client.publish_cmd_vel(current_linear_x, current_angular_z)
    # 0.5 saniye sonra dönüş hızını sıfırla
    
    return JsonResponse({'status': 'success', 'message': 'Sol dönüş'})

@csrf_exempt
def turn_right(request):
    global current_linear_x, current_angular_z
    current_angular_z = -0.3
    ros2_client.publish_cmd_vel(current_linear_x, current_angular_z)
    # 0.5 saniye sonra dönüş hızını sıfırla
   
    return JsonResponse({'status': 'success', 'message': 'Sağ dönüş'})

@csrf_exempt
def stop(request):
    global current_linear_x, current_angular_z
    current_linear_x = 0.0
    current_angular_z = 0.0
    ros2_client.publish_cmd_vel(current_linear_x, current_angular_z)
    return JsonResponse({'status': 'success', 'message': 'Durduruldu'})

def get_position(request):
    position = ros2_client.get_position()
    return JsonResponse({'status': 'success', 'position': position})

# robot_control/urls.py
from django.urls import path
from . import views

app_name = 'robot_control'

urlpatterns = [
    path('', views.home, name='home'),
    path('send_command/', views.send_command, name='send_command'),
    path('get_position/', views.get_position, name='get_position'),
    path('camera_feed/', views.camera_feed, name='camera_feed'),
    path('get_raw_data/', views.get_raw_data, name='get_raw_data'),
]

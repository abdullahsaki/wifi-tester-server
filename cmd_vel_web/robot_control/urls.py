# robot_control/urls.py
from django.urls import path
from . import views

app_name = 'robot_control'

urlpatterns = [
    path('', views.home, name='home'),
    path('teleop/', views.teleop, name='teleop'),
    path('raw-data/', views.raw_data, name='raw_data'),
    path('reports/', views.reports, name='reports'),
    path('about/', views.about, name='about'),
    path('command/', views.send_command, name='send_command'),
    path('position/', views.get_position, name='get_position'),
    path('move/forward/', views.move_forward, name='move_forward'),
    path('move/backward/', views.move_backward, name='move_backward'),
    path('turn/left/', views.turn_left, name='turn_left'),
    path('turn/right/', views.turn_right, name='turn_right'),
    path('stop/', views.stop, name='stop'),
    path('camera-feed/', views.camera_feed, name='camera_feed'),
]

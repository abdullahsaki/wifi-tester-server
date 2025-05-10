from django.apps import AppConfig
from .ros2_client import init_ros2


class RobotControlConfig(AppConfig):
    default_auto_field = 'django.db.models.BigAutoField'
    name = 'robot_control'

    def ready(self):
        init_ros2()

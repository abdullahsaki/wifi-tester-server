# cmd_vel_web/urls.py
from django.contrib import admin
from django.urls import path, include # include fonksiyonunu import edin

urlpatterns = [
    path('admin/', admin.site.urls),
    # /robot/ ile başlayan tüm istekleri robot_control uygulamasının urls.py dosyasına yönlendir
    path('robot/', include('robot_control.urls')),
]

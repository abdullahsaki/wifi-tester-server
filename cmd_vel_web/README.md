# ROS2 Robot Control Web Interface

Bu proje, ROS2 tabanlı robotları web arayüzü üzerinden kontrol etmek için geliştirilmiş bir Django uygulamasıdır.

## Özellikler

- Web tabanlı robot kontrol arayüzü
- ROS2 entegrasyonu
- Gerçek zamanlı veri akışı
- Responsive tasarım
- WebSocket desteği

## Gereksinimler

- Python 3.8+
- ROS2 Humble veya üzeri
- Django 4.2+
- rclpy
- geometry_msgs
- numpy
- opencv-python-headless
- pillow
- python-dotenv
- django-cors-headers
- channels
- daphne

## Kurulum

1. Projeyi klonlayın:
```bash
git clone https://github.com/abdullahsaki/wifi-tester-server.git
cd wifi-tester-server
```

2. Sanal ortam oluşturun ve aktifleştirin:
```bash
python -m venv venv
source venv/bin/activate  # Linux/Mac
# veya
.\venv\Scripts\activate  # Windows
```

3. Bağımlılıkları yükleyin:
```bash
pip install -r requirements.txt
```

4. `.env` dosyası oluşturun:
```bash
cp .env.example .env
# .env dosyasını düzenleyin
```

5. Veritabanı migrasyonlarını uygulayın:
```bash
python manage.py migrate
```

6. Statik dosyaları toplayın:
```bash
python manage.py collectstatic
```

## Kullanım

1. ROS2 ortamını hazırlayın:
```bash
source /opt/ros/humble/setup.bash
```

2. Django sunucusunu başlatın:
```bash
python manage.py runserver
```

3. Tarayıcınızda `http://localhost:8000` adresine gidin.

## Geliştirme

### Proje Yapısı

```
wifi-tester-server/
├── cmd_vel_web/          # Ana Django projesi
├── robot_control/        # Robot kontrol uygulaması
├── static/              # Statik dosyalar
├── media/               # Medya dosyaları
├── templates/           # HTML şablonları
├── requirements.txt     # Python bağımlılıkları
└── manage.py           # Django yönetim scripti
```

### Test

```bash
python manage.py test
```

## Lisans

Bu proje MIT lisansı altında lisanslanmıştır. Detaylar için [LICENSE](LICENSE) dosyasına bakın.

## Katkıda Bulunma

1. Bu depoyu fork edin
2. Yeni bir branch oluşturun (`git checkout -b feature/amazing-feature`)
3. Değişikliklerinizi commit edin (`git commit -m 'Add some amazing feature'`)
4. Branch'inizi push edin (`git push origin feature/amazing-feature`)
5. Bir Pull Request oluşturun

## İletişim

Abdullah Saki - [@abdullahsaki](https://github.com/abdullahsaki)

Proje Linki: [https://github.com/abdullahsaki/wifi-tester-server](https://github.com/abdullahsaki/wifi-tester-server)


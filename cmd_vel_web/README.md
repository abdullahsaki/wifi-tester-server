# TurtleBot3 Web Kontrol Arayüzü

Bu proje, TurtleBot3 robotunu web tabanlı bir arayüz üzerinden kontrol etmenizi sağlayan bir Django uygulamasıdır. ROS2 (Robot Operating System 2) ile entegre çalışarak, robotun hareketlerini kontrol edebilir ve kamera görüntüsünü gerçek zamanlı olarak izleyebilirsiniz.

## Özellikler

- Web tabanlı robot kontrol arayüzü
- Gerçek zamanlı kamera görüntüsü
- Robot pozisyon takibi
- Hız kontrolü ve sınırlamaları
- ROS2 entegrasyonu

## Gereksinimler

- Python 3.8+
- Django 4.2+
- ROS2 (Humble veya üzeri)
- OpenCV
- NumPy

## Kurulum

1. Projeyi klonlayın:
```bash
git clone https://github.com/kullaniciadi/cmd_vel_web.git
cd cmd_vel_web
```

2. Sanal ortam oluşturun ve aktifleştirin:
```bash
python -m venv venv
source venv/bin/activate  # Linux/Mac için
# veya
.\venv\Scripts\activate  # Windows için
```

3. Gerekli paketleri yükleyin:
```bash
pip install -r requirements.txt
```

4. ROS2 ortamınızı kaynaklayın:
```bash
source /opt/ros/humble/setup.bash  # ROS2 Humble için
```

## Kullanım

1. Django sunucusunu başlatın:
```bash
python manage.py runserver 0.0.0.0:8000
```

2. Web tarayıcınızda aşağıdaki adresi açın:
```
http://localhost:8000
```

## Kontroller

- İleri/Geri: Robotu ileri veya geri hareket ettirir
- Sol/Sağ: Robotu sola veya sağa döndürür
- Dur: Robotu durdurur

## Hız Limitleri

- TurtleBot3 Burger:
  - Maksimum Doğrusal Hız: 0.22 m/s
  - Maksimum Açısal Hız: 2.84 rad/s

- TurtleBot3 Waffle:
  - Maksimum Doğrusal Hız: 0.26 m/s
  - Maksimum Açısal Hız: 1.82 rad/s

## Katkıda Bulunma

1. Bu depoyu fork edin
2. Yeni bir özellik dalı oluşturun (`git checkout -b yeni-ozellik`)
3. Değişikliklerinizi commit edin (`git commit -am 'Yeni özellik: Açıklama'`)
4. Dalınıza push yapın (`git push origin yeni-ozellik`)
5. Bir Pull Request oluşturun

## Lisans

Bu proje MIT lisansı altında lisanslanmıştır. Daha fazla bilgi için `LICENSE` dosyasına bakın.

## İletişim

Sorularınız veya önerileriniz için lütfen bir issue açın veya pull request gönderin. 
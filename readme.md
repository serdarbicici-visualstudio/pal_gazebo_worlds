Tamam kanka, gel seninle tertemiz bir başlangıç yapalım. Bilgisayarı yeni açmışsın gibi **sıfırdan, adım adım** yazıyorum.

Toplamda **5 tane terminal** açacağız. Her birini sırayla yap, acele etme.

Önce şu Python kodunu (`mission_master.py`) kaydettiğinden emin ol, sonra başla:

---

### 🖥️ 1. TERMINAL: Simülasyonu (Gazebo) Aç

Önce robotu ve odayı dünyaya getiriyoruz.

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/dev/null
export ROS_LOCALHOST_ONLY=1

ros2 launch tiago_gazebo tiago_gazebo.launch.py is_public_sim:=True world_name:=galatasaray
ros2 launch nav2_bringup slam_launch.py use_sim_time:=True
```

*(Gazebo açılacak, robot ofisin ortasında duracak. Açılana kadar bekle.)*

---

### 🌉 2. TERMINAL: Lazer Köprüsünü Kur (ÇOK ÖNEMLİ)

Bu olmazsa Nav2 kör olur, çalışmaz. Bu terminal hep açık kalacak.

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/dev/null
export ROS_LOCALHOST_ONLY=1

ros2 run topic_tools relay /scan_raw /scan

```

*(Ekranda "Relaying..." yazabilir veya hiçbir şey yazmaz, hata vermiyorsa tamamdır.)*

---
### 🧠 3. TERMINAL: Nav2 Navigasyon Sistemini Başlat

Haritayı yüklüyoruz ve robotun beynini açıyoruz.

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/dev/null
export ROS_LOCALHOST_ONLY=1

# Harita yolunun doğru olduğundan emin ol ($HOME kullanıyoruz)
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=True autostart:=True map:=$HOME/ros2_ws/src/pal_gazebo_worlds/library_bot/my_map.yaml

```

---

### 👁️ 4. TERMINAL: RViz ile Robotu Konumlandır

Robot nerede olduğunu bilmiyor, ona yerini göstermemiz lazım.

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/dev/null
export ROS_LOCALHOST_ONLY=1
ros2 run rviz2 rviz2

```

**RViz Açılınca Yapılacaklar (Sırayla):**

1. **Add** -> **By Topic** -> **/map** (Eğer harita görünmezse Durability: `Transient Local` yap).
2. **Add** -> **By Display Type** -> **RobotModel** (Robotu görmek için).
3. **Add** -> **By Topic** -> **/scan** (Lazer çizgilerini görmek için).
4. **Fixed Frame** ayarını **`map`** yap.
5. Üstteki **"2D Pose Estimate"** butonuna bas.
6. Haritada robotun **Gazebo'da durduğu yeri** işaretle ve yönünü ayarla.
* *İpucu:* Lazer çizgileri (kırmızı noktalar) haritadaki siyah duvarlarla tam örtüşmeli.



---

### 🚀 5. TERMINAL: FİNAL (Kodu Çalıştır)

Her şey hazırsa, robotu göreve gönderiyoruz.

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/dev/null
export ROS_LOCALHOST_ONLY=1

python3 library_bot/mission_master.py

```

**Ne Olacak?**

1. Robot Nav2 ile masanın yakınına gidecek.
2. Orada durup Kamera Moduna geçecek.
3. Kafayı eğip yeşili bulacak ve masaya sıfır yanaşacak.

Hadi yapıştır, şov başlasın! 😎🔥

Bunlar da gazebo kill commandleri illa açık gazeboyu kapatacaksan bunu çalıştır:
pkill -f gzserver
pkill -f gzclient

export FASTRTPS_DEFAULT_PROFILES_FILE=/dev/null
export ROS_LOCALHOST_ONLY=1

# Harita (map) ile Robot Başlangıcı (odom) arasına sabit köprü atıyoruz
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom

# Map save command
cd ~/ros2_ws/src/pal_gazebo_worlds/library_bot/
ros2 run nav2_map_server map_saver_cli -f my_project_map
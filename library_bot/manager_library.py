#!/usr/bin/env python3
import subprocess
import time
import sys
import os
import signal

def run_orchestra():
    # --- AYARLAR ---
    FOLDER_NAME = "library_bot"  # Dosyaların olduğu klasör
    
    # Dosya isimleri (Klasör adını aşağıda otomatik ekleyeceğiz)
    script_go_desk = "go_desk.py"
    script_grip = "grip.py"
    script_auto_lock = "auto_lock.py"
    script_shelf = "shelf_placer.py" # (Veya senin en son yazdığın dosya ismi)
    # ---------------

    # --- 1. ORTAM DEĞİŞKENLERİ AYARI (EXPORT) ---
    ros_env = os.environ.copy()
    ros_env["FASTRTPS_DEFAULT_PROFILES_FILE"] = "/dev/null"
    ros_env["ROS_LOCALHOST_ONLY"] = "1"
    
    print("🛠️  ROS2 Ortam Ayarları Yüklendi (library_bot klasörü için)")
    print(f"📂 Çalışma Klasörü: {FOLDER_NAME}/")
    print("🚀 GÖREV BAŞLIYOR...\n")

    # --- YARDIMCI FONKSİYON: Tam Dosya Yolunu Bul ---
    def get_path(script_name):
        # "library_bot/go_desk.py" gibi bir yol oluşturur
        full_path = os.path.join(FOLDER_NAME, script_name)
        if not os.path.exists(full_path):
            print(f"❌ HATA: '{full_path}' dosyası bulunamadı!")
            print(f"   Lütfen '{script_name}' dosyasının '{FOLDER_NAME}' klasöründe olduğuna emin ol.")
            sys.exit(1)
        return full_path

    # -------------------------------------------------------
    # ADIM 1: MASAYA GİT
    # -------------------------------------------------------
    path_go = get_path(script_go_desk)
    print(f"▶️  ADIM 1: '{path_go}' çalıştırılıyor...")
    try:
        subprocess.run(["python3", path_go], check=True, env=ros_env)
        print("✅ Masaya varıldı.\n")
        time.sleep(1.0)
    except subprocess.CalledProcessError:
        print("❌ HATA: Masaya gidilemedi! Operasyon iptal.")
        sys.exit(1)

    # -------------------------------------------------------
    # ADIM 2: TUT
    # -------------------------------------------------------
    path_grip = get_path(script_grip)
    print(f"▶️  ADIM 2: '{path_grip}' çalıştırılıyor...")
    try:
        subprocess.run(["python3", path_grip], check=True, env=ros_env)
        print("✅ Nesne tutuldu.\n")
        time.sleep(1.0)
    except subprocess.CalledProcessError:
        print("❌ HATA: Tutma başarısız! Operasyon iptal.")
        sys.exit(1)

    # -------------------------------------------------------
    # ADIM 3: AUTO LOCK (ARKAPLAN)
    # -------------------------------------------------------
    path_lock = get_path(script_auto_lock)
    print(f"🔄 ADIM 3: '{path_lock}' ARKAPLANDA başlatılıyor...")
    
    lock_process = subprocess.Popen(["python3", path_lock], env=ros_env)
    
    print("⏳ Lock sisteminin devreye girmesi bekleniyor (2sn)...")
    time.sleep(2.0)
    print("🔒 Auto Lock AKTİF!\n")

    # -------------------------------------------------------
    # ADIM 4: RAFA GİT (Lock çalışırken)
    # -------------------------------------------------------
    path_shelf = get_path(script_shelf)
    print(f"▶️  ADIM 4: '{path_shelf}' çalıştırılıyor...")
    try:
        subprocess.run(["python3", path_shelf], check=True, env=ros_env)
        print("✅ Rafa yerleştirme TAMAMLANDI.\n")
    except subprocess.CalledProcessError:
        print("❌ HATA: Rafa gidilemedi!")
    
    # -------------------------------------------------------
    # FİNAL: AUTO LOCK'I KAPAT
    # -------------------------------------------------------
    print("🛑 OPERASYON BİTTİ. Auto Lock kapatılıyor...")
    
    lock_process.terminate()
    try:
        lock_process.wait(timeout=2)
        print("💀 Auto Lock sonlandırıldı.")
    except subprocess.TimeoutExpired:
        lock_process.kill()
        print("💀 Auto Lock zorla öldürüldü.")

    print("\n🏁🏁🏁 MİSYON BAŞARIYLA TAMAMLANDI! 🏁🏁🏁")

if __name__ == "__main__":
    try:
        run_orchestra()
    except KeyboardInterrupt:
        print("\n⚠️ İptal edildi! Temizlik yapılıyor...")
        # İsmi library_bot içinde olsa bile process adı yine de auto_lock.py görünür
        subprocess.run(["pkill", "-f", "auto_lock.py"])
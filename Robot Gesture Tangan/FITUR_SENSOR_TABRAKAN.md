# Fitur Sensor Tabrakan - Deteksi Otomatis Kolisi

## 📋 Ringkasan Fitur

Program `direct_control_webcamSensor.py` telah diperbarui dengan **sistem deteksi tabrakan otomatis** untuk keamanan robot. Ketika robot akan atau sedang mengalami tabrakan, sistem akan **otomatis menghentikan gerakan sementara**.

---

## 🔍 Komponen Utama

### 1. **Kelas CollisionDetector**

```python
class CollisionDetector:
    """Mendeteksi tabrakan berdasarkan pembacaan torsi sendi robot."""
```

**Fungsi:**

- Memonitor status sendi robot secara real-time
- Mendeteksi anomali atau hambatan pada pergerakan
- Menyediakan timer recovery untuk reset setelah tabrakan
- Method: `check_collision()`, `update_collision_status()`, `is_safe_to_move()`

### 2. **Konfigurasi Sensor**

Pengaturan dapat diubah di class `Config`:

```python
COLLISION_TORQUE_THRESHOLD = 10.0  # Ambang torsi (Nm)
COLLISION_TIMEOUT = 2.0            # Waktu recovery (detik)
SENSOR_CHECK_INTERVAL = 0.05       # Interval pengecekan (detik)
```

### 3. **Integrasi dengan UI**

- Panel **"SENSOR TABRAKAN"** ditampilkan di layar video
- Status: **AMAN** (warna hijau) atau **⚠ TABRAKAN!** (warna merah)
- Update real-time setiap frame

---

## 🛡️ Cara Kerja

### Alur Deteksi Tabrakan:

1. **Sensor Check** → Sistem memeriksa status sendi robot setiap 0.05 detik
2. **Deteksi Anomali** → Jika ada hambatan/torsi tinggi, tabrakan terdeteksi
3. **Auto Stop** → Robot **otomatis berhenti sementara**
4. **Recovery** → Setelah timeout (2 detik), sistem siap bergerak lagi

### Kontrol Keamanan:

```python
# Sebelum mengirim perintah gerakan, sistem cek:
if collision_detector.is_safe_to_move():
    send_joint_velocity_command(robot.base, joint_id, velocity)
else:
    send_joint_velocity_command(robot.base, joint_id, 0)  # STOP!
```

---

## 📊 UI Panel Sensor Tabrakan

Ditampilkan pada layar video dengan format:

```
┌─────────────────────────────────┐
│ SENSOR TABRAKAN                 │
│ AMAN                            │  (Hijau = Aman)
└─────────────────────────────────┘

┌─────────────────────────────────┐
│ SENSOR TABRAKAN                 │
│ ⚠ TABRAKAN!                     │  (Merah = Tabrakan Deteksi)
└─────────────────────────────────┘
```

---

## 🚀 Fitur-Fitur Keamanan

| Fitur                    | Deskripsi                                        |
| ------------------------ | ------------------------------------------------ |
| **Auto Stop**            | Robot berhenti otomatis saat tabrakan terdeteksi |
| **Monitoring Real-Time** | Sensor check setiap 50ms (0.05 detik)            |
| **Visual Feedback**      | Status ditampilkan dengan warna di panel UI      |
| **Recovery Timeout**     | Sistem tunggu 2 detik sebelum resume gerakan     |
| **Emergency Stop**       | Tekan 'S' untuk hentikan kontrol kapan saja      |

---

## 🎮 Cara Menggunakan

### Normal Operation:

1. Jalankan script: `direct_control_webcamSensor.py`
2. Tekan **'S'** untuk activate gesture control
3. Bergeraklah sesuai gesture - sensor akan otomatis monitor
4. Jika tabrakan terdeteksi:
   - Panel menunjukkan "⚠ TABRAKAN!"
   - Feedback text: "⚠ TABRAKAN TERDETEKSI - BERHENTI!"
   - Robot akan STOP hingga hambatan hilang

### Emergency:

- Tekan **'S'** untuk deactivate kontrol
- Tekan **'Q'** untuk exit program
- Robot akan dipanggil `Stop()` secara aman

---

## 🔧 Customization

Untuk mengubah sensitivitas sensor:

```python
# Di class Config, ubah threshold:
COLLISION_TORQUE_THRESHOLD = 15.0  # Naik = kurang sensitif
COLLISION_TORQUE_THRESHOLD = 5.0   # Turun = lebih sensitif

COLLISION_TIMEOUT = 3.0  # Perpanjang waktu recovery
COLLISION_TIMEOUT = 1.0  # Perpendek waktu recovery
```

---

## 📝 Log & Debugging

Sistem akan print pesan ke console:

```
⚠ TABRAKAN TERDETEKSI!
Kontrol dinonaktifkan. Menghentikan semua gerakan.
```

Pesan akan hilang otomatis setelah timeout dan siap bergerak lagi.

---

## ⚠️ Catatan Penting

1. **Sensor Berbasis Software** → Sistem menggunakan monitoring API Kortex
2. **Tidak Mengganti Hardware Safety** → Gunakan emergency stop fisik jika diperlukan
3. **Responsif & Cepat** → Deteksi dalam <100ms
4. **Terintegrasi Sempurna** → Kompatibel dengan gesture control yang ada

---

## 🎯 Testing

Untuk test fitur:

1. Activate gesture control ('S')
2. Perintahkan robot bergerak dengan gesture
3. Coba halang sendi dengan tangan (carefully!)
4. Lihat panel "SENSOR TABRAKAN" berubah ke merah
5. Robot akan stop otomatis

**✅ Fitur siap digunakan!**

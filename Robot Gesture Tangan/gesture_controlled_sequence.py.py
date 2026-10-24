import time
import cv2
import mediapipe as mp
from enum import Enum

# Import Kortex API
from kortex_api.TCPTransport import TCPTransport
from kortex_api.RouterClient import RouterClient
from kortex_api.SessionManager import SessionManager
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.messages import Session_pb2, Base_pb2

# --- ENUM UNTUK MENGELOLA STATE PROGRAM ---
class ProgramState(Enum):
    WAITING_FOR_ACTIVATION = 1
    RUNNING_SEQUENCE = 2
    EMERGENCY_STOP = 3

# ===============================================================
# BAGIAN KONEKSI DAN KONTROL ROBOT (Tidak banyak berubah)
# ===============================================================

def control_gripper(device, position):
    try:
        gripper_command = Base_pb2.GripperCommand()
        gripper_command.mode = Base_pb2.GRIPPER_POSITION
        finger = gripper_command.gripper.finger.add()
        finger.finger_identifier = 1
        finger.value = position
        action_name = "Membuka" if position < 0.1 else "Menutup"
        print(f"Mengirim perintah untuk {action_name} gripper ke posisi {position:.2f}")
        device.base.SendGripperCommand(gripper_command)
        time.sleep(1.5)
        return True
    except Exception as e:
        print(f"Gagal menggerakkan gripper: {str(e)}")
        return False

class DeviceConnection:
    # ... (Isi kelas DeviceConnection tetap sama persis seperti sebelumnya) ...
    def __init__(self, ip_address, username, password):
        self.ip = ip_address
        self.username = username
        self.password = password
        self.session_manager = None
        self.transport = TCPTransport()
        self.router = RouterClient(self.transport, RouterClient.basicErrorCallback)
        self.base = None

    def connect(self):
        print(f"Mencoba menghubungkan ke robot di alamat IP: {self.ip}...")
        try:
            self.transport.connect(self.ip, 10000)
            self.session_manager = SessionManager(self.router)
            session_info = Session_pb2.CreateSessionInfo()
            session_info.username = self.username
            session_info.password = self.password
            session_info.session_inactivity_timeout = 60000
            print("Mengirim permintaan sesi...")
            self.session_manager.CreateSession(session_info)
            self.base = BaseClient(self.router)
            print("Sesi berhasil dibuat.")
            return True
        except Exception as e:
            print(f"Gagal terhubung: {str(e)}")
            return False

    def ensure_control(self):
        try:
            print("Memastikan kontrol robot...")
            self.base.ClearFaults()
            time.sleep(1)
            servoing_mode = Base_pb2.ServoingModeInformation()
            servoing_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
            self.base.SetServoingMode(servoing_mode)
            time.sleep(1)
            return True
        except Exception as e:
            print(f"Gagal memastikan kontrol: {str(e)}")
            return False

    def disconnect(self):
        try:
            if self.base: self.base.Stop()
            if self.session_manager: self.session_manager.CloseSession()
            if self.transport: self.transport.disconnect()
            print("Koneksi & Sesi ditutup.")
        except Exception as e:
            print(f"Error saat memutuskan: {str(e)}")

    def move_to_joint_angles(self, angles, duration=5):
        global current_state 
        try:
            action = Base_pb2.Action()
            action.name = "Joint Angles Movement"
            action.handle.action_type = Base_pb2.REACH_JOINT_ANGLES
            
            joint_angles = action.reach_joint_angles.joint_angles
            for i, angle in enumerate(angles):
                joint_angle = joint_angles.joint_angles.add()
                joint_angle.joint_identifier = i
                joint_angle.value = angle
            
            print(f"Menjalankan pergerakan joint ke {angles}...")
            self.base.ExecuteAction(action)
            
            end_time = time.time() + duration
            while time.time() < end_time:
                if check_for_emergency_stop_gesture():
                    print("!! GESTUR E-STOP TERDETEKSI SELAMA PERGERAKAN !!")
                    self.base.Stop()
                    current_state = ProgramState.EMERGENCY_STOP
                    return "E_STOP"
                time.sleep(0.1)
            
            return "SUCCESS"
        except Exception as e:
            print(f"Gagal menggerakkan joint: {str(e)}")
            return "FAIL"

# ===============================================================
# BAGIAN DETEKSI GESTUR (Tidak berubah)
# ===============================================================
def count_fingers(hand_landmarks):
    # ... (Isi fungsi count_fingers tetap sama persis seperti sebelumnya) ...
    if not hand_landmarks: return -1
    fingers_up = 0
    tip_ids = [4, 8, 12, 16, 20]
    is_left_hand = hand_landmarks.landmark[0].x > hand_landmarks.landmark[17].x
    if is_left_hand:
        if hand_landmarks.landmark[tip_ids[0]].x > hand_landmarks.landmark[tip_ids[0] - 1].x: fingers_up += 1
    else:
        if hand_landmarks.landmark[tip_ids[0]].x < hand_landmarks.landmark[tip_ids[0] - 1].x: fingers_up += 1
    for id in range(1, 5):
        if hand_landmarks.landmark[tip_ids[id]].y < hand_landmarks.landmark[tip_ids[id] - 2].y: fingers_up += 1
    return fingers_up

# ===============================================================
# UI BARU: TAMPILAN INDUSTRIAL
# ===============================================================
COLORS = {
    'bg': (45, 45, 45), 'text': (240, 240, 240),
    'safe': (0, 255, 0), 'warn': (0, 255, 255), 'danger': (0, 0, 255)
}

def draw_industrial_ui(frame, state, gesture_text, progress, total):
    h, w, _ = frame.shape
    overlay = frame.copy()

    # --- Panel Atas (Status Utama) ---
    cv2.rectangle(overlay, (0, 0), (w, 50), COLORS['bg'], -1)
    
    # --- Panel Bawah (Info & Instruksi) ---
    cv2.rectangle(overlay, (0, h - 70), (w, h), COLORS['bg'], -1)

    # Blend overlay dengan frame asli
    alpha = 0.6
    cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame)

    # --- Teks Status ---
    if state == ProgramState.WAITING_FOR_ACTIVATION:
        status_text = "STATUS: SIAGA"
        status_color = COLORS['warn']
        instruction = "GESTUR MENGEPAL: Mulai / Lanjutkan | Q: Keluar"
    elif state == ProgramState.RUNNING_SEQUENCE:
        status_text = "STATUS: MENJALANKAN SEKUENS"
        status_color = COLORS['safe']
        instruction = "GESTUR TELAPAK TERBUKA: Stop Darurat (E-Stop)"
    elif state == ProgramState.EMERGENCY_STOP:
        status_text = "STATUS: E-STOP AKTIF"
        status_color = COLORS['danger']
        instruction = "R: Siapkan Resume | Q: Keluar"

    cv2.putText(frame, status_text, (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 1, status_color, 2)
    
    # --- Teks Info di Panel Bawah ---
    progress_text = f"PROGRES: {progress}/{total}"
    gesture_info = f"GESTUR: {gesture_text}"
    cv2.putText(frame, progress_text, (10, h - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLORS['text'], 2)
    cv2.putText(frame, gesture_info, (10, h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLORS['text'], 2)
    cv2.putText(frame, instruction, (w - cv2.getTextSize(instruction, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0][0] - 10, h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, COLORS['warn'], 2)


# ===============================================================
# FUNGSI UTAMA (Dengan Logika Resume)
# ===============================================================

cap = None
hands = None
current_state = ProgramState.WAITING_FOR_ACTIVATION

def check_for_emergency_stop_gesture():
    # ... (Isi fungsi ini tetap sama) ...
    global cap, hands
    if not cap or not cap.isOpened(): return False
    success, frame = cap.read()
    if not success: return False
    results = hands.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    if results.multi_hand_landmarks:
        return count_fingers(results.multi_hand_landmarks[0]) == 5
    return False

def main():
    global cap, hands, current_state

    # --- Konfigurasi ---
    ROBOT_IP = "192.168.2.10"  
    ROBOT_USERNAME = "admin"
    ROBOT_PASSWORD = "admin"

    # --- Variabel Baru untuk Fitur Resume ---
    sequence_progress_index = 0
    total_poses = 14 # Update manual jumlah pose

    # --- Inisialisasi ---
    device = DeviceConnection(ROBOT_IP, ROBOT_USERNAME, ROBOT_PASSWORD)
    if not device.connect() or not device.ensure_control(): return

    mp_hands = mp.solutions.hands
    hands = mp_hands.Hands(min_detection_confidence=0.7, min_tracking_confidence=0.5, max_num_hands=1)
    mp_drawing = mp.solutions.drawing_utils
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Gagal membuka kamera."); device.disconnect(); return

    current_state = ProgramState.WAITING_FOR_ACTIVATION
    gesture_name = "TIDAK ADA TANGAN"
    
    try:
        print("Program siap. Arahkan tangan ke kamera.")
        
        while cap.isOpened():
            success, frame = cap.read()
            if not success: continue

            frame = cv2.flip(frame, 1)
            results = hands.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            
            num_fingers = -1
            if results.multi_hand_landmarks:
                hand_landmarks = results.multi_hand_landmarks[0]
                mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                num_fingers = count_fingers(hand_landmarks)
                if num_fingers == 0: gesture_name = "MENGEPAL"
                elif num_fingers == 5: gesture_name = "TELAPAK TERBUKA"
                else: gesture_name = f"{num_fingers} JARI"
            else:
                gesture_name = "TIDAK ADA TANGAN"
            
            # --- State Machine Logic ---
            if current_state == ProgramState.WAITING_FOR_ACTIVATION:
                if num_fingers == 0:
                    print("\nGESTUR AKTIVASI TERDETEKSI! Memulai/Melanjutkan sekuens...")
                    current_state = ProgramState.RUNNING_SEQUENCE
                    
                    # Panggil sekuens dengan indeks awal yang tersimpan
                    completed_successfully, sequence_progress_index = run_robot_sequence(device, start_index=sequence_progress_index)
                    
                    if completed_successfully:
                        print("\nSekuens selesai. Kembali ke mode siaga.")
                        sequence_progress_index = 0 # Reset progres jika selesai
                        current_state = ProgramState.WAITING_FOR_ACTIVATION
            
            elif current_state == ProgramState.RUNNING_SEQUENCE:
                if num_fingers == 5:
                    print("\n!!!! EMERGENCY STOP DIAKTIFKAN OLEH GESTUR !!!!")
                    device.base.Stop()
                    current_state = ProgramState.EMERGENCY_STOP
            
            # Gambar UI Industrial baru
            draw_industrial_ui(frame, current_state, gesture_name, sequence_progress_index, total_poses)
            cv2.imshow('Industrial Robot Control Panel', frame)

            key = cv2.waitKey(5) & 0xFF
            if key == ord('q'): break
            elif key == ord('r') and current_state == ProgramState.EMERGENCY_STOP:
                print("Tombol 'R' ditekan. Sistem siap untuk melanjutkan (resume).")
                print(f"Sekuens akan dilanjutkan dari langkah {sequence_progress_index + 1}.")
                current_state = ProgramState.WAITING_FOR_ACTIVATION

    except Exception as e:
        print(f"TERJADI KESALAHAN TAK TERDUGA: {e}")
    finally:
        device.disconnect()
        if cap: cap.release()
        cv2.destroyAllWindows()

def run_robot_sequence(device, start_index=0):
    global current_state 

    poses = [
        [0.001, 343.947, 75.115, 359.995, 299.972, 0.012], [2.988, 343.915, 46.886, 342.985, 298.639, 32.598],
        [325.385, 304.134, 349.85, 1.928, 269.991, 45.329], [325.304, 285.726, 350.125, 1.875, 268.318, 26.715],
        [325.305, 285.726, 350.125, 1.875, 268.319, 26.714], [335.091, 287.657, 353.143, 358.017, 275.178, 25.638],
        [335.092, 287.657, 353.142, 358.019, 275.179, 25.636], [335.513, 304.432, 352.035, 356.016, 275.455, 43.511],
        [45.077, 287.311, 347.981, 354.043, 283.503, 30.917], [89.814, 320.374, 94.673, 129.448, 305.821, 243.531],
        [89.091, 277.561, 80.278, 121.096, 282.6, 261.769], [89.091, 277.561, 80.277, 121.096, 282.6, 261.771],
        [89.356, 308.806, 95.751, 124.345, 295.845, 252.65], [0.004, 343.952, 75.111, 359.954, 299.972, 0.014],
    ]
    current_progress = start_index

    # Loop dari indeks awal, bukan selalu dari 0
    for i in range(start_index, len(poses)):
        current_progress = i # Simpan progres saat ini
        if current_state == ProgramState.EMERGENCY_STOP:
            return False, current_progress

        cv2.waitKey(1)
        
        step_number = i + 1
        print(f"\n--- Melanjutkan Langkah {step_number}/{len(poses)} ---")
        
        pose = poses[i] # Ambil pose berdasarkan indeks loop

        if step_number == 5: control_gripper(device, 0.0)
        
        result = device.move_to_joint_angles(pose, duration=4)
        if result == "FAIL": return False, current_progress
        if result == "E_STOP": return False, current_progress

        if step_number == 7: control_gripper(device, 1.0)
        if step_number == 12: control_gripper(device, 0.0)
        if step_number == len(poses): control_gripper(device, 1.0)
    
    # Jika loop selesai tanpa interupsi, kembalikan status sukses
    return True, len(poses)

if __name__ == "__main__":
    main()
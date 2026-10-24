import collections
import collections.abc

# Memanipulasi Python 3.11 agar terlihat seperti Python 3.9 untuk library lama
collections.MutableMapping = collections.abc.MutableMapping
collections.MutableSequence = collections.abc.MutableSequence
collections.Sequence = collections.abc.Sequence
collections.Iterable = collections.abc.Iterable
collections.Mapping = collections.abc.Mapping

import time
import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from mediapipe import Image, ImageFormat
from mediapipe.tasks.python.vision import drawing_utils
import math

# Pastikan Anda sudah menginstal Kortex API
# pip install kortex_api
# Kortex API imports - commented out as package not installed
try:
    from kortex_api.TCPTransport import TCPTransport  # type: ignore
    from kortex_api.RouterClient import RouterClient  # type: ignore
    from kortex_api.SessionManager import SessionManager  # type: ignore
    from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient  # type: ignore
    from kortex_api.autogen.messages import Session_pb2, Base_pb2  # type: ignore
    KORTEX_AVAILABLE = True
except (ImportError, AttributeError) as e:
    print("⚠️ Warning: kortex_api not available or incompatible with Python version.")
    print(f"   Error: {e}")
    print("   Robot control will be disabled. Using mock classes for debugging.")
    KORTEX_AVAILABLE = False
    
    # Define dummy classes for debugging without robot
    class TCPTransport:
        pass
    class RouterClient:
        def __init__(self, *args, **kwargs):
            pass
        @staticmethod
        def basicErrorCallback(*args):
            pass
    class SessionManager:
        def __init__(self, *args, **kwargs):
            pass
        def CreateSession(self, *args):
            pass
    class BaseClient:
        def __init__(self, *args, **kwargs):
            pass
        def ClearFaults(self):
            pass
        def SetServoingMode(self, *args):
            pass
        def SendJointSpeedsCommand(self, *args):
            pass
        def SendGripperCommand(self, *args):
            pass
    
    class Session_pb2:
        @staticmethod
        def CreateSessionInfo(**kwargs):
            return {}
    
    class Base_pb2:
        SINGLE_LEVEL_SERVOING = 0
        GRIPPER_POSITION = 0
        
        @staticmethod
        def ServoingModeInformation(**kwargs):
            return {}
        
        @staticmethod
        def JointSpeeds(**kwargs):
            return {}
        
        @staticmethod
        def JointSpeed(**kwargs):
            return {}
        
        @staticmethod
        def GripperCommand(**kwargs):
            return {}

# =============================================================================
# --- 1. KONFIGURASI DAN PENGATURAN GAYA (Gaya Profesional) ---
# =============================================================================
class Config:
    """Menyimpan semua konstanta dan pengaturan di satu tempat."""
    # --- Robot ---
    ROBOT_IP = "192.168.2.10"
    ROBOT_USERNAME = "admin"
    ROBOT_PASSWORD = "admin"
    MAX_JOINT_VELOCITY = 25.0 # untuk kecepatan gerakan sendi, 25 itu maximum
    NUM_JOINTS = 6 # ada 6 joints in a robot

    # --- MediaPipe ---
    DETECTION_CONFIDENCE = 0.7
    TRACKING_CONFIDENCE = 0.5
    MAX_HANDS = 1

    # --- Logika Gestur ---
    GESTURE_HOLD_FRAMES = 10 

    # --- Pengaturan Tampilan (UI Profesional - Steel Blue & Amber) ---
    FONT = cv2.FONT_HERSHEY_SIMPLEX 
    FONT_SCALE_MAIN = 0.8
    FONT_SCALE_SUB = 0.6
    THICKNESS = 2
    
    # Palet Warna Profesional (BGR)
    COLOR_BG = (35, 30, 30)          # Abu-abu sangat gelap
    COLOR_FRAME = (90, 80, 80)         # Abu-abu netral untuk bingkai
    COLOR_TEXT = (220, 220, 220)       # Putih pudar
    COLOR_ACCENT = (210, 180, 120)     # Steel Blue (Biru baja yang kalem)
    COLOR_INACTIVE = (30, 140, 210)    # Amber (Kuning tua/Oranye yang tidak norak)
    COLOR_INFO = (210, 180, 120)       # Menggunakan warna aksen untuk konsistensi
    
    UI_PADDING = 15 # Jarak dari tepi layar

# =============================================================================
# --- 2. KELAS KONEKSI ROBOT ---
# =============================================================================
class RobotConnection:
    """Mengelola koneksi dan setup awal untuk robot Kinova."""
    def __init__(self, ip, username, password):
        self.ip = ip
        self.username = username
        self.password = password
        self.session_manager = None
        self.base = None
        self.router = None
        self.transport = TCPTransport()

    def connect(self):
        """Membuat sesi koneksi dengan robot."""
        print(f"Mencoba menghubungkan ke robot di {self.ip}...")
        try:
            self.router = RouterClient(self.transport, RouterClient.basicErrorCallback)
            self.transport.connect(self.ip, 10000)
            session_info = Session_pb2.CreateSessionInfo(
                username=self.username,
                password=self.password,
                session_inactivity_timeout=60000
            )
            self.session_manager = SessionManager(self.router)
            self.session_manager.CreateSession(session_info)
            self.base = BaseClient(self.router)
            print("✅ Koneksi berhasil.")
            return True
        except Exception as e:
            print(f"❌ Koneksi gagal: {e}")
            return False

    def setup_for_control(self):
        """Mengatur robot ke mode servoing untuk kontrol real-time."""
        if not self.base: return False
        print("Mengambil alih kontrol robot...")
        try:
            self.base.ClearFaults()
            time.sleep(1)
            servoing_mode = Base_pb2.ServoingModeInformation(servoing_mode=Base_pb2.SINGLE_LEVEL_SERVOING)
            self.base.SetServoingMode(servoing_mode)
            time.sleep(1)
            print("✅ Robot siap dikontrol.")
            return True
        except Exception as e:
            print(f"❌ Gagal mengambil alih kontrol: {e}")
            return False

    def disconnect(self):
        """Menutup sesi koneksi dengan aman."""
        if self.session_manager:
            print("Menutup sesi...")
            self.session_manager.CloseSession()
        self.transport.disconnect()
        print("Koneksi ditutup.")

# =============================================================================
# --- 3. FUNGSI HELPER (Aksi Robot & Logika Gestur) ---
# =============================================================================
def send_joint_velocity_command(robot_base, joint_id, velocity):
    """Mengirim perintah kecepatan ke satu sendi robot."""
    if not robot_base: return
    joint_speeds = Base_pb2.JointSpeeds()
    joint_speed = joint_speeds.joint_speeds.add()
    joint_speed.joint_identifier = joint_id
    joint_speed.value = velocity
    joint_speed.duration = 0
    try:
        robot_base.SendJointSpeedsCommand(joint_speeds)
    except Exception as e:
        print(f"Error sending joint speed: {e}")

def send_gripper_command(robot_base, value):
    """Mengirim perintah posisi ke gripper (0.0 = Buka, 1.0 = Tutup)."""
    if not robot_base: return
    try:
        gripper_cmd = Base_pb2.GripperCommand(mode=Base_pb2.GRIPPER_POSITION)
        finger = gripper_cmd.gripper.finger.add()
        finger.finger_identifier = 1
        finger.value = value
        robot_base.SendGripperCommand(gripper_cmd)
        status = 'BUKA' if value == 0.0 else 'TUTUP'
        print(f"Perintah Gripper dikirim: {status}")
    except Exception as e:
        print(f"Error sending gripper command: {e}")
        
def count_fingers(hand_landmarks): 
    """Menghitung jumlah jari yang terangkat berdasarkan posisi landmark."""
    if not hand_landmarks: return 0
    fingers_up = 0
    tip_ids = [4, 8, 12, 16, 20]
    
    # Ibu Jari
    if (hand_landmarks[tip_ids[0]].x < hand_landmarks[tip_ids[0] - 1].x):
        fingers_up += 1
        
    # 4 Jari lainnya
    for id in range(1, 5):
        if hand_landmarks[tip_ids[id]].y < hand_landmarks[tip_ids[id] - 2].y:
            fingers_up += 1
            
    return fingers_up
    
# =============================================================================
# --- 4. MANAJEMEN TAMPILAN (UI/HUD - Gaya Profesional) ---
# =============================================================================
def draw_ui(frame, app_state):
    """Menggambar semua elemen UI bergaya profesional ke frame video."""
    h, w, _ = frame.shape
    pad = Config.UI_PADDING

    # --- Gambar Elemen Bingkai/Viewfinder ---
    corner_size = 30
    cv2.line(frame, (pad, pad), (pad + corner_size, pad), Config.COLOR_FRAME, Config.THICKNESS)
    cv2.line(frame, (pad, pad), (pad, pad + corner_size), Config.COLOR_FRAME, Config.THICKNESS)
    cv2.line(frame, (w - pad, pad), (w - pad - corner_size, pad), Config.COLOR_FRAME, Config.THICKNESS)
    cv2.line(frame, (w - pad, pad), (w - pad, pad + corner_size), Config.COLOR_FRAME, Config.THICKNESS)
    cv2.line(frame, (pad, h - pad), (pad + corner_size, h - pad), Config.COLOR_FRAME, Config.THICKNESS)
    cv2.line(frame, (pad, h - pad), (pad, h - pad - corner_size), Config.COLOR_FRAME, Config.THICKNESS)
    cv2.line(frame, (w - pad, h - pad), (w - pad - corner_size, h - pad), Config.COLOR_FRAME, Config.THICKNESS)
    cv2.line(frame, (w - pad, h - pad), (w - pad, h - pad - corner_size), Config.COLOR_FRAME, Config.THICKNESS)

    # --- Kotak Informasi Modular ---
    box_w = 280
    y_pos = pad + 20
    
    # 1. Kotak Status
    status_text = "AKTIF" if app_state['control_active'] else "NONAKTIF"
    status_color = Config.COLOR_ACCENT if app_state['control_active'] else Config.COLOR_INACTIVE
    cv2.rectangle(frame, (pad, y_pos), (pad + box_w, y_pos + 60), Config.COLOR_FRAME, 1)
    cv2.putText(frame, "STATUS", (pad + 10, y_pos + 20), Config.FONT, Config.FONT_SCALE_SUB, Config.COLOR_TEXT, 1)
    cv2.putText(frame, status_text, (pad + 120, y_pos + 45), Config.FONT, Config.FONT_SCALE_MAIN, status_color, Config.THICKNESS)
    y_pos += 70

    # 2. Kotak Sendi
    cv2.rectangle(frame, (pad, y_pos), (pad + box_w, y_pos + 60), Config.COLOR_FRAME, 1)
    cv2.putText(frame, "SENDI AKTIF", (pad + 10, y_pos + 20), Config.FONT, Config.FONT_SCALE_SUB, Config.COLOR_TEXT, 1)
    cv2.putText(frame, str(app_state['selected_joint_id'] + 1), (pad + 120, y_pos + 45), Config.FONT, 1.2, Config.COLOR_ACCENT, Config.THICKNESS)
    y_pos += 70

    # 3. Kotak Gripper
    gripper_status = "TERBUKA" if app_state['gripper_is_open'] else "TERTUTUP"
    cv2.rectangle(frame, (pad, y_pos), (pad + box_w, y_pos + 60), Config.COLOR_FRAME, 1)
    cv2.putText(frame, "GRIPPER", (pad + 10, y_pos + 20), Config.FONT, Config.FONT_SCALE_SUB, Config.COLOR_TEXT, 1)
    cv2.putText(frame, gripper_status, (pad + 120, y_pos + 45), Config.FONT, Config.FONT_SCALE_MAIN, Config.COLOR_INFO, Config.THICKNESS)
    y_pos += 70

    # 4. Kotak Gestur
    cv2.rectangle(frame, (pad, y_pos), (pad + box_w, y_pos + 60), Config.COLOR_FRAME, 1)
    cv2.putText(frame, "GESTUR DETEKSI", (pad + 10, y_pos + 20), Config.FONT, Config.FONT_SCALE_SUB, Config.COLOR_TEXT, 1)
    cv2.putText(frame, app_state['gesture_text'], (pad + 10, y_pos + 45), Config.FONT, 0.7, Config.COLOR_INACTIVE, 1)
    y_pos += 70

    # --- Pesan Feedback ---
    if app_state['feedback_timer'] > 0:
        cv2.rectangle(frame, (pad, y_pos), (pad + box_w, y_pos + 30), Config.COLOR_ACCENT, -1)
        feedback_msg = app_state['feedback_text']
        text_size = cv2.getTextSize(feedback_msg, Config.FONT, 0.7, 1)[0]
        text_x = pad + (box_w - text_size[0]) // 2
        cv2.putText(frame, feedback_msg, (text_x, y_pos + 22), Config.FONT, 0.7, Config.COLOR_BG, 1)
        app_state['feedback_timer'] -= 1

    # --- Teks Bantuan Bawah ---
    cv2.putText(frame, "'S': Start/Stop | 'Q': Keluar | 'H': Bantuan", (pad, h - 20), Config.FONT, Config.FONT_SCALE_SUB, Config.COLOR_TEXT, 1)

    # --- Panel Bantuan ---
    if app_state['show_help']:
        draw_help_panel(frame)

def draw_help_panel(frame):
    """Menggambar panel bantuan bergaya profesional."""
    h, w, _ = frame.shape
    help_w, help_h = 500, 220
    x1 = (w - help_w) // 2
    y1 = (h - help_h) // 2
    
    overlay = frame.copy()
    cv2.rectangle(overlay, (x1, y1), (x1 + help_w, y1 + help_h), Config.COLOR_BG, -1)
    cv2.addWeighted(overlay, 0.8, frame, 0.2, 0, frame)
    cv2.rectangle(frame, (x1, y1), (x1 + help_w, y1 + help_h), Config.COLOR_FRAME, 2)

    cv2.putText(frame, "[ PANDUAN GESTUR ]", (x1 + 100, y1 + 40), Config.FONT, 0.9, Config.COLOR_ACCENT, 2)
    cv2.line(frame, (x1 + 20, y1 + 60), (x1 + help_w - 20, y1 + 60), Config.COLOR_FRAME, 1)
    
    instructions = {
        "GANTI SENDI": "2 JARI (PEACE)",
        "BUKA/TUTUP GRIPPER": "4 JARI",
        "GERAKKAN SENDI": "KEPAL / 1 JARI",
        "HENTIKAN GERAKAN": "5 JARI (TERBUKA)",
    }
    y_offset = y1 + 100
    for action, gesture in instructions.items():
        cv2.putText(frame, f"{action}:", (x1 + 40, y_offset), Config.FONT, 0.7, Config.COLOR_TEXT, 1)
        cv2.putText(frame, gesture, (x1 + 300, y_offset), Config.FONT, 0.7, Config.COLOR_ACCENT, 1)
        y_offset += 30

# =============================================================================
# --- 5. FUNGSI UTAMA (main) ---
# =============================================================================
def main(): 
    """Fungsi utama untuk menjalankan seluruh program."""
    robot = RobotConnection(Config.ROBOT_IP, Config.ROBOT_USERNAME, Config.ROBOT_PASSWORD)
    if not robot.connect() or not robot.setup_for_control():
        return

    BaseOptions = mp.tasks.BaseOptions
    HandLandmarker = mp.tasks.vision.HandLandmarker
    HandLandmarkerOptions = mp.tasks.vision.HandLandmarkerOptions
    VisionRunningMode = mp.tasks.vision.RunningMode

    options = HandLandmarkerOptions(
        base_options=BaseOptions(
            model_asset_path="hand_landmarker.task"
        ),
        running_mode=VisionRunningMode.IMAGE,
        num_hands=Config.MAX_HANDS,
        min_hand_detection_confidence=Config.DETECTION_CONFIDENCE,
        min_hand_presence_confidence=0.5,
        min_tracking_confidence=Config.TRACKING_CONFIDENCE
    )

    hands = HandLandmarker.create_from_options(options)
    mp_drawing = drawing_utils
    cap = cv2.VideoCapture(0)

    app_state = {
        'control_active': False,
        'selected_joint_id': 0,
        'gripper_is_open': True,
        'gesture_text': "TIDAK ADA TANGAN",
        'show_help': True,
        'feedback_text': "",
        'feedback_timer': 0,
    }
    
    gesture_buffer = []
    action_taken_flags = {'peace': False, 'four_finger': False}
    
    print_instructions()

    try:
        while cap.isOpened():
            success, frame = cap.read()
            if not success: continue

            frame = cv2.flip(frame, 1)
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_image = Image(
                image_format=ImageFormat.SRGB,
                data=frame_rgb
            )
            results = hands.detect(mp_image)

            hand_landmarks = results.hand_landmarks[0] if results.hand_landmarks else None
            total_fingers = count_fingers(hand_landmarks)
            
            if hand_landmarks:
                drawing_utils.draw_landmarks(frame, hand_landmarks, mp.tasks.vision.HandLandmarksConnections.HAND_CONNECTIONS)
                
                if total_fingers != 2: action_taken_flags['peace'] = False
                if total_fingers != 4: action_taken_flags['four_finger'] = False

                if app_state['control_active']:
                    gesture_buffer.append(total_fingers)
                    if len(gesture_buffer) > Config.GESTURE_HOLD_FRAMES:
                        gesture_buffer.pop(0)
                    
                    if len(gesture_buffer) == Config.GESTURE_HOLD_FRAMES and all(g == gesture_buffer[0] for g in gesture_buffer):
                        consistent_gesture = gesture_buffer[0]
                        
                        if consistent_gesture == 2 and not action_taken_flags['peace']:
                            app_state['selected_joint_id'] = (app_state['selected_joint_id'] + 1) % Config.NUM_JOINTS
                            app_state['feedback_text'] = f"Sendi {app_state['selected_joint_id'] + 1} Terpilih!"
                            app_state['feedback_timer'] = 30
                            action_taken_flags['peace'] = True
                        
                        elif consistent_gesture == 4 and not action_taken_flags['four_finger']:
                            app_state['gripper_is_open'] = not app_state['gripper_is_open']
                            gripper_value = 0.0 if app_state['gripper_is_open'] else 1.0
                            send_gripper_command(robot.base, gripper_value)
                            status_text = "Dibuka" if app_state['gripper_is_open'] else "Ditutup"
                            app_state['feedback_text'] = f"Gripper {status_text}!"
                            app_state['feedback_timer'] = 30
                            action_taken_flags['four_finger'] = True

                    if total_fingers <= 1:
                        app_state['gesture_text'] = f"GERAKKAN SENDI {app_state['selected_joint_id'] + 1}"
                        wrist_x_norm = (hand_landmarks[0].x - 0.5) * 2
                        joint_velocity = -1 * Config.MAX_JOINT_VELOCITY * wrist_x_norm
                        send_joint_velocity_command(robot.base, app_state['selected_joint_id'], joint_velocity)
                    else:
                        app_state['gesture_text'] = f"{total_fingers} JARI (DIAM)"
                        send_joint_velocity_command(robot.base, app_state['selected_joint_id'], 0)
                else:
                    app_state['gesture_text'] = f"{total_fingers} JARI (NONAKTIF)"
            else:
                app_state['gesture_text'] = "TIDAK ADA TANGAN"
                if app_state['control_active']:
                    send_joint_velocity_command(robot.base, app_state['selected_joint_id'], 0)
                gesture_buffer.clear()

            draw_ui(frame, app_state)
            cv2.imshow('Kontrol Robot Kinova via Gestur Tangan', frame)

            key = cv2.waitKey(5) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                app_state['control_active'] = not app_state['control_active']
                if not app_state['control_active']:
                    print("Kontrol dinonaktifkan. Menghentikan semua gerakan.")
                    robot.base.Stop()
                else:
                    print("Kontrol diaktifkan.")
            elif key == ord('h'):
                app_state['show_help'] = not app_state['show_help']

    finally:
        print("Membersihkan sumber daya...")
        if robot and robot.base:
            robot.base.Stop()
        if robot:
            robot.disconnect()
        hands.close()
        cap.release()
        cv2.destroyAllWindows()
        print("Program selesai.")

def print_instructions():
    """Mencetak instruksi awal ke konsol."""
    print("\n" + "="*50)
    print("PROGRAM KONTROL GESTUR TANGAN UNTUK KINOVA GEN3 LITE")
    print("="*50)
    print("Tekan di jendela OpenCV:")
    print("  'S' : Untuk START/STOP mengaktifkan kontrol gestur.")
    print("  'H' : Untuk menampilkan/menyembunyikan panduan di layar.")
    print("  'Q' : Untuk keluar dari program.")
    print("\nRobot akan mulai merespon gestur setelah Anda menekan 'S'.")
    print("="*50 + "\n")

if __name__ == "__main__":
    main()
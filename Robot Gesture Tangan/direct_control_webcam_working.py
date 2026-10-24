import time
import cv2
import math

# Import Kortex API
from kortex_api.TCPTransport import TCPTransport
from kortex_api.RouterClient import RouterClient
from kortex_api.SessionManager import SessionManager
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.messages import Session_pb2, Base_pb2, Common_pb2

# --- Kelas Koneksi Robot ---
class RobotConnection:
  
    def __init__(self, ip, username, password):
        self.ip = ip
        self.username = username
        self.password = password
        self.session_manager = None
        self.base = None

    def connect(self):
        print(f"Mencoba menghubungkan ke robot di {self.ip}...")
        try:
            transport = TCPTransport()
            router = RouterClient(transport, RouterClient.basicErrorCallback)
            transport.connect(self.ip, 10000)
            self.session_manager = SessionManager(router)
            session_info = Session_pb2.CreateSessionInfo()
            session_info.username = self.username
            session_info.password = self.password
            session_info.session_inactivity_timeout = 60000
            self.session_manager.CreateSession(session_info)
            self.base = BaseClient(router)
            print("Koneksi berhasil.")
            return True
        except Exception as e:
            print(f"Koneksi gagal: {e}")
            return False

    def setup_for_control(self):
        print("Mengambil alih kontrol robot...")
        try:
            self.base.ClearFaults()
            time.sleep(1)
            servoing_mode = Base_pb2.ServoingModeInformation()
            servoing_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
            self.base.SetServoingMode(servoing_mode)
            time.sleep(1)
            print("Robot siap dikontrol.")
            return True
        except Exception as e:
            print(f"Gagal mengambil alih kontrol: {e}")
            return False

    def disconnect(self):
        if self.session_manager:
            print("Menutup sesi...")
            self.session_manager.CloseSession()
        print("Koneksi ditutup.")


# --- Fungsi Helper untuk Aksi Robot ---
def send_joint_velocity_command(robot_base, joint_id, velocity):
    joint_speeds = Base_pb2.JointSpeeds()
    joint_speed = joint_speeds.joint_speeds.add()
    joint_speed.joint_identifier = joint_id
    joint_speed.value = velocity
    joint_speed.duration = 0
    try:
        robot_base.SendJointSpeedsCommand(joint_speeds)
    except Exception as e:
        print(f"Error sending joint speed command: {e}")
        
def send_gripper_command(robot_base, value):
    try:
        gripper_cmd = Base_pb2.GripperCommand()
        gripper_cmd.mode = Base_pb2.GRIPPER_POSITION
        finger = gripper_cmd.gripper.finger.add()
        finger.finger_identifier = 1
        finger.value = value
        robot_base.SendGripperCommand(gripper_cmd)
        print(f"Perintah Gripper dikirim: {'BUKA' if value == 0.0 else 'TUTUP'}")
    except Exception as e:
        print(f"Error sending gripper command: {e}")

# --- Fungsi Utama ---
def main():
    # --- Konfigurasi ---
    ROBOT_IP = "192.168.2.10"
    ROBOT_USERNAME = "admin"
    ROBOT_PASSWORD = "admin"
    MAX_JOINT_VELOCITY = 20.0 

    # --- Inisialisasi Robot ---
    robot = RobotConnection(ROBOT_IP, ROBOT_USERNAME, ROBOT_PASSWORD)
    if not robot.connect() or not robot.setup_for_control():
        return

    # --- Inisialisasi Webcam ---
    cap = cv2.VideoCapture(0)

    # --- Variabel State Aplikasi ---
    control_active = False
    selected_joint_id = 0
    gripper_is_open = True
    
    print_instructions()

    try:
        while cap.isOpened():
            success, frame = cap.read()
            if not success: 
                continue

            frame = cv2.flip(frame, 1)
            h, w, _ = frame.shape

            # Simple placeholder for gesture detection
            gesture_text = "CAMERA AKTIF (tanpa gesture detection)"
            
            # Draw info on screen
            draw_info_on_screen(frame, control_active, gesture_text, selected_joint_id, gripper_is_open)
            cv2.imshow('Kontrol Robot', frame)

            key = cv2.waitKey(5) & 0xFF
            if key == ord('q'): 
                break
            elif key == ord('s'): 
                control_active = not control_active
                if not control_active:
                    send_joint_velocity_command(robot.base, selected_joint_id, 0)
                    robot.base.Stop()
            elif key == ord('1'):  # Joint 1
                selected_joint_id = 0
                print(f"Sendi Aktif: 1")
            elif key == ord('2'):  # Joint 2
                selected_joint_id = 1
                print(f"Sendi Aktif: 2")
            elif key == ord('3'):  # Joint 3
                selected_joint_id = 2
                print(f"Sendi Aktif: 3")
            elif key == ord('4'):  # Joint 4
                selected_joint_id = 3
                print(f"Sendi Aktif: 4")
            elif key == ord('5'):  # Joint 5
                selected_joint_id = 4
                print(f"Sendi Aktif: 5")
            elif key == ord('6'):  # Joint 6
                selected_joint_id = 5
                print(f"Sendi Aktif: 6")
            elif key == ord('g'):  # Toggle Gripper
                gripper_is_open = not gripper_is_open
                gripper_value = 0.0 if gripper_is_open else 1.0
                send_gripper_command(robot.base, gripper_value)
            elif key == ord('a'):  # Move left
                if control_active:
                    send_joint_velocity_command(robot.base, selected_joint_id, MAX_JOINT_VELOCITY * 0.5)
            elif key == ord('d'):  # Move right
                if control_active:
                    send_joint_velocity_command(robot.base, selected_joint_id, -MAX_JOINT_VELOCITY * 0.5)
            elif key == ord(' '):  # Stop
                if control_active:
                    send_joint_velocity_command(robot.base, selected_joint_id, 0)

    finally:
        cleanup(robot, cap, selected_joint_id)

def print_instructions():
    print("\n" + "="*60)
    print("PROGRAM KONTROL ROBOT - MODE KEYBOARD")
    print("="*60)
    print("\nKONTROL DASAR:")
    print("S: START/STOP kontrol")
    print("Q: KELUAR dari program")
    print("\nPILIH SENDI (Joint):")
    print("1-6: Pilih sendi 1-6")
    print("\nGERAKAN SENDI:")
    print("A: Gerakkan ke kiri")
    print("D: Gerakkan ke kanan")
    print("SPACE: Hentikan gerakan")
    print("\nGRIPPER:")
    print("G: Buka/Tutup Gripper (Toggle)")
    print("="*60 + "\n")

def draw_info_on_screen(frame, active, gesture, active_joint_id, gripper_open):
    h, w, _ = frame.shape
    status_text = "AKTIF" if active else "NONAKTIF"
    status_color = (0, 255, 0) if active else (0, 0, 255)
    
    cv2.putText(frame, f"STATUS: {status_text}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, status_color, 2)
    cv2.putText(frame, f"SENDI AKTIF: {active_joint_id + 1}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 255), 2)
    
    gripper_status_text = "TERBUKA" if gripper_open else "TERTUTUP"
    cv2.putText(frame, f"GRIPPER: {gripper_status_text}", (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 0), 2)
    
    cv2.putText(frame, f"STATUS: {gesture}", (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 1)
    cv2.putText(frame, "S: Start/Stop | Q: Quit | 1-6: Joint | A/D: Move", (10, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

def cleanup(robot, cap, active_joint_id):
    print("\nMembersihkan...")
    if robot and robot.base:
        send_joint_velocity_command(robot.base, active_joint_id, 0)
        robot.base.Stop()
    if robot:
        robot.disconnect()
    cap.release()
    cv2.destroyAllWindows()
    print("Program selesai.")

if __name__ == "__main__":
    main()
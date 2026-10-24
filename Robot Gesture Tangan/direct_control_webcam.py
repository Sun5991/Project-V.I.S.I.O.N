import time
import cv2
import math
import numpy as np

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

# --- Fungsi Deteksi Tangan dengan OpenCV ---
def detect_hand_simple(frame):
    """
    Simple hand detection using skin color detection
    Improved for better detection across different skin tones
    Returns: (hand_detected, hand_center_x_norm, hand_contour)
    """
    # Convert to HSV for skin detection
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Multiple skin color ranges for better detection across skin tones
    # Lower range (reddish tones)
    lower_skin1 = np.array([0, 10, 60], dtype=np.uint8)
    upper_skin1 = np.array([20, 255, 255], dtype=np.uint8)
    
    # Higher range (more orange/yellow tones)
    lower_skin2 = np.array([160, 10, 60], dtype=np.uint8)
    upper_skin2 = np.array([180, 255, 255], dtype=np.uint8)
    
    # Create masks for both ranges
    mask1 = cv2.inRange(hsv, lower_skin1, upper_skin1)
    mask2 = cv2.inRange(hsv, lower_skin2, upper_skin2)
    
    # Combine masks
    mask = cv2.bitwise_or(mask1, mask2)
    
    # Morphological operations to improve mask
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    
    # Apply Gaussian blur to smooth
    mask = cv2.GaussianBlur(mask, (5, 5), 0)
    
    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    if len(contours) == 0:
        return False, 0, None
    
    # Get largest contour (hand)
    largest_contour = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest_contour)
    
    if area < 800:  # Minimum hand size (increased from 500)
        return False, 0, None
    
    # Filter by contour shape
    perimeter = cv2.arcLength(largest_contour, True)
    circularity = 4 * np.pi * area / (perimeter ** 2)
    
    # Hand should have reasonable circularity (not too round, not too thin)
    if circularity < 0.2:
        return False, 0, None
    
    # Get hand center
    M = cv2.moments(largest_contour)
    if M["m00"] > 0:
        hand_x = M["m10"] / M["m00"]
        hand_y = M["m01"] / M["m00"]
        h, w = frame.shape[:2]
        hand_x_norm = (hand_x / w) - 0.5  # Normalize to [-0.5, 0.5]
        
        return True, hand_x_norm, largest_contour
    
    return False, 0, largest_contour

def count_fingers_simple(contour, frame):
    """
    Count fingers using convex hull defects analysis
    More reliable than contour vertex counting
    Returns: (num_fingers, is_fist)
    """
    # Get convex hull
    hull = cv2.convexHull(contour)
    
    # Get convex defects
    if len(hull) > 3:
        defects = cv2.convexityDefects(contour, cv2.convexHull(contour, returnPoints=False))
    else:
        return 0, True  # Too small, likely a fist
    
    # If no defects, it's likely a fist
    if defects is None:
        return 0, True
    
    # Count significant defects (valleys between fingers)
    finger_count = 0
    
    for i in range(defects.shape[0]):
        s, e, f, d = defects[i, 0]
        start = tuple(contour[s][0])
        end = tuple(contour[e][0])
        far = tuple(contour[f][0])
        
        # Calculate angle at defect point
        a = np.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
        b = np.sqrt((far[0] - start[0])**2 + (far[1] - start[1])**2)
        c = np.sqrt((end[0] - far[0])**2 + (end[1] - far[1])**2)
        
        if (a + b) == 0:
            continue
            
        angle = np.arccos((a**2 + b**2 - c**2) / (2*a*b))
        
        # Ignore if angle is too large (not a real finger valley)
        if angle < np.pi / 2:  # 90 degrees
            finger_count += 1
    
    # Add 1 to defect count since there's always one more finger than defects
    num_fingers = finger_count + 1
    
    # Limit to reasonable finger count
    if num_fingers > 5:
        num_fingers = 5
    
    # Determine if it's a fist (0 fingers detected)
    is_fist = (num_fingers == 1 and finger_count == 0)
    
    # If very few fingers detected, likely a fist or partially closed hand
    if num_fingers < 2:
        return 0, True
    elif num_fingers == 2:
        return 2, False  # Peace sign
    elif num_fingers <= 3:
        return 3, False
    elif num_fingers <= 4:
        return 4, False
    else:
        return 5, False  # Open hand

# --- Fungsi Utama ---
def main():
    # --- Konfigurasi ---
    ROBOT_IP = "192.168.2.10"
    ROBOT_USERNAME = "admin"
    ROBOT_PASSWORD = "admin"
    MAX_JOINT_VELOCITY = 20.0 

    # --- Inisialisasi Robot ---
    robot = RobotConnection(ROBOT_IP, ROBOT_USERNAME, ROBOT_PASSWORD)
    robot_connected = robot.connect() and robot.setup_for_control()
    if not robot_connected:
        print("PERINGATAN: Robot tidak terhubung, menjalankan dengan mode simulasi (camera only)")
    
    # --- Inisialisasi Webcam ---
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("ERROR: Tidak bisa membuka webcam")
        if robot_connected:
            robot.disconnect()
        return

    # --- Variabel State Aplikasi ---
    control_active = False
    selected_joint_id = 0
    gripper_is_open = True
    gesture_buffer = []
    GESTURE_HOLD_FRAMES = 15
    peace_action_taken = False
    four_finger_action_taken = False
    
    print_instructions()

    try:
        while cap.isOpened():
            success, frame = cap.read()
            if not success: 
                continue

            frame = cv2.flip(frame, 1)
            h, w, c = frame.shape
            
            # Simple hand detection
            hand_detected, hand_x_norm, hand_contour = detect_hand_simple(frame)

            total_fingers = 0
            gesture_text = "TIDAK ADA TANGAN"
            clutch_engaged = False
            is_fist = False

            if hand_detected and hand_contour is not None:
                # Draw hand contour
                cv2.drawContours(frame, [hand_contour], 0, (0, 255, 0), 2)
                
                # Draw convex hull
                hull = cv2.convexHull(hand_contour)
                cv2.drawContours(frame, [hull], 0, (0, 0, 255), 1)
                
                # Count fingers
                total_fingers, is_fist = count_fingers_simple(hand_contour, frame)
                gesture_text = f"{total_fingers} JARI"
                
                # Draw hand center
                M = cv2.moments(hand_contour)
                if M["m00"] > 0:
                    hand_cx = int(M["m10"] / M["m00"])
                    hand_cy = int(M["m01"] / M["m00"])
                    cv2.circle(frame, (hand_cx, hand_cy), 5, (255, 0, 255), -1)
                
                # Reset flags if gesture is no longer held
                if total_fingers != 2: 
                    peace_action_taken = False
                if total_fingers != 4: 
                    four_finger_action_taken = False

                if control_active:
                    # Check for consistent gestures
                    gesture_buffer.append(total_fingers)
                    if len(gesture_buffer) > GESTURE_HOLD_FRAMES: 
                        gesture_buffer.pop(0)
                    
                    # Check if gesture is held consistently
                    if len(gesture_buffer) == GESTURE_HOLD_FRAMES and all(g == gesture_buffer[0] for g in gesture_buffer):
                        consistent_gesture = gesture_buffer[0]
                        
                        # Gesture 2 fingers (PEACE) to select next joint
                        if consistent_gesture == 2 and not peace_action_taken:
                            selected_joint_id = (selected_joint_id + 1) % 6
                            gesture_buffer.clear()
                            print(f"✌️  GESTUR PEACE: Sendi Aktif diubah ke: {selected_joint_id + 1}")
                            peace_action_taken = True
                        
                        # Gesture 4 fingers to toggle gripper
                        elif consistent_gesture == 4 and not four_finger_action_taken:
                            gripper_is_open = not gripper_is_open
                            gripper_value = 0.0 if gripper_is_open else 1.0
                            if robot_connected:
                                send_gripper_command(robot.base, gripper_value)
                            else:
                                print(f"🖖 Gripper (simulasi): {'BUKA' if gripper_is_open else 'TUTUP'}")
                            gesture_buffer.clear()
                            four_finger_action_taken = True
                    
                    # Check gesture for joint movement (fist or 0-1 fingers = clutch engaged)
                    if is_fist or total_fingers <= 1: 
                        clutch_engaged = True
                        gesture_text = f"✊ MENGGERAKKAN SENDI {selected_joint_id + 1}"
                        
                        # Use hand X position for velocity control
                        joint_velocity = -1 * MAX_JOINT_VELOCITY * (hand_x_norm / 0.5)
                        joint_velocity = max(-MAX_JOINT_VELOCITY, min(MAX_JOINT_VELOCITY, joint_velocity))
                        
                        if robot_connected:
                            send_joint_velocity_command(robot.base, selected_joint_id, joint_velocity)
                    else:
                        gesture_text = f"🖐️ CLUTCH DILEPAS ({total_fingers} JARI)"
            else:
                # Reset when no hand is detected
                gesture_buffer.clear()
                peace_action_taken = False
                four_finger_action_taken = False

            # Stop joint movement if clutch is released
            if control_active and not clutch_engaged and robot_connected:
                send_joint_velocity_command(robot.base, selected_joint_id, 0)

            # Draw info on screen
            draw_info_on_screen(frame, control_active, gesture_text, selected_joint_id, gripper_is_open, robot_connected)
            cv2.imshow('Kontrol Gestur Langsung', frame)

            key = cv2.waitKey(5) & 0xFF
            if key == ord('q'): 
                break
            elif key == ord('s'): 
                control_active = not control_active
                if not control_active and robot_connected:
                    send_joint_velocity_command(robot.base, selected_joint_id, 0)
                    robot.base.Stop()
                print(f"Kontrol {'DIMULAI' if control_active else 'DIHENTIKAN'}")
                gesture_buffer.clear()

    finally:
        cleanup(robot, cap, selected_joint_id, robot_connected)

def print_instructions():
    print("\n" + "="*70)
    print("PROGRAM KONTROL ROBOT - MODE GESTURE TANGAN")
    print("="*70)
    print("\nKONTROL DASAR:")
    print("  S (Keyboard): START/STOP kontrol gestur")
    print("  Q (Keyboard): KELUAR dari program")
    print("\nGESTUR TANGAN (Tahan sejenak untuk aktivasi):")
    print("  ✌️  2 JARI (PEACE)  : Pilih sendi berikutnya (1 → 2 → ... → 6 → 1)")
    print("  🖖 4 JARI          : Buka/Tutup Gripper (Toggle)")
    print("  ✊ 0-1 JARI (KEPAL) : Gerakkan sendi yang sedang aktif")
    print("                       Posisi tangan X = Arah gerakan")
    print("  🖐️ 5 JARI (BUKA)   : Hentikan gerakan (Release clutch)")
    print("\nTIPE DETEKSI:")
    print("  - Menggunakan OpenCV Hand Detection (skin color + contour analysis)")
    print("  - Cocok untuk berbagai kondisi pencahayaan dan warna kulit")
    print("\nCATA TAN:")
    print("  - Tekan S untuk mengaktifkan kontrol gestur")
    print("  - Tahan gestur selama ~15 frame untuk aktivasi (≈0.3 detik)")
    print("  - Pastikan tangan terlihat jelas di kamera")
    print("  - Hijau = kontur tangan, Merah = convex hull, Magenta = pusat tangan")
    print("="*70 + "\n")

def count_fingers(hand_landmarks):
    """Legacy finger counting function - kept for reference but not used with cvzone"""
    fingers_up = 0
    tip_ids = [4, 8, 12, 16, 20]
    # Ibu jari
    if (hand_landmarks.landmark[tip_ids[0]].x < hand_landmarks.landmark[tip_ids[0] - 1].x):
        fingers_up += 1
    # 4 jari lain
    for id in range(1, 5):
        if hand_landmarks.landmark[tip_ids[id]].y < hand_landmarks.landmark[tip_ids[id] - 2].y:
            fingers_up += 1
    return fingers_up

def draw_info_on_screen(frame, active, gesture, active_joint_id, gripper_open, connected):
    h, w, _ = frame.shape
    status_text = "AKTIF" if active else "NONAKTIF"
    status_color = (0, 255, 0) if active else (0, 0, 255)
    
    connection_text = "TERHUBUNG" if connected else "SIMULASI"
    connection_color = (0, 255, 0) if connected else (0, 165, 255)
    
    cv2.putText(frame, f"KONEKSI: {connection_text}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, connection_color, 2)
    cv2.putText(frame, f"STATUS: {status_text}", (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.9, status_color, 2)
    cv2.putText(frame, f"SENDI: {active_joint_id + 1}", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 255), 2)
    
    gripper_status_text = "BUKA" if gripper_open else "TUTUP"
    cv2.putText(frame, f"GRIPPER: {gripper_status_text}", (10, 135), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 0), 2)
    
    # Large gesture text in center
    cv2.putText(frame, gesture, (w//2 - 150, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 3)
    
    # Legend for visual markers
    cv2.putText(frame, "Hijau=Tangan  Merah=Hull  Magenta=Pusat", 
                (10, h - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
    
    # Gesture guide
    cv2.putText(frame, "S:Start/Stop | Q:Quit", 
                (10, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

def cleanup(robot, cap, active_joint_id, connected):
    print("\n" + "="*70)
    print("Membersihkan...")
    if connected and robot and robot.base:
        try:
            send_joint_velocity_command(robot.base, active_joint_id, 0)
            robot.base.Stop()
        except:
            pass
    if robot and connected:
        robot.disconnect()
    cap.release()
    cv2.destroyAllWindows()
    print("Program selesai.")
    print("="*70)

if __name__ == "__main__":
    main()
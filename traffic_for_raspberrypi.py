#!/usr/bin/env python3
# ████████████████████████████████████████████████████████████████████████████
# PROJECT  : Dynamic Green Light Traffic System
# FILE     : traffic_pi.py
# DEVICE   : Raspberry Pi
# ROLE     : YOLOv8 Object Detection + UDP Communication with ESP32 Master
# VERSION  : 1.0
# ████████████████████████████████████████████████████████████████████████████
#
# INSTALL DEPENDENCIES:
#   pip install ultralytics opencv-python
#
# ARCHITECTURE:
#   USB Webcam → Raspberry Pi (YOLOv8) → UDP → ESP32 Master
#
# FLOW:
#   Pi boots → sends "CAM:READY" to Master
#   Master sends "CAPTURE:WEST" → Pi captures + detects → sends CNT:WEST,...
#   Master sends "ACK:WEST,GT:45" → Pi waits for next command
#   Master sends "PATROL:WEST,DUR:45" → Pi checks ROI violations
#   Master sends "STOP_PATROL" → Pi stops patrol
#
# ████████████████████████████████████████████████████████████████████████████


import socket
import cv2
import time
from ultralytics import YOLO


# ═══════════════════════════════════════════════════════════════════════════
# SECTION 1: CONFIGURATION — CHANGE THESE
# ═══════════════════════════════════════════════════════════════════════════

MASTER_IP       = "192.168.53.51"  # <<< CHANGE to ESP32 Master IP
MASTER_PORT     = 4210             # Master listens here
PI_PORT         = 4211             # Pi listens here

# ── Camera ───────────────────────────────────────────────────────────────────
WEBCAM_INDEX    = 0                # USB webcam index (0 = first webcam)
FRAME_WIDTH     = 640
FRAME_HEIGHT    = 480

# ── YOLOv8 Model ─────────────────────────────────────────────────────────────
MODEL_PATH      = "best.onnx"      # <<< Path to your trained YOLOv8 model
CONFIDENCE      = 0.50             # Detection confidence threshold (0.0-1.0)

# ── Class Names — Matches your YOLOv8 training labels exactly ────────────────
# ambulance, fire truck, car, threewheel, bus, truck, motorbike, van, pedestrian
CLASS_NAMES = {
    "ambulance"  : "AM",   # → emergency flag
    "fire truck" : "FT",   # → emergency flag
    "car"        : "CR",   # → count
    "threewheel" : "TW",   # → count (tuk-tuk/auto)
    "bus"        : "BS",   # → count
    "truck"      : "TK",   # → count
    "motorbike"  : "MB",   # → count
    "van"        : "VN",   # → count
    "pedestrian" : "PD",   # → flag
}

# ── ROI for violation detection (bottom 15% of frame) ────────────────────────
ROI_RATIO       = 0.85             # ROI starts at 85% height of frame
ROI_DARK_RATIO  = 0.25             # Dark pixel ratio threshold for violation

# ── ACK/Retry ────────────────────────────────────────────────────────────────
ACK_TIMEOUT     = 2.0              # seconds to wait for ACK
MAX_RETRIES     = 3                # retransmit attempts

# ── Patrol ───────────────────────────────────────────────────────────────────
PATROL_INTERVAL = 4.5              # seconds between patrol checks

# ── Debug ────────────────────────────────────────────────────────────────────
DEBUG           = True

def dbg(msg):
    if DEBUG:
        print(f"[{time.strftime('%H:%M:%S')}] {msg}")


# ═══════════════════════════════════════════════════════════════════════════
# SECTION 2: GLOBAL VARIABLES
# ═══════════════════════════════════════════════════════════════════════════

sock        = None   # UDP socket
cap         = None   # OpenCV camera
model       = None   # YOLOv8 model
master_addr = None   # Learned automatically from first packet received


# ═══════════════════════════════════════════════════════════════════════════
# SECTION 3: SETUP
# ═══════════════════════════════════════════════════════════════════════════

def setup_socket():
    global sock
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", PI_PORT))
    sock.settimeout(0.1)
    dbg(f"[UDP] Listening on port {PI_PORT}")


def setup_camera():
    global cap
    cap = cv2.VideoCapture(WEBCAM_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Minimize buffer lag

    if not cap.isOpened():
        print("[CAM] ERROR — Cannot open webcam! Check USB connection.")
        exit(1)
    dbg("[CAM] USB Webcam opened OK")


def setup_model():
    global model
    dbg(f"[YOLO] Loading model: {MODEL_PATH}")
    model = YOLO(MODEL_PATH)
    dbg("[YOLO] Model loaded OK")


# ═══════════════════════════════════════════════════════════════════════════
# SECTION 4: UDP COMMUNICATION
# ═══════════════════════════════════════════════════════════════════════════

def send_to_master(msg: str):
    """Fire and forget — no ACK. Uses learned master_addr if available."""
    global master_addr

    # Use learned IP first, fallback to configured MASTER_IP
    target = master_addr if master_addr else (MASTER_IP, MASTER_PORT)

    try:
        sock.sendto(msg.encode(), target)
        dbg(f"[UDP→Master] {msg}")
    except Exception as e:
        dbg(f"[UDP] Send error: {e}")


def send_with_ack(packet: str) -> bool:
    """Send packet and wait for ACK from Master"""
    global master_addr

    target = master_addr if master_addr else (MASTER_IP, MASTER_PORT)

    for attempt in range(1, MAX_RETRIES + 1):
        dbg(f"[UDP] Attempt {attempt}: {packet}")

        try:
            sock.sendto(packet.encode(), target)
        except Exception as e:
            dbg(f"[UDP] Send error: {e}")
            continue

        # Wait for ACK
        start = time.time()
        while time.time() - start < ACK_TIMEOUT:
            try:
                data, addr = sock.recvfrom(256)
                msg = data.decode().strip()
                # Update master_addr whenever we get a packet
                master_addr = addr
                if msg.startswith("ACK"):
                    dbg(f"[UDP] ACK received: {msg}")
                    return True
            except socket.timeout:
                pass
            except Exception:
                pass
            time.sleep(0.01)

        dbg("[UDP] No ACK — retrying...")
        time.sleep(0.3)

    dbg("[UDP] All retries failed")
    return False


def listen_for_command(timeout_sec: float = 15.0) -> str:
    """Wait for a command from Master — also learns Master IP automatically"""
    global master_addr

    sock.settimeout(timeout_sec)
    try:
        data, addr = sock.recvfrom(256)
        cmd = data.decode().strip()

        # Learn/update ESP32 Master IP automatically from incoming packet
        master_addr = addr
        dbg(f"[CMD] Received: {cmd} from {addr[0]}")

        sock.settimeout(0.1)
        return cmd

    except socket.timeout:
        sock.settimeout(0.1)
        return ""

    except Exception as e:
        dbg(f"[CMD] Error: {e}")
        sock.settimeout(0.1)
        return ""


def check_for_stop_patrol() -> bool:
    """Non-blocking check for STOP_PATROL command"""
    global master_addr

    try:
        data, addr = sock.recvfrom(256)
        cmd = data.decode().strip()

        # Learn/update Master IP
        master_addr = addr
        dbg(f"[CMD] {cmd}")

        if cmd.startswith("STOP_PATROL"):
            return True

    except socket.timeout:
        pass
    except Exception as e:
        dbg(f"[CMD] Error: {e}")

    return False


# ═══════════════════════════════════════════════════════════════════════════
# SECTION 5: CAPTURE FRESH FRAME
# ═══════════════════════════════════════════════════════════════════════════

def capture_frame():
    """Capture latest frame from webcam — flushes buffer for fresh frame"""
    for _ in range(3):
        cap.grab()
    ret, frame = cap.read()
    if not ret:
        dbg("[CAM] Frame capture FAILED")
        return None
    return frame


# ═══════════════════════════════════════════════════════════════════════════
# SECTION 6: IMAGE QUALITY SCORE
# ═══════════════════════════════════════════════════════════════════════════

def get_quality_score(frame) -> int:
    """Returns image quality score 0-100. Below 40 = fog/poor visibility"""
    gray     = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    min_px   = int(gray.min())
    max_px   = int(gray.max())
    contrast = max_px - min_px
    score    = int((contrast / 255) * 100)
    dbg(f"[QC] Contrast={contrast} Score={score}")
    return score


# ═══════════════════════════════════════════════════════════════════════════
# SECTION 7: YOLOV8 INFERENCE
# ═══════════════════════════════════════════════════════════════════════════

def run_inference(frame):
    """
    Run YOLOv8 inference on frame.
    Returns dict with vehicle counts and flags.
    Classes: ambulance, fire truck, car, threewheel, bus, truck, motorbike, van, pedestrian
    """
    result = {
        "CR": 0,   # car
        "TW": 0,   # threewheel (tuk-tuk/auto)
        "BS": 0,   # bus
        "TK": 0,   # truck
        "MB": 0,   # motorbike
        "VN": 0,   # van
        "AM": 0,   # ambulance (flag)
        "FT": 0,   # fire truck (flag)
        "PD": 0,   # pedestrian (flag)
    }

    try:
        results = model(frame, conf=CONFIDENCE, imgsz=640, verbose=False)

        for r in results:
            for box in r.boxes:
                cls_id   = int(box.cls[0])
                cls_name = model.names[cls_id].lower()
                conf_val = float(box.conf[0])

                dbg(f"[YOLO] Detected: {cls_name} ({conf_val:.2f})")

                # Map detected class name to packet key
                for label, key in CLASS_NAMES.items():
                    if label in cls_name:
                        if key in ["AM", "FT", "PD"]:
                            result[key] = 1   # Boolean flag
                        else:
                            result[key] += 1  # Increment count
                        break

    except Exception as e:
        dbg(f"[YOLO] Inference error: {e}")

    dbg(f"[DETECT] CR:{result['CR']} TW:{result['TW']} BS:{result['BS']} "
        f"TK:{result['TK']} MB:{result['MB']} VN:{result['VN']} "
        f"AM:{result['AM']} FT:{result['FT']} PD:{result['PD']}")

    return result


# ═══════════════════════════════════════════════════════════════════════════
# SECTION 8: BUILD CNT PACKET
# ═══════════════════════════════════════════════════════════════════════════

def build_count_packet(lane: str, counts: dict, quality: int) -> str:
    """
    Build CNT packet string to send to Master.
    Classes: car, threewheel, bus, truck, motorbike, van, ambulance, firetruck, pedestrian
    """
    packet = (
        f"CNT:{lane},"
        f"CR:{counts['CR']},"
        f"TW:{counts['TW']},"
        f"BS:{counts['BS']},"
        f"TK:{counts['TK']},"
        f"MB:{counts['MB']},"
        f"VN:{counts['VN']},"
        f"AM:{counts['AM']},"
        f"FT:{counts['FT']},"
        f"PD:{counts['PD']},"
        f"QC:{quality}"
    )
    return packet


# ═══════════════════════════════════════════════════════════════════════════
# SECTION 9: ROI VIOLATION CHECK
# ═══════════════════════════════════════════════════════════════════════════

def check_roi_violation(frame) -> bool:
    """
    Check bottom 15% of frame for vehicle crossing stop line.
    Returns True if violation detected.
    """
    h, w = frame.shape[:2]
    roi_start    = int(h * ROI_RATIO)
    roi          = frame[roi_start:h, 0:w]
    gray         = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    dark_pixels  = int((gray < 100).sum())
    total_pixels = gray.size
    ratio        = dark_pixels / total_pixels
    dbg(f"[ROI] Dark ratio: {ratio:.2f}")
    return ratio > ROI_DARK_RATIO


# ═══════════════════════════════════════════════════════════════════════════
# SECTION 10: HANDLE CAPTURE COMMAND
# ═══════════════════════════════════════════════════════════════════════════

def handle_capture(lane: str):
    """
    Capture frame → run YOLOv8 → send CNT to Master → wait for ACK
    Called when Master sends "CAPTURE:<LANE>"
    """
    dbg(f"\n[CAPTURE] === {lane} ===")

    # Capture fresh frame
    frame = capture_frame()
    if frame is None:
        send_to_master("CAM:TIMEOUT")
        return

    # Image quality check
    quality = get_quality_score(frame)

    # Fog alert
    if quality < 40:
        fog_msg = f"FOG:1,QC:{quality}"
        send_to_master(fog_msg)
        dbg("[FOG] Alert sent to Master")

    # Run YOLOv8 inference
    counts = run_inference(frame)

    # Build and send CNT packet
    packet = build_count_packet(lane, counts, quality)
    ack_ok = send_with_ack(packet)

    if ack_ok:
        dbg(f"[CAPTURE] Count sent and ACKed ✓")
    else:
        dbg(f"[CAPTURE] No ACK — Master will use fallback timing")


# ═══════════════════════════════════════════════════════════════════════════
# SECTION 11: PATROL MODE
# ═══════════════════════════════════════════════════════════════════════════

def handle_patrol(green_lane: str, duration_sec: int):
    """
    Patrol non-green lanes for violations during green phase.
    Checks each RED lane's ROI every 4.5s.
    Stops when STOP_PATROL received or duration expires.
    """
    dbg(f"[PATROL] Starting — green={green_lane} dur={duration_sec}s")

    all_lanes  = ["WEST", "NORTH", "EAST"]
    red_lanes  = [l for l in all_lanes if l != green_lane]
    start_time = time.time()
    idx        = 0

    while time.time() - start_time < duration_sec:

        # Check for STOP_PATROL
        if check_for_stop_patrol():
            dbg("[PATROL] Stopped by Master")
            return

        # Check next RED lane
        check_lane = red_lanes[idx % len(red_lanes)]
        idx += 1

        dbg(f"[PATROL] Checking {check_lane}")

        # Capture and check ROI
        frame = capture_frame()
        if frame is not None:
            if check_roi_violation(frame):
                vio_msg = f"VIO:{check_lane},TS:{int(time.time())}"
                send_to_master(vio_msg)
                dbg(f"[PATROL] ⚠️  VIOLATION on {check_lane}!")

        time.sleep(PATROL_INTERVAL)

    dbg("[PATROL] Duration elapsed")


# ═══════════════════════════════════════════════════════════════════════════
# SECTION 12: PARSE LANE FROM COMMAND
# ═══════════════════════════════════════════════════════════════════════════

def parse_lane(cmd: str) -> str:
    """Extract lane name from command string"""
    for lane in ["WEST", "NORTH", "EAST"]:
        if lane in cmd:
            return lane
    return ""


def parse_duration(cmd: str) -> int:
    """Extract DUR value from PATROL command"""
    if "DUR:" in cmd:
        try:
            return int(cmd.split("DUR:")[1].split(",")[0])
        except Exception:
            pass
    return 30  # Default 30s


# ═══════════════════════════════════════════════════════════════════════════
# SECTION 13: MAIN LOOP
# ═══════════════════════════════════════════════════════════════════════════

def main():
    dbg("\n[SYSTEM] Dynamic Green Light — Raspberry Pi v1.0")
    dbg("[SYSTEM] YOLOv8 + USB Webcam\n")

    # Setup
    setup_socket()
    setup_camera()
    setup_model()

    # Warm up camera
    dbg("[CAM] Warming up camera...")
    for _ in range(10):
        cap.read()
    time.sleep(1)

    # Notify Master that Pi is ready
    time.sleep(1)
    send_to_master("CAM:READY")
    dbg("[SYSTEM] CAM:READY sent — waiting for Master commands\n")

    # ── Main command loop ─────────────────────────────────────────────────
    while True:
        try:
            # Wait for command from Master (15s timeout)
            cmd = listen_for_command(timeout_sec=15.0)

            # No command received
            if cmd == "":
                dbg("[LOOP] No command in 15s — notifying Master")
                send_to_master("CAM:TIMEOUT")
                continue

            # CAPTURE command
            if cmd.startswith("CAPTURE:"):
                lane = parse_lane(cmd)
                if not lane:
                    dbg(f"[LOOP] Unknown lane: {cmd}")
                    continue
                handle_capture(lane)
                continue

            # PATROL command
            if cmd.startswith("PATROL:"):
                green_lane = parse_lane(cmd)
                duration   = parse_duration(cmd)
                if not green_lane:
                    dbg(f"[LOOP] Unknown lane in PATROL: {cmd}")
                    continue
                handle_patrol(green_lane, duration)
                continue

            # STOP_PATROL received outside patrol
            if cmd.startswith("STOP_PATROL"):
                dbg("[LOOP] STOP_PATROL received outside patrol — ignoring")
                continue

            dbg(f"[LOOP] Unknown command: {cmd}")

        except KeyboardInterrupt:
            dbg("\n[SYSTEM] Shutting down...")
            break
        except Exception as e:
            dbg(f"[LOOP] Error: {e}")
            time.sleep(1)

    # Cleanup
    cap.release()
    sock.close()
    dbg("[SYSTEM] Cleanup done. Bye!")


# ═══════════════════════════════════════════════════════════════════════════
# ENTRY POINT
# ═══════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    main()

# ████████████████████████████████████████████████████████████████████████████
# END OF RASPBERRY PI CODE v1.0
# ████████████████████████████████████████████████████████████████████████████

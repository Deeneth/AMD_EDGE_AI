# DYNAMIC GREEN LIGHT TRAFFIC SYSTEM
**AI-Powered Smart Traffic Management**  
Raspberry Pi 5 · YOLOv8 · T-Junction Demo · Version 1.0 · 2025

---

## 📋 Project Overview

The Dynamic Green Light Traffic System is an AI-powered smart traffic management prototype designed for a T-junction (West, North, East). It uses a **Raspberry Pi 5** running **YOLOv8** object detection to count vehicles in each lane. An ESP32 Master processes the counts and dynamically calculates green light durations, replacing static fixed-time signals with intelligent, load-based timing.

### System Architecture
```
USB Webcam → Raspberry Pi 5 (YOLOv8) → UDP/WiFi → ESP32 Master → Traffic Lights + Servo
```

| Component | Role |
|-----------|------|
| **Raspberry Pi 5** | Image capture, YOLOv8 inference, vehicle count, violation detection |
| **ESP32 Master** | Receives counts, calculates green timings, controls lights and servo motor |
| **Servo Motor** | Rotates camera 0° / 90° / 180° to face West / North / East |
| **Traffic Lights** | 3 sets of RGB LEDs or signal modules (one per lane) |
| **Buzzer** | Emergency vehicle alert |

---

## 🔧 Hardware Components

| Component | Spec / Notes |
|-----------|--------------|
| **Raspberry Pi 5** | 8GB RAM, runs YOLOv8 inference with USB webcam |
| **USB Webcam** | 640x480 resolution, mounted on servo motor |
| **ESP32 (Master)** | Handles logic, PWM for lights, UDP communication with Pi |
| **Servo Motor** | SG90 or MG996R — 180° rotation for 3-lane coverage |
| **Traffic Light Module** | 3x RGB LED modules or actual signal lights (one per lane) |
| **Buzzer** | Active buzzer for emergency vehicle alert |
| **Power Supply** | 5V 3A for Raspberry Pi 5, separate 5V for servo |

⚠️ **Important**: Power the servo motor from a separate 5V rail to avoid voltage drops during inference.

---

## 🤖 AI Model & Training

### Model: YOLOv8n (Nano)
- Trained in **VSCode** or **Google Colab**
- Exported to **ONNX** format for Raspberry Pi 5 deployment
- Model file: `models/trained/best.onnx`

### Dataset Classes (9 classes)

| Class | Images | Weight | Notes |
|-------|--------|--------|-------|
| Bike/Motorcycle | ~700 | 1.0 | Standard vehicle |
| Car | ~700 | 2.0 | Standard vehicle |
| Auto Rickshaw | ~700 | 1.5 | Three-wheeler |
| Bus | ~700 | 4.0 | Heavy vehicle |
| Truck | ~700 | 3.0 | Heavy vehicle |
| Ambulance | ~700 | — | **PRIORITY OVERRIDE** |
| Fire Truck | ~700 | — | **PRIORITY OVERRIDE** |
| Van | ~700 | 2.0 | Standard vehicle |
| Pedestrian | ~700 | 0.5 | Green delay trigger |

**Total**: ~5600 images

### ROI Calibration
The stop-line region is defined as a horizontal strip at the bottom 15% of each lane's image frame. This ROI is used for both vehicle counting and violation detection.

---

## 🔄 Complete System Flow

### INIT — All lanes RED
On power-up, all three traffic lights are set to RED. The system waits for the first camera capture before starting the cycle.

### Full Cycle (West → North → East → repeat)

| Step | Action |
|------|--------|
| **1** | Camera servo rotates to face **West**. Pi captures image, runs YOLOv8 inference. Sends vehicle count to ESP32 Master via UDP (with ACK). |
| **2** | ESP32 Master calculates green light duration for West using weighted count formula. Min=5s, Max=60s. |
| **3** | **West = GREEN**, North = RED, East = RED. Buzzer silent. |
| **4** | Camera rotates to **North**, captures still, runs inference, sends North count to Master. Master stores count for next cycle. |
| **5** | Camera enters **violation patrol mode**: rotates W → N → E → W every 4-5 seconds, checking each lane's ROI for red-light crossing. |
| **6** | West green timer expires. **West = RED**. North green duration calculated from stored count. |
| **7** | **North = GREEN**, West = RED, East = RED. Camera rotates to **East**, captures count, stores it. Then begins patrol. |
| **8** | North green expires. **North = RED**. **East = GREEN**. Camera captures West count for next round, then begins patrol. |
| **9** | East green expires. **East = RED**. Cycle restarts from Step 1 (West's pre-captured count already available). |

### Cycle Summary
```
INIT: ALL RED
↓
West GREEN ━━━ [Camera → North count] ━━━ [Patrol W/N/E] ━━━ West RED
↓
North GREEN ━━━ [Camera → East count] ━━━ [Patrol W/N/E] ━━━ North RED
↓
East GREEN ━━━ [Camera → West count] ━━━ [Patrol W/N/E] ━━━ East RED
↓
━━━━ Repeat cycle
```

---

## ⏱️ Dynamic Green Light Timing Algorithm

### Weighted Count Formula
Each vehicle type is assigned a weight reflecting its physical size and road impact:

```
Weighted Score = (bikes × 1.0) + (cars × 2.0) + (autos × 1.5) 
                + (buses × 4.0) + (trucks × 3.0) + (vans × 2.0) 
                + (pedestrians × 0.5)

Green Time (seconds) = BASE_TIME + (Weighted Score × TIME_PER_UNIT)
```

**Parameters**:
- `BASE_TIME` = 10 seconds (minimum guaranteed green)
- `TIME_PER_UNIT` = 2 seconds per weighted unit
- `MIN GREEN` = 5 seconds
- `MAX GREEN` = 60 seconds

**Example**:
```
Lane has: 5 bikes, 8 cars, 2 buses, 1 truck
Score = (5×1) + (8×2) + (2×4) + (1×3) = 5+16+8+3 = 32
Green Time = 10 + (32 × 2) = 74 → capped at 60 seconds
```

### Lane Classification

| Lane Type | Condition | Behaviour |
|-----------|-----------|-----------|
| **Heavy** | buses+trucks > 30% of count | Base time +10s, max 60s |
| **Moderate** | mixed traffic | Standard formula |
| **Fast** | bikes+cars > 70% of count | Base time -5s, faster rotation |
| **Empty** | count = 0 | Skip lane, use 5s minimum |

---

## 🚨 Emergency & Special Conditions

### 1. Ambulance / Fire Truck Priority Override

| Stage | Action |
|-------|--------|
| **DETECT** | Ambulance/Fire Truck detected in Lane X during patrol scan |
| **WARN** | Current green lane → YELLOW for 2 seconds (warning to drivers) |
| **SWITCH** | Lane X → GREEN immediately. All other lanes → RED. **BUZZER ON** |
| **LOCK** | Camera locks onto Lane X. Continuously monitors ROI for ambulance crossing |
| **CLEAR** | Ambulance crosses ROI → BUZZER OFF. 3-second clearance gap maintained |
| **RESUME** | System resumes normal cycle from where it left off (not a full restart) |

⚠️ **Edge Case**: Two ambulances simultaneously → Priority order: West > North > East

### 2. Camera / Connection Failure — Fallback Mode

| Stage | Action |
|-------|--------|
| **DETECT** | No count received within 5 seconds → flag CAMERA FAIL |
| **FALLBACK** | Switch to fixed 20-second green rotation: West → North → East → repeat |
| **ALERT** | RED light blinks once every 5 seconds on all lanes |
| **RETRY** | Master retries UDP connection every 10 seconds automatically |
| **RESTORE** | When connection restored → auto-resume dynamic mode |

### 3. Fog / Heavy Rain Detection

Image quality assessed by computing contrast score of captured frame.

| Stage | Action |
|-------|--------|
| **DETECT** | Contrast score below threshold AND brightness within normal range → Poor visibility flag |
| **SIGNAL** | All lanes blink YELLOW rapidly (warning mode) |
| **FALLBACK** | Switch to fixed 20-second fallback rotation |
| **NIGHT** | Dual condition check: contrast AND brightness to avoid false trigger at night |
| **RESTORE** | Image quality re-checked every cycle. When clarity restored → resume dynamic mode |

### 4. Pedestrian Safety — Green Light Delay

| Stage | Action |
|-------|--------|
| **CHECK** | In last 3s of current green → camera faces next-to-go lane and scans ROI |
| **DETECTED** | Pedestrian in crossing zone → delay GREEN by +2 seconds |
| **RECHECK** | After 2 seconds, re-scan ROI. If pedestrian still present → extend again by +2s |
| **MAX** | Maximum 2 extensions (4 seconds total). After max, proceed to GREEN to prevent deadlock |
| **CLEAR** | No pedestrian detected → switch to GREEN immediately as scheduled |

---

## 📡 Data Format & Communication

### Count Packet Format (Raspberry Pi → ESP32 Master)

The Raspberry Pi sends a compact string over UDP. The Master replies with an ACK. If no ACK within 2 seconds, the Pi retransmits (max 3 retries).

```
// Sent by Raspberry Pi:
CNT:WEST,BK:3,CR:5,AT:2,BS:1,TK:0,VN:2,AM:0,FT:0,PD:1,QC:87

// Field meanings:
CNT  → Packet type (COUNT)
WEST → Lane identifier (WEST / NORTH / EAST)
BK   → Bike count
CR   → Car count
AT   → Auto Rickshaw count
BS   → Bus count
TK   → Truck count
VN   → Van count
AM   → Ambulance detected (0 or 1)
FT   → Fire Truck detected (0 or 1)
PD   → Pedestrian in ROI (0 or 1)
QC   → Image quality score (0-100, below 40 = fog/rain)

// ACK reply from ESP32 Master:
ACK:WEST,GT:45

GT → Calculated green time in seconds

// Violation alert (sent immediately during patrol):
VIO:NORTH,TS:1712345678

VIO → Violation detected
TS  → Timestamp (Unix epoch)
```

### ACK / Retry Logic
After sending a CNT packet, the Raspberry Pi waits up to 2 seconds for an ACK reply. If no ACK is received, it retransmits up to 3 times. After 3 failures, it signals the Master to enter fallback mode.

---

## 📂 Project Structure

```
├── data/                          # Datasets
│   └── Yolo model5/              # Training dataset
│       ├── rebalanced/
│       │   ├── train/
│       │   ├── val/
│       │   └── test/
│       └── data.yaml
├── models/                        # Model files
│   ├── yolov8n.pt                # Pretrained YOLOv8 nano
│   └── trained/                  # Trained models
│       ├── best.pt               # Best checkpoint (PyTorch)
│       ├── best.onnx             # ONNX export for Pi 5
│       └── last.pt               # Last checkpoint
├── src/                          # Source code
│   ├── train.py                  # Training script
│   └── visualize_live.py         # Live webcam detection
├── scripts/                      # Utility scripts
│   ├── check_gpu.py              # GPU availability check
│   ├── fix_labels.py             # Dataset label fixer
│   └── export_onnx.py            # Model export to ONNX
├── configs/                      # Configuration files
├── notebooks/                    # Jupyter notebooks
├── outputs/                      # Training outputs
│   └── runs/                     # YOLOv8 training runs
├── requirements.txt              # Python dependencies
├── .gitignore                    # Git ignore rules
└── README.md                     # This file
```

---

## 🚀 Installation & Setup

### 1. Raspberry Pi 5 Setup

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install Python dependencies
sudo apt install python3-pip python3-opencv -y

# Install YOLOv8
pip3 install ultralytics opencv-python

# Install ONNX runtime for faster inference
pip3 install onnxruntime
```

### 2. Clone Project & Setup

```bash
cd /home/pi
git clone <your-repo-url>
cd "Yolo model8"

# Install requirements
pip3 install -r requirements.txt
```

### 3. Copy Trained Model

Transfer `models/trained/best.onnx` to Raspberry Pi:
```bash
scp models/trained/best.onnx pi@<pi-ip>:/home/pi/"Yolo model8"/models/trained/
```

---

## 🎯 Usage

### Train Model (on PC/Laptop)
```bash
python src/train.py
```

### Export to ONNX (for Raspberry Pi)
```bash
python scripts/export_onnx.py
```

### Live Detection (on Raspberry Pi 5)
```bash
python src/visualize_live.py
```
Press `q` to quit.

### Traffic System (on Raspberry Pi 5)
```bash
python src/traffic_pi.py
```

---

## 📊 System Priority Table

| Priority | Condition | System Response |
|----------|-----------|-----------------|
| **P1 — CRITICAL** | Ambulance / Fire Truck detected | Immediate GREEN override + BUZZER. Resume from where left off after clearance. |
| **P2 — HIGH** | Pedestrian in crossing zone | Delay GREEN by +2s (max +4s total). Prevent deadlock with extension limit. |
| **P3 — MEDIUM** | Fog / Heavy rain (low contrast) | Rapid YELLOW blink all lanes + 20s fixed fallback mode. |
| **P4 — MEDIUM** | Camera / connection failure | 20s fixed rotation + RED blink alert + auto-restore. |
| **P5 — NORMAL** | Normal traffic conditions | Dynamic weighted count timing algorithm. Min 5s, Max 60s. |

---

## 🔄 Scalability — Real-World Implementation

### Prototype vs Production

| Prototype | Production Equivalent | Reason |
|-----------|----------------------|--------|
| Raspberry Pi 5 + Servo | Wide-angle IP camera per lane | Eliminates mechanical parts, simultaneous all-lane view |
| YOLOv8 ONNX | YOLOv8 on Jetson Nano / RPi 5 | Real-time video stream inference, higher accuracy |
| Single still image | Video stream + line-crossing logic | Accurate queue length, handles occluded vehicles |
| UDP over WiFi | MQTT over LAN / 4G | Reliable, scalable to city-wide network |
| ESP32 Master | Raspberry Pi / Edge server | Handles multiple junctions, logging, remote monitoring |
| Local logging | Cloud database | Historical analysis, traffic pattern learning |

### Production Architecture
```
IP Cameras (per lane)
    ↓
Edge Server (Jetson Nano / RPi 5)
    ↓ YOLOv8 inference (real-time video)
    ↓
MQTT broker
    ↓
Signal Controller → Traffic Lights
    ↓
Cloud Dashboard (logging, remote monitoring)
```

---

## 🎯 Key Innovations

✅ Dynamic timing based on **weighted vehicle count** (not raw count)  
✅ Single camera + servo covers all 3 lanes (cost-effective prototype)  
✅ **Next lane count captured immediately** after setting GREEN (pre-loading)  
✅ Violation patrol rotation every 4-5s during green phase  
✅ **Ambulance/Fire truck priority override** with resume-from-position logic  
✅ Pedestrian safety delay with anti-deadlock maximum extension limit  
✅ Fog/rain detection via image contrast scoring  
✅ UDP communication with ACK/retry logic  
✅ Camera failure fallback to fixed 20s rotation with visual alert  
✅ Violation snapshot logging with timestamp  
✅ Clear path to scale: servo → fixed cameras, Pi → Jetson/RPi cluster  

---

## 📝 Configuration

Edit `src/traffic_pi.py` to configure:

```python
MASTER_IP       = "192.168.53.51"  # ESP32 Master IP
MASTER_PORT     = 4210             # Master listens here
PI_PORT         = 4211             # Pi listens here

WEBCAM_INDEX    = 0                # USB webcam index
FRAME_WIDTH     = 640
FRAME_HEIGHT    = 480

MODEL_PATH      = "best.onnx"      # Path to trained model
CONFIDENCE      = 0.50             # Detection confidence threshold

ROI_RATIO       = 0.85             # ROI starts at 85% height
PATROL_INTERVAL = 4.5              # Seconds between patrol checks
```

---

## 🐛 Troubleshooting

### Camera not detected
```bash
# List USB devices
lsusb

# Test camera
python3 -c "import cv2; print(cv2.VideoCapture(0).isOpened())"
```

### Model not loading
```bash
# Check ONNX file exists
ls -lh models/trained/best.onnx

# Test ONNX runtime
python3 -c "import onnxruntime; print(onnxruntime.get_device())"
```

### UDP connection issues
```bash
# Check network connectivity
ping <ESP32_IP>

# Check firewall
sudo ufw allow 4211/udp
```

---

## 📄 License

MIT License - Feel free to use for educational and commercial purposes.

---

## 👥 Contributors

**Dynamic Green Light Traffic System**  
Version 1.0 · 2025

---

**🚦 Making Traffic Smarter, One Junction at a Time**

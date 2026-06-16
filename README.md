# camera_node_1

DepthAI pellet detection node with Sparkplug B (ISA-95) MQTT bridge.

Runs standalone — **no ROS 2 dependency**. Drives the OAK-D / OAK-D Lite camera,
performs HSV-based pellet and foreign-object detection, and publishes results to a
SCADA system via MQTT Sparkplug B. Two run modes:

| Mode | File | Use |
|------|------|-----|
| **Panel** (default) | `camera_panel.py` | 800×480 touchscreen — live feed + detection results + controls |
| **Headless** | `camera_spb_node.py` | No display — bridge only, SCADA-triggered cycles |
| **Debug GUI** | `camera_gui.py` | Development monitor — camera tuning, HSV colour tuner |

---

## Package layout

```
camera_node_1/
├── camera_panel.py          Production touchscreen panel (PyQt5)
├── camera_spb_node.py       Headless Sparkplug B bridge
├── camera_gui.py            Development monitor / debug GUI
├── camera.py                CameraDetector + VideoWorker (no Qt dependency)
├── color_tuner.py           Interactive HSV threshold tuner dialog
├── config_loader.py         Loads camera_params.yaml + camera_secrets.yaml
├── config/
│   ├── camera_params.yaml   Main configuration (SPB identity, camera, thresholds)
│   ├── camera_secrets.yaml  Gitignored broker credentials (copy from .example)
│   ├── camera_secrets.example.yaml
│   └── color_presets.yaml   Saved HSV tuner presets
├── systemd/
│   ├── camera_node.service           Panel mode (touchscreen)
│   └── camera_node_headless.service  Headless mode (no display)
├── scripts/
│   └── install_service.sh   One-shot systemd service installer
└── README.md                ← this file
```

---

## Dependencies

Install once on the Jetson (L4T Ubuntu 20.04 / 22.04):

```bash
# MQTT
pip3 install paho-mqtt

# Sparkplug B (tahu library — must match the version used in agx_arm_gui)
pip3 install tahu

# DepthAI SDK
pip3 install depthai

# Computer vision
pip3 install opencv-python-headless numpy

# PyQt5 (for panel and debug GUI modes)
sudo apt install python3-pyqt5
# OR: pip3 install PyQt5
```

Verify DepthAI can see the camera:

```bash
python3 -c "import depthai as dai; print(dai.Device.getAllAvailableDevices())"
```

---

## Configuration

### 1 — Fill in broker credentials

```bash
cp config/camera_secrets.example.yaml config/camera_secrets.yaml
nano config/camera_secrets.yaml
```

`camera_secrets.yaml` is **gitignored** — never commit credentials. It is
deep-merged on top of `camera_params.yaml` at load time, so you only need to
set the keys you want to override.

### 2 — Adjust detection thresholds (optional)

Edit `config/camera_params.yaml` or use the colour tuner in `camera_gui.py`.
The tuner writes changes to `camera_secrets.yaml` (the gitignored overlay),
leaving `camera_params.yaml` clean.

Key parameters:

| Key | Default | Meaning |
|-----|---------|---------|
| `camera.pellet_color.lower_hsv` | `[28, 140, 160]` | Pellet HSV lower bound (yellow) |
| `camera.pellet_color.upper_hsv` | `[38, 255, 255]` | Pellet HSV upper bound |
| `camera.foreign_color.lower_hsv` | `[0, 0, 0]` | Foreign object lower (all-zero = disabled) |
| `camera.foreign_color.upper_hsv` | `[0, 0, 0]` | Foreign object upper |
| `camera.pellet_pixel_threshold` | `100` | Min pixels to detect pellets |
| `camera.foreign_pixel_threshold` | `50` | Min foreign pixels to reject |
| `camera.detection_timeout_s` | `30.0` | Seconds before alarm 8002 fires |
| `spb_bridge.broker_type` | `local` | `local` \| `hivemq` |
| `spb_bridge.heartbeat_interval_s` | `0.5` | SCADA heartbeat toggle rate |

---

## Running

### Panel mode (touchscreen — recommended for production)

```bash
cd ~/agx_arm_ws/src/camera_node_1

# Use broker from camera_params.yaml
python3 camera_panel.py

# Override broker
python3 camera_panel.py --broker hivemq

# Windowed (development / non-touch screen)
python3 camera_panel.py --no-fullscreen

# Custom resolution
python3 camera_panel.py --width 1024 --height 600
```

### Headless mode (no display — SCADA-only)

```bash
python3 camera_spb_node.py

# Override broker via env var
CAMERA_BROKER_TYPE=hivemq python3 camera_spb_node.py
```

### Debug GUI (development)

```bash
python3 camera_gui.py
```

---

## Panel UI — operator guide

The panel shows:

```
┌──────────────────────────────────────────────────────────────────┐
│ CAMERA DETECTION      ● ONLINE  0 alarms  ● IDLE    12:00:00   │  ← header
├──────────────────────────────────┬───────────────────────────────┤
│                                  │   LAST RESULT                 │
│                                  │                               │
│   LIVE VIDEO FEED                │       PASS                    │  ← result card
│   (annotated: pellets green,     │                               │
│    foreign red)                  │  Pellet: 4,231 px · 12 circ  │
│                                  │  Foreign: 0 px                │
│                                  ├───────────────────────────────┤
│                                  │  CYCLE START  (green)         │  ← start button
│                                  ├───────────────────────────────┤
│                                  │  RESET    │  CLEAR ALARMS     │  ← lower buttons
└──────────────────────────────────┴───────────────────────────────┘
```

### Button states

| Button | Enabled when | Action |
|--------|-------------|--------|
| **CYCLE START** | State = Idle, 0 active alarms | Triggers one detection cycle |
| **RESET** | State = Complete | Returns to Idle |
| **CLEAR ALARMS** | State = Aborted | Clears all alarms → Idle |

### PackML state machine

```
                 ┌──────────────────────────────────────────────────────────┐
                 ▼                                                          │
             ┌──────┐   CYCLE START    ┌─────────┐   detection done    ┌──────────┐
             │ Idle │ ───────────────► │ Execute │ ──────────────────► │ Complete │
             └──────┘                 └─────────┘                     └──────────┘
                 ▲                         │                                │
                 │ CLEAR ALARMS            │ timeout / cam error            │ RESET
                 │                         ▼                                │
             ┌─────────┐            ┌─────────┐                            │
             │ Aborted │ ◄──────────┤         │ ◄──────────────────────────┘
             └─────────┘            └─────────┘
```

Stop and Reset commands are accepted from SCADA at any time.

### Alarm catalogue

| Code | Name | Priority | Cause |
|------|------|----------|-------|
| 8001 | CameraOffline | Critical | DepthAI device unavailable or pipeline error |
| 8002 | DetectionTimeout | High | Detection took longer than `detection_timeout_s` |
| 8003 | PrimaryHostOffline | Critical | SCADA primary host went offline |

---

## Sparkplug B identity

| Field | Value |
|-------|-------|
| Group ID | `DMATDTS_DLSU_LS_MiniFactory` |
| Edge Node ID | `camera_node` |
| Device ID | `depthai_camera` |
| DBIRTH topic | `spBv1.0/DMATDTS_DLSU_LS_MiniFactory/DBIRTH/camera_node/depthai_camera` |
| DDATA topic | `spBv1.0/DMATDTS_DLSU_LS_MiniFactory/DDATA/camera_node/depthai_camera` |
| DCMD topic | `spBv1.0/DMATDTS_DLSU_LS_MiniFactory/DCMD/camera_node/depthai_camera` |

### Published metrics (DDATA)

| Metric | Type | Description |
|--------|------|-------------|
| `Status/State/Current/Idle` | Boolean | One-hot state flags |
| `Status/State/Current/Execute` | Boolean | |
| `Status/State/Current/Complete` | Boolean | |
| `Status/State/Current/Aborted` | Boolean | |
| `Status/Heartbeat` | Boolean | Toggles every `heartbeat_interval_s` |
| `Result/Last/Pass` | Boolean | True = PASS, False = REJECT |
| `Result/Last/PelletCount` | Int32 | Discrete pellets (HoughCircles) |
| `Result/Last/PelletPixelCount` | Int32 | Total pellet pixels |
| `Result/Last/ForeignPixelCount` | Int32 | Total foreign-object pixels |
| `Result/Last/TimestampMs` | Int64 | Unix epoch ms of last detection |
| `Alarm/Active/{code}/State` | Int32 | 1 = Normal, 2 = Unacknowledged |
| `Alarm/Active/{code}/Priority` | Int32 | 1 = Critical, 2 = High |
| `Alarm/Active/{code}/Message` | String | Alarm name |
| `Alarm/Summary/ActiveCount` | Int32 | Total active alarms |

### DCMD write tags (SCADA → node)

Write `True` to any of the following to trigger the command:

| Metric | Effect |
|--------|--------|
| `Cmd/CntrlCmd/Start` | Begin detection (Idle only) |
| `Cmd/CntrlCmd/Reset` | Complete → Idle |
| `Cmd/CntrlCmd/Stop` | Abort current cycle → Idle |
| `Cmd/CntrlCmd/Clear` | Clear alarms, Aborted → Idle |

---

## Auto-start on boot (systemd)

### Install — panel mode (touchscreen)

```bash
bash ~/agx_arm_ws/src/camera_node_1/scripts/install_service.sh
sudo systemctl start camera_node.service
```

### Install — headless mode

```bash
bash ~/agx_arm_ws/src/camera_node_1/scripts/install_service.sh headless
sudo systemctl start camera_node_headless.service
```

### Service management

```bash
# Status
sudo systemctl status camera_node.service

# Stop / start / restart
sudo systemctl stop    camera_node.service
sudo systemctl start   camera_node.service
sudo systemctl restart camera_node.service

# Logs (live)
journalctl -u camera_node.service -f

# Disable auto-start
sudo systemctl disable camera_node.service
```

### Re-install after editing the service file

```bash
bash ~/agx_arm_ws/src/camera_node_1/scripts/install_service.sh
sudo systemctl restart camera_node.service
```

---

## Display setup for systemd

The `camera_node.service` file assumes Xorg is running and exports `DISPLAY=:0`.
Adjust for your compositor:

| Environment | Required env vars | Notes |
|-------------|------------------|-------|
| Xorg | `DISPLAY=:0` `XAUTHORITY=/home/dmat2/.Xauthority` | Default |
| Wayland | `WAYLAND_DISPLAY=wayland-0` `QT_QPA_PLATFORM=wayland` | Jetson Orin ships Weston |
| Framebuffer only | `QT_QPA_PLATFORM=linuxfb` | No compositor needed |

Edit `systemd/camera_node.service`, change the `Environment=` lines, then
re-run `install_service.sh`.

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| `ModuleNotFoundError: depthai` | Missing SDK | `pip3 install depthai` |
| `No DepthAI devices found` | Camera not connected | Check USB cable / power; try `lsusb` |
| `RuntimeError: DepthAI pipeline error` | Another process holds the device | Kill other `camera_*.py` processes; only one can own the device |
| `[SPB] Broker rejected connection rc=5` | Wrong credentials | Check `camera_secrets.yaml` username/password |
| Black screen on touchscreen | `$DISPLAY` not set | Add `Environment=DISPLAY=:0` to service file |
| `xcb: could not connect to display` | Qt can't find Xorg | Switch to `QT_QPA_PLATFORM=linuxfb` for framebuffer |
| `ImportError: libGL.so.1` | Missing OpenGL libs | `sudo apt install libgl1` |
| Panel starts but no video | Camera initialising | Wait 5–10 s; DepthAI takes time to enumerate |
| SCADA Start command ignored | Active alarms | Run CLEAR ALARMS first |
| `camera_secrets.yaml not found` | Missing credentials file | `cp config/camera_secrets.example.yaml config/camera_secrets.yaml` |

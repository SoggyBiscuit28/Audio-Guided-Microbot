# Audio-Guided Microbot - Code Documentation

This folder contains all the source code for the Audio-Guided Microbot project. The system enables voice command control of a microbot using an ESP32-S3 with on-device machine learning for audio classification.

## Table of Contents

- [Overview](#overview)
- [Folder Structure](#folder-structure)
- [Components](#components)
  - [Firmware](#firmware)
  - [Communication Classification](#communication-classification)
  - [GUI](#gui)
  - [Model Training](#model-training)
- [Hardware Requirements](#hardware-requirements)
- [Getting Started](#getting-started)

---

## Overview

The Audio-Guided Microbot is an embedded system that:
- Captures audio input via I2S MEMS microphone
- Processes audio using on-device neural network inference (TensorFlow Lite Micro)
- Classifies voice commands (Forward, Backward, Left, Right)
- Controls motors based on recognized commands
- Communicates with a companion GUI via BLE for manual override and monitoring
- Includes obstacle detection using ultrasonic sensors

---

## Folder Structure

```
Code/
├── README.md                           # This file
├── Communication_Classification/       # ESP32 code for BLE and LoRa testing/classification
│   ├── BLE/
│   │   ├── Client.ino                  # BLE client for throughput testing
│   │   └── Server.ino                  # BLE server for throughput testing
│   └── LoRa/
│       ├── receiver.ino                # LoRa receiver with RSSI/SNR logging
│       └── Transmitter.ino             # LoRa transmitter for range testing
├── firmware/                           # Main ESP32-S3 firmware (PlatformIO project)
│   ├── platformio.ini                  # PlatformIO configuration
│   ├── src/
│   │   ├── main.cpp                    # Main application entry point
│   │   ├── config.h                    # Hardware pin configurations
│   │   ├── CommandDetector.cpp/.h      # Audio gating and command detection
│   │   ├── CommandProcessor.cpp/.h     # Command execution logic
│   │   ├── ble_commands.h              # BLE communication handlers
│   │   └── neopixel_motorpin.h         # Motor and NeoPixel pin definitions
│   └── lib/
│       ├── audio_input/                # I2S microphone sampling library
│       │   ├── I2SMicSampler.cpp/.h    # I2S MEMS microphone driver
│       │   ├── I2SSampler.cpp/.h       # Base sampler class
│       │   └── RingBuffer.h            # Circular buffer for audio samples
│       ├── audio_processor/            # Audio feature extraction
│       │   └── src/
│       │       ├── AudioProcessor.cpp/.h   # MFCC feature extraction
│       │       ├── HammingWindow.cpp/.h    # Hamming window function
│       │       └── kissfft/                # FFT library
│       ├── neural_network/             # TensorFlow Lite Micro inference
│       │   └── src/
│       │       ├── model.cc/.h         # Trained model weights
│       │       └── NeuralNetwork.cpp/.h    # Inference wrapper
│       └── tfmicro/                    # TensorFlow Lite Micro library
│           ├── tensorflow/             # TF Lite Micro core
│           └── third_party/            # Dependencies (flatbuffers, gemmlowp, ruy)
├── GUI/                                # Graphical User Interface
│   ├── ESP32/
│   │   └── Client.ino                  # ESP32 BLE client (connects laptop to microbot)
│   └── python_GUI/
│       ├── latest_ble_gui.py           # PySide6 GUI for robot control and monitoring
│       └── requirements.txt            # Python dependencies for GUI
└── model/                              # Machine Learning model training
    ├── Generate Training Data Command Recognition.ipynb    # Data generation notebook
    ├── Train Model-All Words.ipynb                         # Model training notebook
    ├── Convert Trained Model To TFLite.ipynb               # TFLite conversion notebook
    └── requirements.txt                                    # Python dependencies
```

---

## Components

### Firmware

**Location:** `firmware/`

The main ESP32-S3 firmware that runs on the microbot. Built with PlatformIO using the Arduino framework.

**Key Features:**
- **Audio Input:** Captures audio via I2S MEMS microphone at 16kHz
- **Audio Gating:** Energy-based voice activity detection to filter noise
- **Neural Network Inference:** TensorFlow Lite Micro for on-device command recognition
- **Motor Control:** PWM-based motor control with joystick override support
- **BLE Communication:** Receives commands and sends status updates
- **Obstacle Detection:** Ultrasonic sensor integration for collision avoidance
- **Voltage Monitoring:** ADC-based battery voltage monitoring

**Supported Commands:**
| Code | Command | Action |
|------|---------|--------|
| 0 | Toggle Override | Enable/disable voice command mode |
| 1 | Forward | Move forward for 5 seconds |
| 2 | Backward | Move backward for 5 seconds |
| 3 | Left | Turn left for 5 seconds |
| 4 | Right | Turn right for 5 seconds |
| 6 | Controller Mode | Toggle joystick override mode |

**Pin Configuration (ESP32-S3):**
| Pin | Function |
|-----|----------|
| GPIO18 | I2S Mic Serial Data |
| GPIO19 | I2S Mic Left/Right Clock |
| GPIO20 | I2S Mic Serial Clock |
| GPIO7 | ADC Voltage Sensor |
| GPIO4 | Ultrasonic Trigger |
| GPIO5 | Ultrasonic Echo |

---

### Communication Classification

**Location:** `Communication_Classification/`

Contains test code for evaluating BLE and LoRa communication protocols for the microbot.

#### BLE Testing
- **Server.ino:** Creates a BLE server that measures throughput by receiving data packets
- **Client.ino:** BLE client that sends test payloads and measures RSSI

#### LoRa Testing
- **Transmitter.ino:** Transmits packets at configurable intervals with SF12/BW125kHz
- **receiver.ino:** Receives packets and logs RSSI, SNR, and packet statistics

**LoRa Configuration:**
| Parameter | Value |
|-----------|-------|
| Frequency | 433 MHz |
| Spreading Factor | 12 |
| Bandwidth | 125 kHz |
| Coding Rate | 4/5 |
| TX Power | 20 dBm |

---

### GUI

**Location:** `GUI/`

A Python-based graphical user interface for controlling and monitoring the microbot.

#### ESP32 Client (`ESP32/Client.ino`)
- Runs on a separate ESP32 connected to the laptop via USB
- Acts as a BLE bridge between the Python GUI and the ESP32-S3 microbot
- Forwards serial commands as BLE writes
- Receives BLE notifications and prints them to serial

#### Python GUI (`python_GUI/latest_ble_gui.py`)
A modern PySide6 application with:

**Features:**
- Serial port selection with auto-baud detection
- Real-time communication console
- Robot movement controls (Forward, Backward, Left, Right)
- Keyboard arrow key shortcuts
- Xbox controller/joystick support for analog control
- Battery level monitoring with voltage display
- Current and voltage line graphs
- Speed indicator
- Obstacle detection status
- Motion state display
- Override mode toggles

**Requirements:**
```
PySide6
pyserial
pygame (optional, for joystick support)
```

---

### Model Training

**Location:** `model/`

Jupyter notebooks for training the voice command recognition model.

#### Notebooks:
1. **Generate Training Data Command Recognition.ipynb**
   - Generates synthetic training data for voice commands
   - Creates spectrograms and audio features

2. **Train Model-All Words.ipynb**
   - Trains a CNN model for command classification
   - Supports 4 command classes: Forward, Backward, Left, Right

3. **Convert Trained Model To TFLite.ipynb**
   - Converts the trained Keras model to TensorFlow Lite format
   - Quantizes the model for embedded deployment
   - Generates C header file for embedding in firmware

**Model Architecture:**
- Input: MFCC features from audio spectrograms
- Output: 4-class softmax (Forward, Backward, Left, Right)
- Optimized for ESP32-S3 with ~300KB tensor arena

---

## Hardware Requirements

- **ESP32-S3 DevKitC-1** (main microbot controller)
- **ESP32** (BLE bridge for laptop communication)
- **I2S MEMS Microphone** (e.g., INMP441)
- **DRV8833 Motor Driver** or similar H-bridge
- **DC Motors** (2x for differential drive)
- **HC-SR04 Ultrasonic Sensor**
- **LiPo Battery** with voltage divider for monitoring
- **NeoPixel LED** (optional, for status indication)

---

## Getting Started

### Firmware Setup

1. Install [PlatformIO](https://platformio.org/) in VS Code
2. Open the `firmware/` folder in PlatformIO
3. Connect ESP32-S3 via USB
4. Update `upload_port` and `monitor_port` in `platformio.ini` for your OS:
   ```ini
   ; Windows example
   upload_port = COM3
   monitor_port = COM3
   ```
5. Build and upload:
   ```bash
   pio run --target upload
   ```

### GUI Setup

1. Upload `GUI/ESP32/Client.ino` to a separate ESP32 using Arduino IDE
2. Install Python dependencies:
   ```bash
   pip install -r GUI/python_GUI/requirements.txt
   ```
3. Connect the ESP32 client to your laptop via USB
4. Run the GUI:
   ```bash
   python GUI/python_GUI/latest_ble_gui.py
   ```

### Model Training

1. Install Jupyter and dependencies:
   ```bash
   pip install -r model/requirements.txt
   ```
2. Run notebooks in order:
   - Generate Training Data → Train Model → Convert to TFLite
3. Copy the generated model header to `firmware/lib/neural_network/src/model.cc`

---

## Communication Protocol

The GUI communicates with the microbot via BLE using simple text commands:

| Command | Description |
|---------|-------------|
| `0` | Toggle voice command mode |
| `1` | Move forward |
| `2` | Move backward |
| `3` | Turn left |
| `4` | Turn right |
| `6` | Toggle joystick controller mode |
| `J x.xxx y.yyy` | Joystick vector (x, y in range -1.0 to 1.0) |

**BLE Service UUID:** `4fafc201-1fb5-459e-8fcc-c5c9c331914b`  
**BLE Characteristic UUID:** `beb5483e-36e1-4688-b7f5-ea07361b26a8`

---
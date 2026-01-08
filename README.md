[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/Ei5Ot7FV)

# Audio-Guided Microbot

### Team Biscuit

A voice-controlled microbot using ESP32-S3 with on-device machine learning for real-time audio command recognition.

---

## Overview

This project implements a microbot that responds to voice commands (Forward, Backward, Left, Right) using TensorFlow Lite Micro for on-device inference. The system includes BLE communication for manual override and a Python GUI for monitoring.

## Repository Structure

| Folder | Description |
|--------|-------------|
| [`Code/`](./Code/) | ESP32-S3 firmware, GUI application, and ML model training notebooks |
| [`Demos/`](./Demos/) | Demo video and images |
| [`Presentation/`](./Presentation/) | Final evaluation presentation (PDF) |
| [`Report/`](./Report/) | Project report |

## Quick Links

- 📹 [Demo Video](https://drive.google.com/file/d/1aCh8zQsNOZ-Cvipodxt4e_dWg6QOUhP8/view?usp=drivesdk)
- 📁 [Code Documentation](./Code/README.md)

## Tech Stack

- **Hardware:** ESP32-S3, I2S MEMS Microphone, DRV8833 Motor Driver
- **Firmware:** PlatformIO, Arduino Framework, TensorFlow Lite Micro
- **GUI:** Python, PySide6, BLE Communication
- **ML:** TensorFlow/Keras, MFCC Feature Extraction
 
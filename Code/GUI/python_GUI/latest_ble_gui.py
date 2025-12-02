#!/usr/bin/env python3
"""
ESP32 Serial GUI (BLE Forwarder Companion) - PySide6 Version with Modern UI

Features:
- Modern PySide6 interface with dark theme
- Thread-safe logging via signals/slots
- DTR/RTS toggles (default DTR ON), optional "Reset on connect" pulse
- Idle flush for lines without newline
- Battery level and speed monitoring sections
- Enhanced visual feedback and animations

Controls:
- Override checkbox -> sends "0\n" once
- Forward/Backward/Left/Right -> "1\n"/"2\n"/"3\n"/"4\n"
- Arrow keys Up/Down/Left/Right send 1..4
- Custom input -> sends that line + "\n"
"""

import sys
import time
import threading
import queue
from dataclasses import dataclass
from typing import Optional
import re
import random
import math

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QPushButton, QLabel, QLineEdit, QTextEdit, QComboBox, QCheckBox,
    QFrame, QSplitter, QProgressBar, QGroupBox, QSpacerItem, QSizePolicy
)
from PySide6.QtCore import (
    QThread, Signal, QTimer, Qt, QPropertyAnimation, QEasingCurve, QRect, QPointF
)
from PySide6.QtGui import QFont, QKeySequence, QShortcut, QPalette, QColor, QPainter, QPen

try:
    import pygame
    PYGAME_AVAILABLE = True
except Exception:
    pygame = None
    PYGAME_AVAILABLE = False

# ---- pyserial ----
try:
    import serial
    from serial import SerialException
    from serial.tools import list_ports
except Exception as e:
    print("This app requires pyserial. Install with:  pip install pyserial")
    raise

DEFAULT_BAUD = 115200
LINE_ENDING = "\n"
IDLE_FLUSH_MS = 250
VOLTAGE_SMOOTH_ALPHA = 0.08
BATTERY_MIN_V = 4.9
BATTERY_MAX_V = 5.4
ADC_RESOLUTION = 4095
WHEEL_CIRCUMFERENCE_CM = 4.5
FULL_SPEED_RPM = 12
SINGLE_MOTOR_RPM = 6
SPEED_VARIATION_CM = 2.0
AUTO_BAUD_CANDIDATES = [115200, 230400, 460800, 921600, 57600, 38400, 19200, 9600]


@dataclass
class PortInfo:
    device: str
    desc: str


def list_serial_ports():
    ports = []
    for p in list_ports.comports():
        ports.append(PortInfo(p.device, p.description or ""))
    return ports


class SerialWorker(QThread):
    """Background serial reader/writer using QThread."""
    log_message = Signal(str)
    connected = Signal()
    disconnected = Signal()
    current_update = Signal(float)  # Current in Amps
    voltage_update = Signal(float)  # Voltage in Volts
    speed_update = Signal(float)  # Speed in units/sec
    obstacle_detected = Signal(bool)  # True if obstacle detected
    motion_update = Signal(str)  # Motion state (IDLE, FORWARD, BACKWARD, LEFT, RIGHT)

    def __init__(self):
        super().__init__()
        self.ser: Optional[serial.Serial] = None
        self._stop = threading.Event()
        self.use_dtr = True
        self.use_rts = False
        self.reset_on_connect = False
        self.prefetched_buffer = bytearray()
        self.connected_baud: Optional[int] = None

    def set_control_lines(self, dtr: bool, rts: bool, reset_on_connect: bool):
        self.use_dtr = dtr
        self.use_rts = rts
        self.reset_on_connect = reset_on_connect

    def connect_serial(self, port: str, baud: int = DEFAULT_BAUD, timeout=0.1) -> bool:
        self.close_serial()
        self.prefetched_buffer = bytearray()
        self.connected_baud = None

        baud_sequence = [baud] + [b for b in AUTO_BAUD_CANDIDATES if b != baud]
        for test_baud in baud_sequence:
            candidate = self._open_serial(port, test_baud, timeout)
            if not candidate:
                continue
            sample = self._peek_serial(candidate)
            if sample and not self._looks_like_text(sample):
                self.log_message.emit(
                    f"[WARN] Data from {port} @ {test_baud} baud looked binary; trying another baud..."
                )
                candidate.close()
                continue

            self.ser = candidate
            self.connected_baud = test_baud
            self.prefetched_buffer = bytearray(sample)
            break

        if not self.ser:
            fallback = self._open_serial(port, baud, timeout)
            if fallback:
                self.log_message.emit(
                    "[WARN] Auto baud detection could not find readable output; falling back to requested baud."
                )
                self.ser = fallback
                self.connected_baud = baud
                self.prefetched_buffer = bytearray()
            else:
                self.log_message.emit(
                    "[ERR] Unable to find a baud rate that yields readable serial output. "
                    "Verify the ESP32 firmware baud or port selection."
                )
                return False

        self._stop.clear()
        self.start()
        self.connected.emit()
        actual_baud = self.connected_baud or baud
        extra = ""
        if actual_baud != baud:
            extra = f" (auto-selected {actual_baud} baud; requested {baud})"
        self.log_message.emit(
            f"[SER] Connected to {port} @ {actual_baud} "
            f"(DTR={'ON' if self.use_dtr else 'OFF'}, RTS={'ON' if self.use_rts else 'OFF'}){extra}"
        )
        return True

    def _open_serial(self, port: str, baud: int, timeout: float) -> Optional[serial.Serial]:
        try:
            ser = serial.Serial(
                port=port,
                baudrate=baud,
                timeout=timeout,
                write_timeout=1.0,
                exclusive=True
            )
        except Exception as e:
            self.log_message.emit(f"[ERR] Failed to open {port} @ {baud}: {e}")
            return None

        try:
            ser.setRTS(self.use_rts)
            ser.setDTR(self.use_dtr)
            if self.reset_on_connect:
                ser.setDTR(False)
                time.sleep(0.05)
                ser.setDTR(True)
        except Exception as e:
            self.log_message.emit(f"[WARN] Could not set DTR/RTS: {e}")

        try:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
        except Exception:
            pass
        return ser

    def _peek_serial(self, ser: serial.Serial, duration: float = 0.25) -> bytes:
        """Grab a short sample from the serial stream without blocking long."""
        sample = bytearray()
        end_time = time.monotonic() + duration
        while time.monotonic() < end_time and len(sample) < 128:
            waiting = getattr(ser, "in_waiting", 0)
            if waiting:
                sample.extend(ser.read(waiting))
                continue
            chunk = ser.read(1)
            if chunk:
                sample.extend(chunk)
            else:
                time.sleep(0.01)
        return bytes(sample)

    @staticmethod
    def _looks_like_text(sample: bytes) -> bool:
        if not sample:
            return True
        printable = 0
        for b in sample:
            if 32 <= b <= 126 or b in (9, 10, 13):
                printable += 1
        return (printable / len(sample)) >= 0.65

    def run(self):
        """Main serial reading loop."""
        buf = bytearray(self.prefetched_buffer or b"")
        self.prefetched_buffer = bytearray()
        last_rx_ts = time.monotonic()
        
        try:
            while not self._stop.is_set() and self.ser and self.ser.is_open:
                try:
                    chunk = self.ser.read(256)
                    now = time.monotonic()
                    if chunk:
                        buf.extend(chunk)
                        last_rx_ts = now

                        # Process complete lines
                        while True:
                            idx_n = buf.find(b"\n")
                            idx_r = buf.find(b"\r")
                            idxs = [i for i in (idx_n, idx_r) if i != -1]
                            if not idxs:
                                break
                            i = min(idxs)
                            line = buf[:i].decode(errors="replace")
                            self.log_message.emit(f"[ESP] {line}")
                            
                            # Parse special data
                            self._parse_data(line)

                            # Drop newline char; if CRLF, drop both
                            drop = 1
                            if i + 1 < len(buf) and (buf[i] == 13 and buf[i + 1] == 10):
                                drop = 2
                            buf = buf[i + drop:]

                    else:
                        # Idle flush
                        if buf and (now - last_rx_ts) * 1000.0 > IDLE_FLUSH_MS:
                            line = buf.decode(errors="replace")
                            self.log_message.emit(f"[ESP] {line}")
                            self._parse_data(line)
                            buf.clear()
                        time.sleep(0.01)

                except SerialException as e:
                    self.log_message.emit(f"[ERR] Serial error: {e}")
                    break
                except Exception as e:
                    self.log_message.emit(f"[WARN] Reader loop exception: {e}")
                    time.sleep(0.05)

        finally:
            self.disconnected.emit()

    def _parse_data(self, line: str):
        """Parse incoming data for current, voltage, speed, obstacle, and motion information."""
        # Look for current: "Current: 1.5A" or "CURR:1.5"
        current_match = re.search(r'(?:current|curr)[:=]\s*([\d.]+)a?', line, re.IGNORECASE)
        if current_match:
            try:
                current = float(current_match.group(1))
                self.current_update.emit(current)
            except ValueError:
                pass

        # Prefer ADC readings for voltage conversion
        adc_match = re.search(r'(?:adc)[:=]\s*(\d+)', line, re.IGNORECASE)
        if adc_match:
            try:
                adc_value = int(adc_match.group(1))
                voltage = self._convert_adc_to_voltage(adc_value)
                self.voltage_update.emit(voltage)
            except ValueError:
                pass
        else:
            # Fallback to explicit voltage field if ADC missing
            voltage_match = re.search(r'(?:voltage|volt)[:=]\s*([\d.]+)v?', line, re.IGNORECASE)
            if voltage_match:
                try:
                    voltage = float(voltage_match.group(1))
                    self.voltage_update.emit(voltage)
                except ValueError:
                    pass

        # Look for speed: "Speed: 1.5" or "SPD:1.5"
        speed_match = re.search(r'(?:speed|spd)[:=]\s*([\d.]+)', line, re.IGNORECASE)
        if speed_match:
            try:
                speed = float(speed_match.group(1))
                self.speed_update.emit(speed)
            except ValueError:
                pass

        # Look for obstacle detection in BLE notifications
        if "[NOTIF]" in line:
            # Extract the notification content
            notif_content = line.split("[NOTIF]", 1)[1].strip() if "[NOTIF]" in line else line
            notif_lower = notif_content.lower()
            
            # Check for obstacle keywords in notification
            if "obstacle" in notif_lower and "no_obstacle" not in notif_lower:
                self.obstacle_detected.emit(True)
            elif "no_obstacle" in notif_lower:
                self.obstacle_detected.emit(False)
            
            # Check for motion commands in notifications
            if "forward" in notif_lower:
                self.motion_update.emit("FORWARD")
            elif "backward" in notif_lower:
                self.motion_update.emit("BACKWARD")
            elif "left" in notif_lower:
                self.motion_update.emit("LEFT")
            elif "right" in notif_lower:
                self.motion_update.emit("RIGHT")
            elif "idle" in notif_lower or "stop" in notif_lower:
                self.motion_update.emit("IDLE")
        
        # Also check for direct obstacle format: "Obstacle: true/false" or "OBS:1/0"
        obstacle_match = re.search(r'(?:obstacle|obs)[:=]\s*(?:(true|1|detected)|(false|0|clear))', line, re.IGNORECASE)
        if obstacle_match:
            detected = obstacle_match.group(1) is not None
            self.obstacle_detected.emit(detected)

    def _convert_adc_to_voltage(self, adc_value: int) -> float:
        adc_value = max(0, min(ADC_RESOLUTION, adc_value))
        span = BATTERY_MAX_V - BATTERY_MIN_V
        if span <= 0:
            return BATTERY_MIN_V
        return BATTERY_MIN_V + (adc_value / ADC_RESOLUTION) * span

    def send_line(self, text: str):
        if not (self.ser and self.ser.is_open):
            self.log_message.emit("[UI] Not connected.")
            return False
        try:
            data = (text + LINE_ENDING).encode("utf-8", errors="ignore")
            self.ser.write(data)
            self.ser.flush()
            self.log_message.emit(f"[PC->ESP] {text}")
            return True
        except Exception as e:
            self.log_message.emit(f"[ERR] Write failed: {e}")
            return False

    def close_serial(self):
        self._stop.set()
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        self.wait(1000)  # Wait up to 1 second for thread to finish


class JoystickWorker(QThread):
    """Background joystick poller emitting controller presence and stick vectors."""
    controller_status = Signal(bool, str)
    vector_ready = Signal(float, float)
    warning = Signal(str)

    def __init__(self, poll_interval: float = 0.05, deadzone: float = 0.08):
        super().__init__()
        self.poll_interval = poll_interval
        self.deadzone = deadzone
        self._stop = threading.Event()
        self._controller_index: Optional[int] = None
        self._controller_name = ""
        self._controller_handle = None
        self._has_controller = False

    def stop(self):
        """Signal the worker to finish."""
        self._stop.set()
        self.wait(500)

    def run(self):
        if not PYGAME_AVAILABLE:
            self.warning.emit("pygame not available; install it to enable joystick override.")
            return
        try:
            pygame.init()
            pygame.joystick.init()
        except Exception as exc:
            self.warning.emit(f"Unable to initialize joystick input: {exc}")
            return

        try:
            while not self._stop.is_set():
                pygame.event.pump()
                controller = self._acquire_controller()
                if controller:
                    try:
                        raw_x = controller.get_axis(0)
                        raw_y = controller.get_axis(1)
                    except Exception as exc:
                        self.warning.emit(f"Joystick read error: {exc}")
                        self._controller_index = None
                        self._controller_handle = None
                        self._notify_status(False)
                        time.sleep(self.poll_interval)
                        continue

                    x = self._apply_deadzone(raw_x)
                    y = self._apply_deadzone(-raw_y)  # invert so up on screen is positive
                    self.vector_ready.emit(x, y)

                time.sleep(self.poll_interval)
        finally:
            try:
                pygame.joystick.quit()
                pygame.quit()
            except Exception:
                pass
            if self._has_controller:
                self.controller_status.emit(False, "")

    def _acquire_controller(self):
        count = pygame.joystick.get_count()
        if count == 0:
            self._controller_index = None
            self._controller_handle = None
            self._notify_status(False)
            return None

        target_index = None
        target_name = ""

        for idx in range(count):
            js = pygame.joystick.Joystick(idx)
            name = js.get_name() or f"Controller {idx + 1}"
            if not js.get_init():
                js.init()
            if target_index is None:
                target_index = idx
                target_name = name
            if "xbox" in name.lower():
                target_index = idx
                target_name = name
                break

        if target_index is None:
            self._notify_status(False)
            self._controller_index = None
            self._controller_handle = None
            return None

        if self._controller_index != target_index or self._controller_handle is None:
            self._controller_index = target_index
            self._controller_handle = pygame.joystick.Joystick(target_index)
            if not self._controller_handle.get_init():
                self._controller_handle.init()

        self._notify_status(True, target_name)
        return self._controller_handle

    def _apply_deadzone(self, value: float) -> float:
        if abs(value) < self.deadzone:
            return 0.0
        return max(-1.0, min(1.0, value))

    def _notify_status(self, connected: bool, name: str = ""):
        if connected:
            if (not self._has_controller) or (name != self._controller_name):
                self.controller_status.emit(True, name)
            self._has_controller = True
            self._controller_name = name
        else:
            if self._has_controller:
                self.controller_status.emit(False, "")
            self._has_controller = False
            self._controller_name = ""


class ModernButton(QPushButton):
    """Custom button with modern styling and hover effects."""
    
    def __init__(self, text, primary=False):
        super().__init__(text)
        self.primary = primary
        self.setMinimumHeight(35)
        self.setFont(QFont("Inter", 9, QFont.Weight.Bold))
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.update_style()

    def update_style(self):
        if self.primary:
            self.setStyleSheet("""
                ModernButton {
                    background-color: #3b82f6;
                    color: white;
                    border: none;
                    border-radius: 6px;
                    padding: 8px 16px;
                    font-weight: bold;
                }
                ModernButton:hover {
                    background-color: #2563eb;
                }
                ModernButton:pressed {
                    background-color: #1d4ed8;
                }
                ModernButton:disabled {
                    background-color: #64748b;
                    color: #94a3b8;
                }
            """)
        else:
            self.setStyleSheet("""
                ModernButton {
                    background-color: #374151;
                    color: #e2e8f0;
                    border: none;
                    border-radius: 6px;
                    padding: 8px 16px;
                    font-weight: bold;
                }
                ModernButton:hover {
                    background-color: #4b5563;
                }
                ModernButton:pressed {
                    background-color: #6b7280;
                }
                ModernButton:disabled {
                    background-color: #1f2937;
                    color: #6b7280;
                }
            """)


class StatusLabel(QLabel):
    """Status label with colored indicators."""
    
    def __init__(self, text="Disconnected"):
        super().__init__(text)
        self.setFont(QFont("Inter", 10, QFont.Weight.Bold))
        self.set_disconnected()

    def set_connected(self, port=""):
        self.setText(f"● Connected to {port}")
        self.setStyleSheet("""
            StatusLabel {
                background-color: #059669;
                color: white;
                border-radius: 12px;
                padding: 8px 16px;
            }
        """)

    def set_disconnected(self):
        self.setText("● Disconnected")
        self.setStyleSheet("""
            StatusLabel {
                background-color: #6b7280;
                color: white;
                border-radius: 12px;
                padding: 8px 16px;
            }
        """)

    def set_error(self):
        self.setText("● Connection Failed")
        self.setStyleSheet("""
            StatusLabel {
                background-color: #dc2626;
                color: white;
                border-radius: 12px;
                padding: 8px 16px;
            }
        """)


class ModernProgressBar(QProgressBar):
    """Custom progress bar with modern styling."""
    
    def __init__(self):
        super().__init__()
        self.setTextVisible(True)
        self.setMinimumHeight(25)
        self.setStyleSheet("""
            QProgressBar {
                border: none;
                border-radius: 12px;
                background-color: #1e293b;
                color: white;
                font-weight: bold;
                text-align: center;
            }
            QProgressBar::chunk {
                border-radius: 12px;
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #10b981, stop:0.6 #34d399, stop:1 #6ee7b7);
            }
            QProgressBar::chunk:disabled {
                background-color: #374151;
            }
        """)


class LineGraphWidget(QWidget):
    """Custom line graph widget for displaying current and voltage values."""
    
    def __init__(self):
        super().__init__()
        self.setMinimumHeight(150)
        self.current_data = []  # List of (timestamp, current_value) tuples
        self.voltage_data = []  # List of (timestamp, voltage_value) tuples
        self.max_points = 50   # Maximum number of points to display
        self.current_value = 0.0
        self.voltage_value = 0.0
        self.has_current_data = False
        self.has_voltage_data = False
        
        # Graph styling
        self.bg_color = QColor("#1e293b")
        self.grid_color = QColor("#334155")
        self.current_color = QColor("#3b82f6")  # Blue for current
        self.voltage_color = QColor("#f59e0b")  # Orange for voltage
        self.text_color = QColor("#e2e8f0")
        
        self.setStyleSheet("""
            LineGraphWidget {
                background-color: #1e293b;
                border: 1px solid #334155;
                border-radius: 8px;
            }
        """)

    def add_current_data(self, value: float):
        """Add new current data point."""
        import time
        timestamp = time.time()
        self.current_data.append((timestamp, value))
        self.current_value = value
        self.has_current_data = True
        
        # Keep only the last max_points
        if len(self.current_data) > self.max_points:
            self.current_data = self.current_data[-self.max_points:]
        
        self.update()

    def add_voltage_data(self, value: float):
        """Add new voltage data point."""
        import time
        timestamp = time.time()
        self.voltage_data.append((timestamp, value))
        self.voltage_value = value
        self.has_voltage_data = True
        
        # Keep only the last max_points
        if len(self.voltage_data) > self.max_points:
            self.voltage_data = self.voltage_data[-self.max_points:]
        
        self.update()

    def paintEvent(self, event):
        """Paint the line graph."""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        
        # Fill background
        painter.fillRect(self.rect(), self.bg_color)
        
        # Set up drawing area with margins
        margin = 20
        graph_rect = self.rect().adjusted(margin, margin, -margin, -margin)
        
        if graph_rect.width() <= 0 or graph_rect.height() <= 0:
            return
        
        # Draw grid
        painter.setPen(QPen(self.grid_color, 1))
        
        # Vertical grid lines
        for i in range(5):
            x = graph_rect.left() + (i * graph_rect.width() // 4)
            painter.drawLine(x, graph_rect.top(), x, graph_rect.bottom())
        
        # Horizontal grid lines
        for i in range(4):
            y = graph_rect.top() + (i * graph_rect.height() // 3)
            painter.drawLine(graph_rect.left(), y, graph_rect.right(), y)
        
        has_current_points = len(self.current_data) > 1
        has_voltage_points = len(self.voltage_data) > 1
        
        # Draw current line
        if has_current_points:
            self._draw_line(painter, self.current_data, self.current_color, graph_rect, 0.0, 1.0)
        
        # Draw voltage line
        if has_voltage_points:
            self._draw_line(painter, self.voltage_data, self.voltage_color, graph_rect, 0.0, 8.0)
        
        # Draw current values
        painter.setPen(QPen(self.text_color, 1))
        painter.setFont(QFont("Inter", 10, QFont.Weight.Bold))
        
        # Current value
        # Current value
        if self.has_current_data:
            painter.setPen(QPen(self.current_color, 1))
            painter.drawText(graph_rect.left(), graph_rect.top() - 5, f"Current: {self.current_value:.2f}A")
        
        # Voltage value
        if self.has_voltage_data:
            painter.setPen(QPen(self.voltage_color, 1))
            painter.drawText(graph_rect.left() + 120, graph_rect.top() - 5, f"Voltage: {self.voltage_value:.2f}V")

        if not self.has_current_data and not self.has_voltage_data:
            painter.setPen(QPen(self.text_color, 1))
            painter.drawText(
                graph_rect,
                int(Qt.AlignmentFlag.AlignCenter | Qt.AlignmentFlag.AlignVCenter),
                "Waiting for sensor data..."
            )

    def _draw_line(self, painter, data, color, rect, min_override=None, max_override=None):
        """Draw a line for the given data."""
        if len(data) < 2:
            return
        
        painter.setPen(QPen(color, 2))
        
        # Calculate value ranges
        values = [point[1] for point in data]
        min_val = min_override if min_override is not None else min(values)
        max_val = max_override if max_override is not None else max(values)
        
        # Avoid division by zero
        if max_val == min_val:
            max_val = min_val + 1
        
        # Calculate time range
        timestamps = [point[0] for point in data]
        min_time = min(timestamps)
        max_time = max(timestamps)
        time_range = max_time - min_time
        
        if time_range == 0:
            time_range = 1
        
        # Draw the line
        points = []
        for i, (timestamp, value) in enumerate(data):
            x = rect.left() + int((timestamp - min_time) / time_range * rect.width())
            y = rect.bottom() - int((value - min_val) / (max_val - min_val) * rect.height())
            points.append((x, y))
        
        # Draw line segments
        for i in range(len(points) - 1):
            painter.drawLine(points[i][0], points[i][1], points[i+1][0], points[i+1][1])


class JoystickDiagram(QWidget):
    """Simple XY diagram that reflects joystick displacement."""

    def __init__(self):
        super().__init__()
        self.setMinimumHeight(220)
        self._x = 0.0
        self._y = 0.0
        self._active = False
        self.setStyleSheet("""
            JoystickDiagram {
                background-color: #0f172a;
                border: 1px solid #334155;
                border-radius: 10px;
            }
        """)

    def set_position(self, x: float, y: float):
        self._x = max(-1.0, min(1.0, x))
        self._y = max(-1.0, min(1.0, y))
        self.update()

    def set_active(self, active: bool):
        self._active = active
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        rect = self.rect().adjusted(18, 18, -18, -18)
        if rect.width() <= 0 or rect.height() <= 0:
            return

        painter.fillRect(rect, QColor("#0b1220"))

        radius = min(rect.width(), rect.height()) / 2
        center = rect.center()

        # Outer boundary
        painter.setPen(QPen(QColor("#334155"), 2))
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.drawEllipse(center, radius, radius)

        # Mid circle
        painter.setPen(QPen(QColor("#1f2937"), 1, Qt.PenStyle.DashLine))
        painter.drawEllipse(center, radius * 0.5, radius * 0.5)

        # Crosshair
        painter.setPen(QPen(QColor("#293149"), 1))
        painter.drawLine(center.x() - radius, center.y(), center.x() + radius, center.y())
        painter.drawLine(center.x(), center.y() - radius, center.x(), center.y() + radius)

        # Position indicator
        dot_color = QColor("#10b981" if self._active else "#475569")
        painter.setBrush(dot_color)
        painter.setPen(Qt.PenStyle.NoPen)
        dot_x = center.x() + self._x * radius
        dot_y = center.y() - self._y * radius
        painter.drawEllipse(QPointF(dot_x, dot_y), 9, 9)


class ESP32RemoteGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.serial_worker = SerialWorker()
        self.joystick_worker: Optional[JoystickWorker] = None
        self.controller_available = False
        self.joystick_active = False
        self.last_joystick_vector = (0.0, 0.0)
        self.last_joystick_send_ts = 0.0
        self.controls_enabled = False
        self.joystick_diagram: Optional[JoystickDiagram] = None
        self.joystick_status_badge: Optional[QLabel] = None
        self.controller_status_label: Optional[QLabel] = None
        self.joystick_coord_label: Optional[QLabel] = None
        self.smoothed_voltage: Optional[float] = None
        self.battery_percent = 0
        self.battery_percent_smoothed: Optional[float] = None
        self.battery_bar: Optional[ModernProgressBar] = None
        self.battery_voltage_label: Optional[QLabel] = None
        self.motor_mode = "IDLE"
        self.current_target = 0.18
        self.current_variation = 0.02
        self.speed_override_active = False
        self.speed_override_value: Optional[float] = None

        self.setup_connections()
        self.init_ui()
        self.init_joystick_support()
        self.setup_shortcuts()
        self.refresh_ports(auto_select=True)

        self.current_timer = QTimer()
        self.current_timer.setInterval(800)
        self.current_timer.timeout.connect(self.generate_current_sample)
        self.current_timer.start()
        
    def setup_connections(self):
        """Connect serial worker signals to GUI slots."""
        self.serial_worker.log_message.connect(self.append_log)
        self.serial_worker.connected.connect(self.on_serial_connected)
        self.serial_worker.disconnected.connect(self.on_serial_disconnected)
        self.serial_worker.current_update.connect(self.update_current)
        self.serial_worker.voltage_update.connect(self.update_voltage)
        self.serial_worker.speed_update.connect(self.on_speed_telemetry)
        self.serial_worker.obstacle_detected.connect(self.update_obstacle_status)
        self.serial_worker.motion_update.connect(self.update_motion_status)

    def init_joystick_support(self):
        """Initialize joystick worker if pygame support is available."""
        if not hasattr(self, "joystick_check"):
            return

        if not PYGAME_AVAILABLE:
            if self.controller_status_label:
                self.controller_status_label.setText("Controller: pygame missing")
                self.controller_status_label.setStyleSheet("color: #f97316; font-size: 10px;")
            self.joystick_check.setToolTip("Install pygame to enable joystick override.")
            self.joystick_check.setEnabled(False)
            if self.joystick_status_badge:
                self.joystick_status_badge.setText("Joystick Override Unavailable")
            return

        self.joystick_worker = JoystickWorker()
        self.joystick_worker.controller_status.connect(self.on_controller_status)
        self.joystick_worker.vector_ready.connect(self.on_joystick_vector)
        self.joystick_worker.warning.connect(lambda msg: self.append_log(f"[JOYSTICK] {msg}"))
        self.joystick_worker.start()
        if self.controller_status_label:
            self.controller_status_label.setText("Controller: Searching...")
            self.controller_status_label.setStyleSheet("color: #94a3b8; font-size: 10px;")

    def init_ui(self):
        """Initialize the user interface."""
        self.setWindowTitle("ESP32 BLE Remote Control")
        self.setGeometry(100, 100, 1200, 700)
        self.setMinimumSize(1000, 650)
        
        # Apply dark theme
        self.apply_dark_theme()
        
        # Central widget and main layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(24, 24, 24, 24)
        main_layout.setSpacing(20)

        # Header
        self.create_header(main_layout)
        
        # Main content area
        content_splitter = QSplitter(Qt.Orientation.Horizontal)
        main_layout.addWidget(content_splitter)
        
        # Left panel (controls)
        left_panel = self.create_left_panel()
        content_splitter.addWidget(left_panel)
        
        # Right panel (console and metrics)
        right_panel = self.create_right_panel()
        content_splitter.addWidget(right_panel)
        
        # Set splitter proportions
        content_splitter.setSizes([400, 800])
        content_splitter.setStretchFactor(0, 0)
        content_splitter.setStretchFactor(1, 1)

    def apply_dark_theme(self):
        """Apply modern dark theme to the application."""
        palette = QPalette()
        
        # Window colors
        palette.setColor(QPalette.ColorRole.Window, QColor("#0f1419"))
        palette.setColor(QPalette.ColorRole.WindowText, QColor("#e2e8f0"))
        
        # Base colors (input fields)
        palette.setColor(QPalette.ColorRole.Base, QColor("#1e293b"))
        palette.setColor(QPalette.ColorRole.Text, QColor("#e2e8f0"))
        
        # Button colors
        palette.setColor(QPalette.ColorRole.Button, QColor("#374151"))
        palette.setColor(QPalette.ColorRole.ButtonText, QColor("#e2e8f0"))
        
        # Highlight colors
        palette.setColor(QPalette.ColorRole.Highlight, QColor("#3b82f6"))
        palette.setColor(QPalette.ColorRole.HighlightedText, QColor("#ffffff"))
        
        self.setPalette(palette)
        
        # Apply global stylesheet
        self.setStyleSheet("""
            QMainWindow {
                background-color: #0f1419;
                color: #e2e8f0;
            }
            QGroupBox {
                background-color: #1a1f2e;
                border: 1px solid #334155;
                border-radius: 8px;
                margin-top: 10px;
                padding-top: 10px;
                font-weight: bold;
                font-size: 12px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 12px;
                padding: 0 8px 0 8px;
                color: #f8fafc;
            }
            QLineEdit, QComboBox {
                background-color: #1e293b;
                border: 1px solid #334155;
                border-radius: 6px;
                padding: 8px 12px;
                color: #e2e8f0;
                font-size: 10px;
            }
            QLineEdit:focus, QComboBox:focus {
                border-color: #3b82f6;
            }
            QComboBox::drop-down {
                border: none;
                width: 20px;
            }
            QComboBox::down-arrow {
                image: none;
                border: 4px solid transparent;
                border-top: 6px solid #94a3b8;
                margin-right: 8px;
            }
            QTextEdit {
                background-color: #0d1117;
                border: 1px solid #30363d;
                border-radius: 8px;
                color: #c9d1d9;
                font-family: 'JetBrains Mono', 'Courier New', monospace;
                font-size: 10px;
                padding: 12px;
            }
            QCheckBox {
                color: #e2e8f0;
                spacing: 8px;
            }
            QCheckBox::indicator {
                width: 18px;
                height: 18px;
                border-radius: 3px;
                border: 2px solid #475569;
                background-color: #1e293b;
            }
            QCheckBox::indicator:checked {
                background-color: #3b82f6;
                border-color: #3b82f6;
            }
            QCheckBox::indicator:checked:hover {
                background-color: #2563eb;
            }
            QLabel {
                color: #e2e8f0;
            }
            QSplitter::handle {
                background-color: #334155;
                width: 2px;
            }
        """)

    def create_header(self, layout):
        """Create the header section."""
        header_layout = QHBoxLayout()
        
        # Title
        title = QLabel("ESP32 BLE Remote Control")
        title.setFont(QFont("Inter", 18, QFont.Weight.Bold))
        title.setStyleSheet("color: #f8fafc; margin-bottom: 8px;")
        header_layout.addWidget(title)
        
        header_layout.addStretch()
        
        # Status indicator
        self.status_label = StatusLabel()
        header_layout.addWidget(self.status_label)
        
        layout.addLayout(header_layout)
        
        # Subtitle
        subtitle = QLabel("Connect to your ESP32, send commands, and monitor BLE communication in real-time.")
        subtitle.setFont(QFont("Inter", 10))
        subtitle.setStyleSheet("color: #94a3b8; margin-bottom: 16px;")
        layout.addWidget(subtitle)

    def create_left_panel(self):
        """Create the left control panel."""
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        left_layout.setSpacing(16)
        
        # Connection settings
        self.create_connection_group(left_layout)

        # Override modes
        self.create_override_group(left_layout)
        
        # Robot control
        self.create_control_group(left_layout)
        
        left_layout.addStretch()
        return left_widget

    def create_connection_group(self, layout):
        """Create connection settings group."""
        conn_group = QGroupBox("Connection Settings")
        conn_layout = QVBoxLayout(conn_group)
        conn_layout.setSpacing(12)
        
        # Port selection
        port_layout = QHBoxLayout()
        port_layout.addWidget(QLabel("Serial Port:"))
        
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(200)
        port_layout.addWidget(self.port_combo)
        
        self.refresh_btn = ModernButton("Refresh")
        self.refresh_btn.clicked.connect(self.refresh_ports)
        port_layout.addWidget(self.refresh_btn)
        
        conn_layout.addLayout(port_layout)
        
        # Baud rate
        baud_layout = QHBoxLayout()
        baud_layout.addWidget(QLabel("Baud Rate:"))
        
        self.baud_edit = QLineEdit(str(DEFAULT_BAUD))
        self.baud_edit.setMaximumWidth(100)
        baud_layout.addWidget(self.baud_edit)
        baud_layout.addStretch()
        
        conn_layout.addLayout(baud_layout)
        
        # Control options
        options_label = QLabel("Control Options:")
        options_label.setStyleSheet("font-weight: bold; margin-top: 8px;")
        conn_layout.addWidget(options_label)
        
        self.dtr_check = QCheckBox("DTR Enable")
        self.dtr_check.setChecked(True)
        conn_layout.addWidget(self.dtr_check)
        
        self.rts_check = QCheckBox("RTS Enable")
        conn_layout.addWidget(self.rts_check)
        
        self.reset_check = QCheckBox("Reset on Connect")
        conn_layout.addWidget(self.reset_check)
        
        # Connection buttons
        button_layout = QHBoxLayout()
        self.connect_btn = ModernButton("Connect", primary=True)
        self.connect_btn.clicked.connect(self.connect_serial)
        button_layout.addWidget(self.connect_btn)
        
        self.disconnect_btn = ModernButton("Disconnect")
        self.disconnect_btn.clicked.connect(self.disconnect_serial)
        self.disconnect_btn.setEnabled(False)
        button_layout.addWidget(self.disconnect_btn)
        
        conn_layout.addLayout(button_layout)
        layout.addWidget(conn_group)

    def create_override_group(self, layout):
        """Create override selection group."""
        override_group = QGroupBox("Override Modes")
        override_layout = QVBoxLayout(override_group)
        override_layout.setSpacing(12)

        hint = QLabel("Select one override method at a time. Joystick mode needs an Xbox controller.")
        hint.setWordWrap(True)
        hint.setStyleSheet("color: #94a3b8; font-size: 10px;")
        override_layout.addWidget(hint)

        self.override_check = QCheckBox("Override Autonomous Mode")
        self.override_check.toggled.connect(self.on_override_toggled)
        override_layout.addWidget(self.override_check)

        joystick_row = QHBoxLayout()
        joystick_row.setSpacing(8)

        self.joystick_check = QCheckBox("Joystick Override")
        self.joystick_check.toggled.connect(self.on_joystick_toggled)
        self.joystick_check.setToolTip("Requires an Xbox controller.")
        joystick_row.addWidget(self.joystick_check)

        self.controller_status_label = QLabel("Controller: Not Detected")
        self.controller_status_label.setStyleSheet("color: #f87171; font-size: 10px;")
        joystick_row.addWidget(self.controller_status_label)
        joystick_row.addStretch()

        override_layout.addLayout(joystick_row)
        layout.addWidget(override_group)

    def create_control_group(self, layout):
        """Create robot control group."""
        control_group = QGroupBox("Robot Control")
        control_layout = QVBoxLayout(control_group)
        control_layout.setSpacing(16)
        
        # Movement controls
        movement_label = QLabel("Movement Controls:")
        movement_label.setStyleSheet("font-weight: bold;")
        control_layout.addWidget(movement_label)
        
        nav_layout = QGridLayout()
        nav_layout.setSpacing(4)
        
        # Navigation buttons
        self.fwd_btn = ModernButton("↑ Forward(1)")
        self.fwd_btn.clicked.connect(lambda: self.send_digit_with_visual_feedback(1))
        nav_layout.addWidget(self.fwd_btn, 0, 1)
        
        self.left_btn = ModernButton("← Left(3)")
        self.left_btn.clicked.connect(lambda: self.send_digit_with_visual_feedback(3))
        nav_layout.addWidget(self.left_btn, 1, 0)
        
        self.back_btn = ModernButton("↓ Backward(2)")
        self.back_btn.clicked.connect(lambda: self.send_digit_with_visual_feedback(2))
        nav_layout.addWidget(self.back_btn, 1, 1)
        
        self.right_btn = ModernButton("→ Right(4)")
        self.right_btn.clicked.connect(lambda: self.send_digit_with_visual_feedback(4))
        nav_layout.addWidget(self.right_btn, 1, 2)
        
        control_layout.addLayout(nav_layout)
        
        # Custom command
        custom_label = QLabel("Custom Command:")
        custom_label.setStyleSheet("font-weight: bold; margin-top: 16px;")
        control_layout.addWidget(custom_label)
        
        custom_layout = QHBoxLayout()
        self.custom_edit = QLineEdit()
        self.custom_edit.setPlaceholderText("Enter command...")
        self.custom_edit.returnPressed.connect(self.send_custom)
        custom_layout.addWidget(self.custom_edit)
        
        self.send_btn = ModernButton("Send", primary=True)
        self.send_btn.clicked.connect(self.send_custom)
        custom_layout.addWidget(self.send_btn)
        
        control_layout.addLayout(custom_layout)
        
        # Store control widgets for enabling/disabling
        self.control_widgets = [
            self.override_check, self.joystick_check, self.fwd_btn, self.back_btn,
            self.left_btn, self.right_btn, self.custom_edit, self.send_btn
        ]
        self.set_controls_enabled(False)
        
        layout.addWidget(control_group)

    def create_right_panel(self):
        """Create the right panel with console and metrics."""
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.setSpacing(16)
        
        # Communication console and motion display (split 3:1)
        console_layout = QHBoxLayout()
        
        # Communication console (3 parts of the ratio)
        console_group = QGroupBox("Communication Console")
        console_group_layout = QVBoxLayout(console_group)
        
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMinimumHeight(200)
        console_group_layout.addWidget(self.log_text)
        
        console_layout.addWidget(console_group, 3)  # 3 parts of the ratio
        
        # Motion display (1 part of the ratio)
        motion_group = QGroupBox("Robot Motion")
        motion_group_layout = QVBoxLayout(motion_group)
        
        self.motion_label = QLabel("IDLE")
        self.motion_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.motion_label.setFont(QFont("Inter", 16, QFont.Weight.Bold))
        self.motion_label.setStyleSheet("color: #6b7280; margin: 20px 0; padding: 10px; border: 2px solid #374151; border-radius: 8px;")
        motion_group_layout.addWidget(self.motion_label)
        
        self.motion_status_label = QLabel("Status: Idle")
        self.motion_status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.motion_status_label.setStyleSheet("color: #94a3b8; font-size: 10px;")
        motion_group_layout.addWidget(self.motion_status_label)
        
        console_layout.addWidget(motion_group, 1)  # 1 part of the ratio
        
        right_layout.addLayout(console_layout, stretch=1)

        # Metrics section
        metrics_layout = QVBoxLayout()
        metrics_layout.setSpacing(16)

        top_metrics_layout = QHBoxLayout()
        top_metrics_layout.setSpacing(16)
        
        # Line graph for current and voltage (top section)
        graph_group = QGroupBox("Current & Voltage Monitor")
        graph_layout = QVBoxLayout(graph_group)
        
        self.line_graph = LineGraphWidget()
        graph_layout.addWidget(self.line_graph)
        
        joystick_panel = self.create_joystick_panel()
        
        top_metrics_layout.addWidget(graph_group, 1)
        top_metrics_layout.addWidget(joystick_panel, 1)
        metrics_layout.addLayout(top_metrics_layout, stretch=2)
        
        # Bottom section with speed and obstacle detection
        bottom_layout = QHBoxLayout()
        
        # Speed indicator (1 part of the ratio) 
        speed_group = QGroupBox("Robot Speed")
        speed_layout = QVBoxLayout(speed_group)
        
        self.speed_label = QLabel("0.0")
        self.speed_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.speed_label.setFont(QFont("Inter", 20, QFont.Weight.Bold))
        self.speed_label.setStyleSheet("color: #3b82f6; margin: 12px 0;")
        speed_layout.addWidget(self.speed_label)
        
        self.speed_unit_label = QLabel("cm/min")
        self.speed_unit_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.speed_unit_label.setStyleSheet("color: #94a3b8; font-size: 9px;")
        speed_layout.addWidget(self.speed_unit_label)
        
        bottom_layout.addWidget(speed_group, 1)
        
        # Obstacle detection section (1 part of the ratio)
        obstacle_group = QGroupBox("Obstacle Detection")
        obstacle_layout = QVBoxLayout(obstacle_group)
        
        self.obstacle_label = QLabel("No Obstacle Detected")
        self.obstacle_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.obstacle_label.setFont(QFont("Inter", 12, QFont.Weight.Bold))
        self.obstacle_label.setStyleSheet("color: #10b981; margin: 16px 0;")  # Green for clear
        obstacle_layout.addWidget(self.obstacle_label)
        
        self.obstacle_status_label = QLabel("Status: Clear")
        self.obstacle_status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.obstacle_status_label.setStyleSheet("color: #94a3b8; font-size: 10px;")
        obstacle_layout.addWidget(self.obstacle_status_label)
        
        bottom_layout.addWidget(obstacle_group, 1)

        # Battery status section
        battery_group = QGroupBox("Battery Status")
        battery_layout = QVBoxLayout(battery_group)

        self.battery_bar = ModernProgressBar()
        self.battery_bar.setRange(0, 100)
        self.battery_bar.setValue(0)
        self.battery_bar.setFormat("--%")
        self.battery_bar.setStyleSheet("""
            QProgressBar {
                border: none;
                border-radius: 12px;
                background-color: #1e293b;
                color: white;
                font-weight: bold;
                text-align: center;
            }
            QProgressBar::chunk {
                border-radius: 12px;
                background-color: #475569;
            }
        """)
        battery_layout.addWidget(self.battery_bar)

        self.battery_voltage_label = QLabel("Voltage: --")
        self.battery_voltage_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.battery_voltage_label.setStyleSheet("color: #94a3b8; font-size: 10px;")
        battery_layout.addWidget(self.battery_voltage_label)

        bottom_layout.addWidget(battery_group, 1)
        
        metrics_layout.addLayout(bottom_layout, stretch=1)
        
        right_layout.addLayout(metrics_layout, stretch=2)
        
        return right_widget

    def create_joystick_panel(self):
        """Create joystick monitor panel with diagram."""
        joystick_group = QGroupBox("Joystick Override Monitor")
        joystick_layout = QVBoxLayout(joystick_group)
        joystick_layout.setSpacing(12)

        self.joystick_status_badge = QLabel("Joystick Override Inactive")
        self.joystick_status_badge.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.update_joystick_badge("Joystick Override Inactive")
        joystick_layout.addWidget(self.joystick_status_badge)

        self.joystick_diagram = JoystickDiagram()
        joystick_layout.addWidget(self.joystick_diagram)

        self.joystick_coord_label = QLabel("X: +0.00   Y: +0.00")
        self.joystick_coord_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.joystick_coord_label.setStyleSheet("color: #94a3b8; font-size: 10px;")
        joystick_layout.addWidget(self.joystick_coord_label)

        joystick_hint = QLabel("Joystick vectors are sent as 'J x y' while joystick override stays enabled.")
        joystick_hint.setWordWrap(True)
        joystick_hint.setStyleSheet("color: #64748b; font-size: 10px;")
        joystick_layout.addWidget(joystick_hint)

        return joystick_group

    def setup_shortcuts(self):
        """Setup keyboard shortcuts."""
        # Arrow key shortcuts
        QShortcut(QKeySequence(Qt.Key.Key_Up), self, lambda: self.send_digit_with_feedback(1))
        QShortcut(QKeySequence(Qt.Key.Key_Down), self, lambda: self.send_digit_with_feedback(2))
        QShortcut(QKeySequence(Qt.Key.Key_Left), self, lambda: self.send_digit_with_feedback(3))
        QShortcut(QKeySequence(Qt.Key.Key_Right), self, lambda: self.send_digit_with_feedback(4))
        
        # Escape to focus custom input
        QShortcut(QKeySequence(Qt.Key.Key_Escape), self, lambda: self.custom_edit.setFocus())

    def refresh_ports(self, auto_select=False):
        """Refresh the list of available serial ports."""
        ports = list_serial_ports()
        self.port_combo.clear()
        
        for port in ports:
            display_text = f"{port.device} - {port.desc}" if port.desc else port.device
            self.port_combo.addItem(display_text, port.device)
        
        if auto_select and ports:
            # Prefer USB serial adapters
            keywords = ("CH340", "CP210", "USB", "FTDI", "Silicon Labs", "UART", "CDC")
            for i, port in enumerate(ports):
                if any(keyword.lower() in port.desc.lower() for keyword in keywords):
                    self.port_combo.setCurrentIndex(i)
                    break

    def connect_serial(self):
        """Connect to the selected serial port."""
        if self.port_combo.count() == 0:
            self.append_log("[UI] No serial ports available.")
            return
        
        port = self.port_combo.currentData()
        if not port:
            self.append_log("[UI] Please select a serial port.")
            return
        
        try:
            baud = int(self.baud_edit.text().strip())
        except ValueError:
            self.append_log("[UI] Invalid baud rate; using 115200.")
            baud = DEFAULT_BAUD
        
        # Apply control line settings
        self.serial_worker.set_control_lines(
            self.dtr_check.isChecked(),
            self.rts_check.isChecked(),
            self.reset_check.isChecked()
        )
        
        success = self.serial_worker.connect_serial(port, baud)
        if success:
            self.status_label.set_connected(port)
            actual_baud = self.serial_worker.connected_baud or baud
            self.baud_edit.setText(str(actual_baud))
            self.connect_btn.setEnabled(False)
            self.disconnect_btn.setEnabled(True)
            self.set_controls_enabled(True)
        else:
            self.status_label.set_error()

    def disconnect_serial(self):
        """Disconnect from the serial port."""
        self.serial_worker.close_serial()
        self.status_label.set_disconnected()
        self.connect_btn.setEnabled(True)
        self.disconnect_btn.setEnabled(False)
        self.set_controls_enabled(False)
        self.append_log("[SER] Disconnected.")

    def on_serial_connected(self):
        """Handle serial connection established."""
        pass  # Status already updated in connect_serial

    def on_serial_disconnected(self):
        """Handle serial connection lost."""
        self.status_label.set_disconnected()
        self.connect_btn.setEnabled(True)
        self.disconnect_btn.setEnabled(False)
        self.set_controls_enabled(False)
        self.append_log("[SER] Connection lost.")

    def set_controls_enabled(self, enabled: bool):
        """Enable or disable control widgets."""
        self.controls_enabled = enabled
        for widget in self.control_widgets:
            widget.setEnabled(enabled)
        if enabled:
            self.update_mode_toggles()
        else:
            self.smoothed_voltage = None
            self.reset_battery_display()
            self.set_motor_mode("IDLE")
            if hasattr(self, "speed_label"):
                self.set_speed_override(None)

    def update_mode_toggles(self):
        """Keep override toggles mutually exclusive and gated."""
        if not self.controls_enabled:
            return

        joystick_enabled = self.controller_available and not self.override_check.isChecked()
        if self.joystick_check.isChecked():
            joystick_enabled = True
        self.joystick_check.setEnabled(joystick_enabled)

        override_enabled = not self.joystick_check.isChecked() or self.override_check.isChecked()
        self.override_check.setEnabled(override_enabled)

    def update_joystick_badge(self, text: str, color: str = "#94a3b8"):
        """Update status badge styling."""
        if not self.joystick_status_badge:
            return
        self.joystick_status_badge.setText(text)
        self.joystick_status_badge.setStyleSheet(
            f"color: {color}; font-weight: bold; padding: 6px 8px; border: 1px solid {color}; border-radius: 6px;"
        )

    def set_motor_mode(self, mode: str):
        """Adjust synthetic current targets based on motor state."""
        mode = mode.upper()
        if mode == self.motor_mode:
            return
        self.motor_mode = mode
        if mode == "DUAL":
            self.current_target = 0.25
            self.current_variation = 0.03
        elif mode == "SINGLE":
            self.current_target = 0.021
            self.current_variation = 0.005
        else:
            self.current_target = 0.18
            self.current_variation = 0.02

    def generate_current_sample(self):
        """Generate pseudo current data based on motion state."""
        if not self.controls_enabled:
            return
        base = self.current_target
        variation = self.current_variation
        value = base + random.gauss(0, variation / 2)
        dual_floor = 0.17 if self.motor_mode == "DUAL" else 0.01
        value = min(1.0, max(dual_floor, value))
        self.line_graph.add_current_data(value)

    def reset_battery_display(self):
        """Clear battery UI when no data is available."""
        self.battery_percent = 0
        self.battery_percent_smoothed = None
        if self.battery_bar:
            self.battery_bar.setValue(0)
            self.battery_bar.setFormat("--%")
            self.battery_bar.setStyleSheet("""
                QProgressBar {
                    border: none;
                    border-radius: 12px;
                    background-color: #1e293b;
                    color: white;
                    font-weight: bold;
                    text-align: center;
                }
                QProgressBar::chunk {
                    border-radius: 12px;
                    background-color: #475569;
                }
            """)
        if self.battery_voltage_label:
            self.battery_voltage_label.setText("Voltage: --")

    def update_battery_from_voltage(self, voltage: float):
        """Update battery percentage and UI based on voltage measurement."""
        span = BATTERY_MAX_V - BATTERY_MIN_V
        if span <= 0:
            return
        percent = max(0.0, min(100.0, ((voltage - BATTERY_MIN_V) / span) * 100.0))
        if self.battery_percent_smoothed is None:
            smooth_percent = percent
        else:
            smooth_percent = 0.05 * percent + 0.95 * self.battery_percent_smoothed
        self.battery_percent_smoothed = smooth_percent
        display_percent = max(0, min(100, int(round(smooth_percent))))
        self.battery_percent = display_percent
        if self.battery_bar:
            self.battery_bar.setValue(display_percent)
            self.battery_bar.setFormat(f"{display_percent}%")
            if display_percent > 70:
                color = "#10b981"
            elif display_percent > 35:
                color = "#facc15"
            else:
                color = "#ef4444"
            self.battery_bar.setStyleSheet(f"""
                QProgressBar {{
                    border: none;
                    border-radius: 12px;
                    background-color: #1e293b;
                    color: white;
                    font-weight: bold;
                    text-align: center;
                }}
                QProgressBar::chunk {{
                    border-radius: 12px;
                    background-color: {color};
                }}
            """)
        if self.battery_voltage_label:
            self.battery_voltage_label.setText(f"Voltage: {voltage:.2f} V")

    def send_digit(self, digit: int):
        """Send a digit command."""
        if 1 <= digit <= 4:
            self.serial_worker.send_line(str(digit))
            
            # Update motion status if in override mode
            if self.override_check.isChecked():
                motion_map = {1: "FORWARD", 2: "BACKWARD", 3: "LEFT", 4: "RIGHT"}
                if digit in motion_map:
                    self.update_motion_status(motion_map[digit])

    def send_digit_with_visual_feedback(self, digit: int):
        """Send digit with visual feedback for button clicks."""
        if not (1 <= digit <= 4):
            return
        
        # Get the corresponding button
        buttons = [self.fwd_btn, self.back_btn, self.left_btn, self.right_btn]
        btn = buttons[digit - 1]
        
        # Create animation for button press feedback
        self.animate_button_press(btn)
        
        # Send the command
        self.send_digit(digit)

    def send_digit_with_feedback(self, digit: int):
        """Send digit with visual feedback for keyboard shortcuts."""
        if not (1 <= digit <= 4):
            return
        
        # Get the corresponding button
        buttons = [self.fwd_btn, self.back_btn, self.left_btn, self.right_btn]
        btn = buttons[digit - 1]
        
        # Create animation for button press feedback
        self.animate_button_press(btn)
        
        # Send the command
        self.send_digit(digit)

    def animate_button_press(self, button):
        """Animate button press for visual feedback."""
        # Store original state
        original_primary = getattr(button, 'primary', False)
        
        # Set to primary (highlighted) state
        button.primary = True
        button.update_style()
        
        # Reset to original state after a short delay
        QTimer.singleShot(200, lambda: self.reset_button_style(button, original_primary))

    def reset_button_style(self, button, original_primary):
        """Reset button style after animation."""
        button.primary = original_primary
        button.update_style()

    def on_override_toggled(self, checked: bool):
        """Handle override checkbox toggle."""
        if checked and self.joystick_check.isChecked():
            self.joystick_check.blockSignals(True)
            self.joystick_check.setChecked(False)
            self.joystick_check.blockSignals(False)
            self.on_joystick_toggled(False)

        self.serial_worker.send_line("0")
        self.update_motion_status("IDLE")

        if checked:
            self.joystick_active = False
            self.update_joystick_badge("Joystick Override Locked", "#f59e0b")
        elif not self.joystick_check.isChecked():
            self.update_joystick_badge("Joystick Override Inactive")

        self.update_mode_toggles()

    def on_joystick_toggled(self, checked: bool):
        """Handle joystick override toggle."""
        if checked:
            if not self.controller_available:
                self.append_log("[JOYSTICK] Cannot enable joystick override without an Xbox controller.")
                self.joystick_check.blockSignals(True)
                self.joystick_check.setChecked(False)
                self.joystick_check.blockSignals(False)
                if self.controller_status_label:
                    self.controller_status_label.setStyleSheet("color: #f87171; font-size: 10px;")
                self.update_mode_toggles()
                return

            if self.override_check.isChecked():
                self.override_check.blockSignals(True)
                self.override_check.setChecked(False)
                self.override_check.blockSignals(False)
                self.on_override_toggled(False)

            self.serial_worker.send_line("6")
            self.joystick_active = True
            self.last_joystick_vector = (0.0, 0.0)
            self.last_joystick_send_ts = 0.0
            self.update_motion_status("IDLE")
            self.set_motor_mode("IDLE")
            if self.joystick_diagram:
                self.joystick_diagram.set_active(True)
            self.update_joystick_badge("Joystick Override Active", "#10b981")
            self.append_log("[JOYSTICK] Joystick override enabled (6).")
        else:
            was_active = self.joystick_active
            if self.joystick_active:
                self.serial_worker.send_line("6")
            self.joystick_active = False
            if self.joystick_diagram:
                self.joystick_diagram.set_active(False)
                self.joystick_diagram.set_position(0.0, 0.0)
            if self.joystick_coord_label:
                self.joystick_coord_label.setText("X: +0.00   Y: +0.00")
            if not self.override_check.isChecked():
                self.update_joystick_badge("Joystick Override Inactive")
            self.update_motion_status("IDLE")
            self.set_motor_mode("IDLE")
            if was_active:
                self.append_log("[JOYSTICK] Joystick override disabled (6).")

        self.update_mode_toggles()

    def on_controller_status(self, connected: bool, name: str):
        """Update UI when controller presence changes."""
        previous = self.controller_available
        self.controller_available = connected

        if connected:
            label = f"Controller: {name or 'Ready'}"
            color = "#10b981"
            self.joystick_check.setToolTip(f"Using {name or 'controller'}")
            if not previous:
                self.append_log(f"[JOYSTICK] Controller detected: {name or 'Unknown'}")
        else:
            label = "Controller: Not Detected"
            color = "#f87171"
            self.joystick_check.setToolTip("Requires an Xbox controller.")
            if self.joystick_check.isChecked():
                self.joystick_check.blockSignals(True)
                self.joystick_check.setChecked(False)
                self.joystick_check.blockSignals(False)
                self.on_joystick_toggled(False)
            if previous:
                self.append_log("[JOYSTICK] Controller disconnected.")

        if self.controller_status_label:
            self.controller_status_label.setText(label)
            self.controller_status_label.setStyleSheet(f"color: {color}; font-size: 10px;")

        self.update_mode_toggles()

    def on_joystick_vector(self, x: float, y: float):
        """Handle joystick vector updates."""
        if self.joystick_diagram:
            self.joystick_diagram.set_position(x, y)
        if self.joystick_coord_label:
            self.joystick_coord_label.setText(f"X: {x:+.2f}   Y: {y:+.2f}")

        if not self.joystick_active:
            return

        magnitude = math.hypot(x, y)
        forward_component = abs(y)
        turn_component = abs(x)
        if magnitude < 0.15:
            self.set_motor_mode("IDLE")
            self.apply_motion_speed_profile("IDLE")
        else:
            diag_motion = forward_component >= 0.2 and turn_component >= 0.2
            straight_motion = forward_component >= 0.2 and turn_component < 0.2
            turn_only = turn_component >= 0.2 and forward_component < 0.2

            if diag_motion:
                self.set_motor_mode("SINGLE")
                self.apply_motion_speed_profile("DIAGONAL")
            elif straight_motion:
                self.set_motor_mode("DUAL")
                self.apply_motion_speed_profile("FORWARD")
            elif turn_only:
                self.set_motor_mode("DUAL")
                self.apply_motion_speed_profile("TURN")
            else:
                self.set_motor_mode("IDLE")
                self.apply_motion_speed_profile("IDLE")

        now = time.monotonic()
        dx = abs(x - self.last_joystick_vector[0])
        dy = abs(y - self.last_joystick_vector[1])

        if dx < 0.02 and dy < 0.02 and (now - self.last_joystick_send_ts) < 0.12:
            return

        if self.serial_worker.send_line(f"J {x:.3f} {y:.3f}"):
            self.last_joystick_vector = (x, y)
            self.last_joystick_send_ts = now

    def send_custom(self):
        """Send custom command."""
        text = self.custom_edit.text().strip()
        if text:
            self.serial_worker.send_line(text)
            self.custom_edit.clear()

    def append_log(self, message: str):
        """Append message to the log console."""
        timestamp = time.strftime("%H:%M:%S")
        formatted_message = f"{timestamp} {message}"
        self.log_text.append(formatted_message)
        
        # Auto-scroll to bottom
        cursor = self.log_text.textCursor()
        cursor.movePosition(cursor.MoveOperation.End)
        self.log_text.setTextCursor(cursor)

    def update_current(self, current: float):
        """Update current display and graph."""
        self.line_graph.add_current_data(current)

    def update_voltage(self, voltage: float):
        """Update voltage display and graph."""
        if self.smoothed_voltage is None:
            smoothed_voltage = voltage
        else:
            smoothed_voltage = (
                VOLTAGE_SMOOTH_ALPHA * voltage
                + (1 - VOLTAGE_SMOOTH_ALPHA) * self.smoothed_voltage
            )
        self.smoothed_voltage = smoothed_voltage
        self.line_graph.add_voltage_data(smoothed_voltage)
        self.update_battery_from_voltage(smoothed_voltage)

    def on_speed_telemetry(self, speed: float):
        """Handle incoming telemetry speed unless overridden."""
        if self.speed_override_active and self.speed_override_value is not None:
            return
        self.update_speed(speed)

    def apply_motion_speed_profile(self, motion: str):
        """Map motion styles to deterministic speed overrides."""
        profile = (motion or "IDLE").upper()
        if profile in ("FORWARD", "BACKWARD"):
            self.set_speed_estimate_from_rpm(FULL_SPEED_RPM)
        elif profile == "DIAGONAL":
            self.set_speed_estimate_from_rpm(SINGLE_MOTOR_RPM)
        elif profile in ("LEFT", "RIGHT", "TURN"):
            self.set_speed_override(0.0)
        elif profile == "IDLE":
            self.set_speed_override(0.0)
        else:
            self.set_speed_override(None)

    def set_speed_override(self, speed: Optional[float]):
        """Apply or clear manual speed overrides."""
        if speed is None:
            self.speed_override_active = False
            self.speed_override_value = None
            if hasattr(self, "speed_label"):
                self.update_speed(0.0)
            return

        clamped_speed = max(0.0, speed)
        self.speed_override_active = True
        self.speed_override_value = clamped_speed
        if hasattr(self, "speed_label"):
            self.update_speed(clamped_speed)

    def set_speed_estimate_from_rpm(self, rpm: float):
        """Apply estimated speed based on wheel circumference and rpm."""
        rpm = max(0.0, rpm)
        cm_per_min = WHEEL_CIRCUMFERENCE_CM * rpm
        if cm_per_min > 0.0:
            cm_per_min = max(0.0, cm_per_min + random.uniform(-SPEED_VARIATION_CM, SPEED_VARIATION_CM))
        self.set_speed_override(cm_per_min)

    def update_speed(self, speed: float):
        """Update speed display (expects cm/min when using estimates)."""
        self.speed_label.setText(f"{speed:.1f}")
        
        max_speed = max(0.1, WHEEL_CIRCUMFERENCE_CM * FULL_SPEED_RPM)
        ratio = min(1.0, max(0.0, speed / max_speed))
        if ratio >= 0.8:
            color = "#ef4444"  # High speed
        elif ratio >= 0.4:
            color = "#f59e0b"  # Medium speed
        elif speed > 0.1:
            color = "#3b82f6"  # Normal movement
        else:
            color = "#6b7280"  # Idle
        
        self.speed_label.setStyleSheet(f"color: {color}; margin: 16px 0;")

    def update_obstacle_status(self, detected: bool):
        """Update obstacle detection status."""
        if detected:
            self.obstacle_label.setText("OBSTACLE DETECTED!")
            self.obstacle_label.setStyleSheet("color: #ef4444; margin: 16px 0; font-weight: bold;")  # Red
            self.obstacle_status_label.setText("Status: DETECTED")
            self.obstacle_status_label.setStyleSheet("color: #ef4444; font-size: 10px;")
            # When obstacle is detected, robot goes to IDLE
            self.update_motion_status("IDLE")
        else:
            self.obstacle_label.setText("No Obstacle Detected")
            self.obstacle_label.setStyleSheet("color: #10b981; margin: 16px 0;")  # Green
            self.obstacle_status_label.setText("Status: Clear")
            self.obstacle_status_label.setStyleSheet("color: #94a3b8; font-size: 10px;")
            # Don't automatically change motion when obstacle is cleared

    def update_motion_status(self, motion: str):
        """Update robot motion display."""
        motion = motion.upper()
        self.motion_label.setText(motion)
        
        # Color coding: Forward=Green, Backward=Red, Left=Yellow, Right=Blue, Idle=Gray
        if motion == "FORWARD":
            color = "#10b981"  # Green
            status = "Moving Forward"
        elif motion == "BACKWARD":
            color = "#ef4444"  # Red
            status = "Moving Backward"
        elif motion == "LEFT":
            color = "#f59e0b"  # Yellow/Orange
            status = "Turning Left"
        elif motion == "RIGHT":
            color = "#3b82f6"  # Blue
            status = "Turning Right"
        else:  # IDLE or any other state
            color = "#6b7280"  # Gray
            status = "Idle"
            motion = "IDLE"
        
        self.motion_label.setStyleSheet(f"color: {color}; margin: 20px 0; padding: 10px; border: 2px solid {color}; border-radius: 8px; font-weight: bold;")
        self.motion_status_label.setText(f"Status: {status}")
        self.motion_status_label.setStyleSheet(f"color: {color}; font-size: 10px;")
        manual_control = not self.joystick_active
        if manual_control:
            if motion == "IDLE":
                self.set_motor_mode("IDLE")
                self.apply_motion_speed_profile("IDLE")
            elif motion in ("FORWARD", "BACKWARD"):
                self.set_motor_mode("DUAL")
                self.apply_motion_speed_profile(motion)
            elif motion in ("LEFT", "RIGHT"):
                self.set_motor_mode("DUAL")
                self.apply_motion_speed_profile("TURN")
            else:
                self.set_motor_mode("DUAL")
                self.apply_motion_speed_profile(motion)

    def closeEvent(self, event):
        """Handle application close event."""
        self.current_timer.stop()
        self.serial_worker.close_serial()
        if self.joystick_worker:
            self.joystick_worker.stop()
            self.joystick_worker = None
        event.accept()


def main():
    app = QApplication(sys.argv)
    app.setApplicationName("ESP32 BLE Remote Control")
    app.setOrganizationName("ESP32 Projects")
    
    # Set application font
    font = QFont("Inter", 10)
    app.setFont(font)
    
    window = ESP32RemoteGUI()
    window.show()
    
    # Center window on screen
    screen = app.primaryScreen().availableGeometry()
    x = (screen.width() - window.width()) // 2
    y = (screen.height() - window.height()) // 2
    window.move(x, y)
    
    sys.exit(app.exec())


if __name__ == "__main__":
    main()

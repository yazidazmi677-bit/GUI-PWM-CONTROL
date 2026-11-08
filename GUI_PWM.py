import sys
import time
import math
import serial.tools.list_ports
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QSlider, QComboBox, QGroupBox,
    QGridLayout, QProgressBar, QTextEdit, QFrame, QMessageBox,
    QRadioButton, QButtonGroup
)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QThread, QRectF, QPointF
from PyQt6.QtGui import QPainter, QColor, QPen, QBrush, QFont

try:
    from pymavlink import mavutil
    MAVLINK_AVAILABLE = True
except ImportError:
    MAVLINK_AVAILABLE = False

try:
    import pygame
    PYGAME_AVAILABLE = True
except ImportError:
    PYGAME_AVAILABLE = False


# ==============================================================================
# WIDGET VISUALIZER GAMEPAD XBOX INTERAKTIF (REAL-TIME)
# ==============================================================================
class XboxVisualizerWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.setFixedSize(380, 210)
        self.lx, self.ly, self.rx, self.ry = 0.0, 0.0, 0.0, 0.0
        self.lt, self.rt = 0.0, 0.0
        self.buttons = [False] * 15

    def update_state(self, lx, ly, rx, ry, lt, rt, buttons):
        self.lx, self.ly, self.rx, self.ry = lx, ly, rx, ry
        self.lt, self.rt = lt, rt
        self.buttons = buttons
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        painter.fillRect(0, 0, self.width(), self.height(), QColor("#090d16"))
        painter.setPen(QPen(QColor("#1e293b"), 2))
        painter.drawRoundedRect(4, 4, self.width() - 8, self.height() - 8, 10, 10)

        # LB & RB
        lb = self.buttons[4] if len(self.buttons) > 4 else False
        rb = self.buttons[5] if len(self.buttons) > 5 else False
        painter.setBrush(QBrush(QColor("#38bdf8") if lb else QColor("#1e293b")))
        painter.drawRoundedRect(40, 20, 60, 16, 4, 4)
        painter.setPen(QColor("#f8fafc"))
        painter.setFont(QFont("Segoe UI", 8, QFont.Weight.Bold))
        painter.drawText(55, 32, "LB")

        painter.setBrush(QBrush(QColor("#38bdf8") if rb else QColor("#1e293b")))
        painter.drawRoundedRect(280, 20, 60, 16, 4, 4)
        painter.drawText(295, 32, "RB")

        # Left Thumbstick
        cx_l, cy_l = 90, 110
        painter.setPen(QPen(QColor("#334155"), 2))
        painter.setBrush(QBrush(QColor("#111827")))
        painter.drawEllipse(QPointF(cx_l, cy_l), 32, 32)
        knob_x_l = cx_l + int(self.lx * 18)
        knob_y_l = cy_l + int(self.ly * 18)
        painter.setBrush(QBrush(QColor("#0284c7")))
        painter.drawEllipse(QPointF(knob_x_l, knob_y_l), 16, 16)
        painter.setPen(QColor("#ffffff"))
        painter.drawText(cx_l - 22, cy_l + 45, "Left Stick\n(Throttle)")

        # Right Thumbstick
        cx_r, cy_r = 290, 110
        painter.setPen(QPen(QColor("#334155"), 2))
        painter.setBrush(QBrush(QColor("#111827")))
        painter.drawEllipse(QPointF(cx_r, cy_r), 32, 32)
        knob_x_r = cx_r + int(self.rx * 18)
        knob_y_r = cy_r + int(self.ry * 18)
        painter.setBrush(QBrush(QColor("#f59e0b")))
        painter.drawEllipse(QPointF(knob_x_r, knob_y_r), 16, 16)
        painter.setPen(QColor("#ffffff"))
        painter.drawText(cx_r - 24, cy_r + 45, "Right Stick\n(CW / CCW)")

        # Buttons ABXY (Y = Index 3)
        buttons_pos = {'X': (175, 110, 0), 'Y': (192, 94, 3), 'B': (210, 110, 1), 'A': (192, 128, 2)}
        for name, (bx, by, b_idx) in buttons_pos.items():
            pressed = self.buttons[b_idx] if len(self.buttons) > b_idx else False
            color = QColor("#00ff88") if pressed else QColor("#1e293b")
            painter.setPen(QPen(QColor("#475569"), 1))
            painter.setBrush(QBrush(color))
            painter.drawEllipse(bx, by, 18, 18)
            painter.setPen(QColor("#000000") if pressed else QColor("#f8fafc"))
            painter.drawText(bx + 5, by + 13, name)


# ==============================================================================
# WORKER THREAD MAVLINK (DENGAN HARDWARE CUT-OFF SAAT DISARM)
# ==============================================================================
class AUVWorkerThread(QThread):
    sig_status = pyqtSignal(bool, str)
    sig_log = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.master = None
        self.running = False
        self.port = ""
        self.baud = 115200
        self.target_system = 1
        self.target_component = 1
        self.pending_pwm = [1500] * 8
        self.is_armed = False

    def connect_and_unlock(self, port, baud):
        self.port = port
        self.baud = baud
        self.running = True
        self.start()

    def set_arm_state(self, arm: bool):
        self.is_armed = arm
        if not self.master or not self.running:
            return
        
        val = 1 if arm else 0
        try:
            # Kirim Force Arm (Magic 21196) atau Force Disarm (0)
            self.master.mav.command_long_send(
                self.target_system, self.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, val, 21196.0, 0, 0, 0, 0, 0
            )

            if not arm:
                # POTONG SINYAL AKTIF: Kirim 0 us (Kill Pulse) agar ESC benar-benar mati
                for ch in range(1, 9):
                    self.master.mav.command_long_send(
                        self.target_system, self.target_component,
                        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                        0, ch, 0, 0, 0, 0, 0, 0
                    )
                # Lepaskan RC override jika ada
                self.master.mav.rc_channels_override_send(
                    self.target_system, self.target_component,
                    0, 0, 0, 0, 0, 0, 0, 0
                )
        except Exception:
            pass

    def disconnect_fcu(self):
        self.running = False
        self.is_armed = False
        if self.master:
            try:
                # Matikan semua output ke 0 us sebelum tutup koneksi
                for ch in range(1, 9):
                    self.master.mav.command_long_send(
                        self.target_system, self.target_component,
                        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                        0, ch, 0, 0, 0, 0, 0, 0
                    )
                self.master.mav.command_long_send(
                    self.target_system, self.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0, 0, 0, 0, 0, 0, 0, 0
                )
                self.master.close()
            except Exception:
                pass
            self.master = None
        self.sig_status.emit(False, "Disconnected")

    def run(self):
        if not MAVLINK_AVAILABLE:
            self.sig_status.emit(False, "Library pymavlink belum terinstal!")
            return

        self.sig_log.emit(f"Membuka port serial AUV {self.port} @ {self.baud}...")
        try:
            self.master = mavutil.mavlink_connection(self.port, baud=self.baud, autoreconnect=True)
            self.sig_log.emit("Menunggu Heartbeat AUV / Pixhawk...")
            hb = self.master.wait_heartbeat(timeout=6.0)
            if hb is None:
                self.sig_status.emit(False, "Heartbeat Timeout! Cek port COM.")
                self.disconnect_fcu()
                return

            self.target_system = self.master.target_system
            self.target_component = self.master.target_component
            self.sig_status.emit(True, f"Connected [SysID: {self.target_system}]")
            self.sig_log.emit(">> Berhasil terhubung ke sistem AUV (Status Awal: DISARMED / CUT-OFF).")

            # Bypass parameter proteksi ArduPilot[cite: 1]
            self.master.mav.param_set_send(self.target_system, self.target_component, b'BRD_SAFETY_DEFLT', 0.0, mavutil.mavlink.MAV_PARAM_TYPE_INT8)
            time.sleep(0.04)
            self.master.mav.param_set_send(self.target_system, self.target_component, b'BRD_SAFETY_MASK', 65535.0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
            time.sleep(0.04)
            self.master.mav.param_set_send(self.target_system, self.target_component, b'ARMING_CHECK', 0.0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
            time.sleep(0.04)

            for ch in range(1, 9):
                self.master.mav.param_set_send(self.target_system, self.target_component, f"SERVO{ch}_FUNCTION".encode('utf-8'), 0.0, mavutil.mavlink.MAV_PARAM_TYPE_INT16)
                time.sleep(0.02)

            # Awal start: Matikan sinyal ke 0 us (no pulse)
            for ch in range(1, 9):
                self.master.mav.command_long_send(
                    self.target_system, self.target_component,
                    mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                    0, ch, 0, 0, 0, 0, 0, 0
                )

            self.sig_log.emit(">> Proteksi dilucuti. Motor dalam kondisi MATI (Cut-off). Tekan 'Y' atau tombol ARM untuk mulai.")

            while self.running:
                if self.is_armed:
                    # Hanya pancarkan PWM aktif jika status sudah ARMED
                    for idx, pwm in enumerate(self.pending_pwm):
                        ch = idx + 1
                        self.master.mav.command_long_send(
                            self.target_system, self.target_component,
                            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                            0, ch, pwm, 0, 0, 0, 0, 0
                        )
                time.sleep(0.05)

        except Exception as e:
            self.sig_status.emit(False, f"Error: {e}")
            self.running = False

    def update_pwm(self, channel, pwm_value):
        if 1 <= channel <= 8:
            self.pending_pwm[channel - 1] = pwm_value


class AUVChannelCard(QFrame):
    sig_changed = pyqtSignal(int, int)

    def __init__(self, ch_num):
        super().__init__()
        self.ch_num = ch_num
        self.setStyleSheet("background-color: #121824; border: 1px solid #1e293b; border-radius: 6px;")

        lay = QVBoxLayout(self)
        lay.setContentsMargins(10, 8, 10, 8)
        lay.setSpacing(6)

        header = QHBoxLayout()
        self.lbl_title = QLabel(f"MAIN OUT {ch_num}")
        self.lbl_title.setStyleSheet("color: #38bdf8; font-weight: bold; font-size: 11px; border: none;")
        self.lbl_val = QLabel("1500 µs (Stop)")
        self.lbl_val.setStyleSheet("color: #00ff88; font-family: monospace; font-weight: bold; border: none;")
        header.addWidget(self.lbl_title)
        header.addStretch()
        header.addWidget(self.lbl_val)
        lay.addLayout(header)

        self.pbar = QProgressBar()
        self.pbar.setRange(1000, 2000)
        self.pbar.setValue(1500)
        self.pbar.setTextVisible(False)
        self.pbar.setFixedHeight(5)
        self.pbar.setStyleSheet("QProgressBar { background: #070a0f; border: none; border-radius: 2px; } QProgressBar::chunk { background: #38bdf8; border-radius: 2px; }")
        lay.addWidget(self.pbar)

        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(1000, 2000)
        self.slider.setValue(1500)
        self.slider.setEnabled(False)
        self.slider.valueChanged.connect(self.on_slide)
        lay.addWidget(self.slider)

    def on_slide(self, val):
        if val == 1500:
            txt, color = "1500 µs (Stop)", "#00ff88"
        elif val > 1500:
            txt, color = f"{val} µs (Maju / CW)", "#38bdf8"
        else:
            txt, color = f"{val} µs (Mundur / CCW)", "#f59e0b"

        self.lbl_val.setText(txt)
        self.lbl_val.setStyleSheet(f"color: {color}; font-family: monospace; font-weight: bold; border: none;")
        self.pbar.setValue(val)
        self.sig_changed.emit(self.ch_num, val)

    def set_active(self, active: bool):
        self.slider.setEnabled(active)
        if not active:
            self.slider.setValue(1500)


# ==============================================================================
# MAIN WINDOW UTAMA
# ==============================================================================
class AUVProMainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("AUV Pixhawk 6C - Xbox Controller Suite [Zero Cut-Off Fixed]")
        self.resize(1120, 830)
        self.setStyleSheet("background-color: #070a0f; color: #f1f5f9; font-family: 'Segoe UI', sans-serif;")

        self.worker = AUVWorkerThread()
        self.worker.sig_status.connect(self.on_connection_changed)
        self.worker.sig_log.connect(self.log)

        self.is_armed = False
        self.last_y_button_state = False
        self.active_main_channel = 1

        self.joystick = None
        self.init_xbox_controller()

        self.joy_timer = QTimer()
        self.joy_timer.setInterval(33)
        self.joy_timer.timeout.connect(self.poll_xbox_input)

        self.init_ui()
        self.scan_ports()

    def init_xbox_controller(self):
        if not PYGAME_AVAILABLE:
            return
        try:
            pygame.quit()
            pygame.init()
            pygame.joystick.init()
            if pygame.joystick.get_count() > 0:
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
        except Exception:
            pass

    def refresh_joystick(self):
        self.init_xbox_controller()
        if self.joystick:
            name = self.joystick.get_name()
            self.lbl_joy_info.setText(f"🎮 <b>Gamepad:</b> <span style='color:#00ff88;'>TERHUBUNG ({name})</span>")
            self.log(f"Gamepad berhasil direfresh: {name}")
        else:
            self.lbl_joy_info.setText("🎮 <b>Gamepad:</b> <span style='color:#ef4444;'>TIDAK TERDETEKSI</span>")
            self.log("Gamepad tidak ditemukan.")

    def init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_lay = QVBoxLayout(central)
        main_lay.setContentsMargins(14, 14, 14, 14)
        main_lay.setSpacing(10)

        # 1. Toolbar Atas
        top_frame = QFrame()
        top_frame.setStyleSheet("background: #0f172a; border: 1px solid #1e293b; border-radius: 6px;")
        top_lay = QHBoxLayout(top_frame)
        top_lay.setContentsMargins(10, 8, 10, 8)

        top_lay.addWidget(QLabel("<b>Port:</b>"))
        self.combo_port = QComboBox()
        self.combo_port.setFixedWidth(150)
        top_lay.addWidget(self.combo_port)

        btn_scan = QPushButton("🔄")
        btn_scan.setFixedWidth(32)
        btn_scan.clicked.connect(self.scan_ports)
        top_lay.addWidget(btn_scan)

        top_lay.addWidget(QLabel("<b>Baud:</b>"))
        self.combo_baud = QComboBox()
        self.combo_baud.addItems(["115200", "57600", "921600"])
        top_lay.addWidget(self.combo_baud)

        self.btn_conn = QPushButton("CONNECT AUV")
        self.btn_conn.setFixedHeight(32)
        self.btn_conn.setStyleSheet("background: #0284c7; font-weight: bold; padding: 0 14px; border-radius: 4px;")
        self.btn_conn.clicked.connect(self.toggle_conn)
        top_lay.addWidget(self.btn_conn)

        self.btn_arm = QPushButton("🔒 DISARMED (CUT-OFF)")
        self.btn_arm.setFixedHeight(32)
        self.btn_arm.setEnabled(False)
        self.btn_arm.setStyleSheet("QPushButton { background-color: #78350f; color: #fde047; font-weight: bold; padding: 0 14px; border-radius: 4px; border: 1px solid #b45309; }")
        self.btn_arm.clicked.connect(self.toggle_arm_state)
        top_lay.addWidget(self.btn_arm)

        self.lbl_status = QLabel("OFFLINE")
        self.lbl_status.setStyleSheet("color: #ef4444; font-weight: bold; margin-left: 6px;")
        top_lay.addWidget(self.lbl_status)

        top_lay.addStretch()

        self.btn_stop = QPushButton("🛑 EMERGENCY STOP (KILL)")
        self.btn_stop.setFixedHeight(32)
        self.btn_stop.setStyleSheet("background: #dc2626; color: white; font-weight: bold; padding: 0 12px; border-radius: 4px;")
        self.btn_stop.clicked.connect(self.emergency_stop)
        top_lay.addWidget(self.btn_stop)

        main_lay.addWidget(top_frame)

        # 1.5 Panel Radio Button Selector MAIN OUT
        selector_frame = QFrame()
        selector_frame.setStyleSheet("background: #0b132b; border: 1px solid #1e3a8a; border-radius: 6px;")
        selector_lay = QHBoxLayout(selector_frame)
        selector_lay.setContentsMargins(12, 6, 12, 6)
        
        selector_lay.addWidget(QLabel("🎯 <b>Target Kontrol Gamepad:</b>"))
        
        self.radio_group = QButtonGroup(self)
        for i in range(1, 9):
            rb = QRadioButton(f"MAIN {i}")
            rb.setStyleSheet("color: #93c5fd; font-weight: bold; border: none; margin-left: 6px;")
            if i == 1:
                rb.setChecked(True)
            self.radio_group.addButton(rb, i)
            selector_lay.addWidget(rb)
        
        self.radio_group.idClicked.connect(self.on_main_out_selected)
        selector_lay.addStretch()
        main_lay.addWidget(selector_frame)

        # 2. Panel Tengah: Grid Channel (Kiri) & Visualizer Gamepad (Kanan)
        middle_layout = QHBoxLayout()
        middle_layout.setSpacing(10)

        grid_grp = QGroupBox("AUV Thrusters (MAIN 1 - 8) [Stop: 1500 µs | Disarmed: No Signal]")
        grid_grp.setStyleSheet("QGroupBox { border: 1px solid #1e293b; border-radius: 6px; font-weight: bold; padding-top: 18px; color: #38bdf8; background: #0f172a; }")
        grid_lay = QGridLayout(grid_grp)
        grid_lay.setSpacing(8)

        self.cards = []
        for i in range(8):
            card = AUVChannelCard(i + 1)
            card.sig_changed.connect(self.on_pwm_change)
            self.cards.append(card)
            grid_lay.addWidget(card, i // 2, i % 2)

        middle_layout.addWidget(grid_grp, stretch=3)

        right_panel = QWidget()
        right_lay = QVBoxLayout(right_panel)
        right_lay.setContentsMargins(0, 0, 0, 0)
        right_lay.setSpacing(8)

        vis_grp = QGroupBox("Gamepad Live Visualizer")
        vis_grp.setStyleSheet("QGroupBox { border: 1px solid #1e293b; border-radius: 6px; font-weight: bold; padding-top: 18px; color: #38bdf8; background: #0f172a; }")
        vis_lay = QVBoxLayout(vis_grp)
        vis_lay.setContentsMargins(8, 8, 8, 8)

        self.xbox_vis = XboxVisualizerWidget()
        vis_lay.addWidget(self.xbox_vis, alignment=Qt.AlignmentFlag.AlignCenter)

        lbl_info_map = QLabel(
            "<b>Mapping Kontrol Gamepad:</b><br>"
            "• <b>Left Stick (Y):</b> Throttle Target Terpilih (Maju/Mundur)<br>"
            "• <b>Right Stick (X):</b> Rotasi Belok (Kanan=<b>CW</b>, Kiri=<b>CCW</b>)<br>"
            "• <b>Tombol Y:</b> ARM / DISARM Cepat (Wajib ARM agar PWM aktif)<br>"
            "• <b>Deadzone:</b> Otomatis kunci presisi 1500 µs saat stik dilepas"
        )
        lbl_info_map.setStyleSheet("color: #94a3b8; font-size: 11px; border: none; background: #090d16; padding: 8px; border-radius: 4px;")
        vis_lay.addWidget(lbl_info_map)

        status_bar_joy = QHBoxLayout()
        init_name = self.joystick.get_name() if self.joystick else "Tidak Ada"
        init_color = "#00ff88" if self.joystick else "#ef4444"
        init_txt = f"TERHUBUNG ({init_name})" if self.joystick else "TIDAK TERDETEKSI"

        self.lbl_joy_info = QLabel(f"🎮 <b>Status:</b> <span style='color:{init_color};'>{init_txt}</span>")
        self.lbl_joy_info.setStyleSheet("border: none; font-size: 11px;")
        status_bar_joy.addWidget(self.lbl_joy_info)

        btn_ref_joy = QPushButton("🔄 Refresh")
        btn_ref_joy.setStyleSheet("background: #1e40af; color: white; font-size: 10px; padding: 3px 8px; border-radius: 4px;")
        btn_ref_joy.clicked.connect(self.refresh_joystick)
        status_bar_joy.addWidget(btn_ref_joy)

        vis_lay.addLayout(status_bar_joy)
        right_lay.addWidget(vis_grp)
        right_lay.addStretch()

        middle_layout.addWidget(right_panel, stretch=2)
        main_lay.addLayout(middle_layout)

        # 3. Console Log Bawah
        grp_log = QGroupBox("Console Telemetry & Status")
        grp_log.setStyleSheet(grid_grp.styleSheet())
        log_lay = QVBoxLayout(grp_log)
        log_lay.setContentsMargins(8, 12, 8, 8)
        self.console = QTextEdit()
        self.console.setReadOnly(True)
        self.console.setStyleSheet("background: #04060a; color: #34d399; font-family: monospace; font-size: 11px; border: 1px solid #1e293b; border-radius: 4px;")
        log_lay.addWidget(self.console)
        main_lay.addWidget(grp_log, stretch=1)

    def scan_ports(self):
        self.combo_port.clear()
        for p in serial.tools.list_ports.comports():
            self.combo_port.addItem(f"{p.device} ({p.description[:15]})", p.device)
        if self.combo_port.count() == 0:
            self.combo_port.addItem("COM3", "COM3")

    def log(self, text):
        t = time.strftime("%H:%M:%S")
        self.console.append(f"[{t}] {text}")

    def on_main_out_selected(self, ch_id):
        self.active_main_channel = ch_id
        self.log(f"🎯 Target kontrol Left Stick Gamepad dialihkan ke: MAIN OUT {ch_id}")

    def toggle_conn(self):
        if not self.worker.running:
            port = self.combo_port.currentData() or self.combo_port.currentText().split(" ")[0]
            baud = int(self.combo_baud.currentText())
            self.worker.connect_and_unlock(port, baud)
            self.btn_conn.setText("CONNECTING...")
            self.btn_conn.setEnabled(False)
        else:
            self.worker.disconnect_fcu()

    def on_connection_changed(self, connected, msg):
        self.btn_conn.setEnabled(True)
        if connected:
            self.lbl_status.setText("ONLINE")
            self.lbl_status.setStyleSheet("color: #10b981; font-weight: bold;")
            self.btn_conn.setText("DISCONNECT")
            self.btn_arm.setEnabled(True)
            for c in self.cards:
                c.set_active(True)
            if self.joystick:
                self.joy_timer.start()
        else:
            self.lbl_status.setText("OFFLINE")
            self.lbl_status.setStyleSheet("color: #ef4444; font-weight: bold;")
            self.btn_conn.setText("CONNECT AUV")
            self.btn_arm.setEnabled(False)
            self.set_arm_ui_state(False)
            for c in self.cards:
                c.set_active(False)
            self.joy_timer.stop()
        self.log(msg)

    def toggle_arm_state(self):
        self.is_armed = not self.is_armed
        self.worker.set_arm_state(self.is_armed)
        self.set_arm_ui_state(self.is_armed)

    def set_arm_ui_state(self, armed: bool):
        self.is_armed = armed
        if armed:
            self.btn_arm.setText("⚡ ARMED (LIVE)")
            self.btn_arm.setStyleSheet("QPushButton { background-color: #dc2626; color: white; font-weight: bold; padding: 0 14px; border-radius: 4px; border: 1px solid #f87171; }")
            self.log("⚡ ARMED: Output PWM aktif. Thruster siap menerima sinyal putar.")
        else:
            self.btn_arm.setText("🔒 DISARMED (CUT-OFF)")
            self.btn_arm.setStyleSheet("QPushButton { background-color: #78350f; color: #fde047; font-weight: bold; padding: 0 14px; border-radius: 4px; border: 1px solid #b45309; }")
            self.log("🔒 DISARMED: Sinyal PWM dimatikan (0 µs). Thruster dipaksa OFF total.")

    def on_pwm_change(self, ch, val):
        self.worker.update_pwm(ch, val)

    def poll_xbox_input(self):
        if not self.joystick:
            return
        try:
            pygame.event.pump()
            
            lx = self.joystick.get_axis(0)
            ly = self.joystick.get_axis(1)
            rx = self.joystick.get_axis(3)
            ry = self.joystick.get_axis(4) if self.joystick.get_numaxes() > 4 else self.joystick.get_axis(2)
            
            lt = (self.joystick.get_axis(2) + 1.0) / 2.0 if self.joystick.get_numaxes() > 2 else 0.0
            rt = (self.joystick.get_axis(5) + 1.0) / 2.0 if self.joystick.get_numaxes() > 5 else 0.0

            num_buttons = self.joystick.get_numbuttons()
            buttons = [self.joystick.get_button(b) for b in range(min(15, num_buttons))]

            self.xbox_vis.update_state(lx, ly, rx, ry, lt, rt, buttons)

            # Tombol Y untuk toggle ARM / DISARM
            current_y_state = buttons[3] if len(buttons) > 3 else False
            if current_y_state and not self.last_y_button_state:
                if self.worker.running:
                    self.toggle_arm_state()
            self.last_y_button_state = current_y_state

            # DEADZONE FILTER DIPERLEBAR: Mencegah drift potong 1500 us
            if abs(ly) < 0.12:
                ly = 0.0
            if abs(rx) < 0.12:
                rx = 0.0

            # Hitung PWM Target
            pwm_target = int(1500 - (ly * 500))
            pwm_target = max(1000, min(2000, pwm_target))

            # Target channel yang dipilih via Radio Button
            target_idx = self.active_main_channel - 1
            if 0 <= target_idx < len(self.cards) and self.cards[target_idx].slider.isEnabled():
                self.cards[target_idx].slider.setValue(pwm_target)

            # Rotasi belok CW / CCW pada channel sekunder (MAIN 2)
            if len(self.cards) > 1 and self.cards[1].slider.isEnabled():
                pwm_rot = int(1500 + (rx * 500))
                pwm_rot = max(1000, min(2000, pwm_rot))
                self.cards[1].slider.setValue(pwm_rot)

        except Exception:
            pass

    def emergency_stop(self):
        # Paksa DISARM & potong sinyal ke 0 us
        self.is_armed = False
        self.worker.set_arm_state(False)
        self.set_arm_ui_state(False)
        for c in self.cards:
            c.slider.setValue(1500)
        self.log("🛑 EMERGENCY STOP: Sinyal PWM dipotong ke 0 µs dan sistem diset ke DISARM.")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = AUVProMainWindow()
    window.show()
    sys.exit(app.exec())
import sys
import time
import json
import serial
import pyqtgraph.opengl as gl
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QLabel, QPushButton, QMessageBox
from PyQt5.QtCore import QTimer
from threading import Thread, Lock
from NatNetClient import NatNetClient
import numpy as np


class RealTimeDataAnalyzer3D(QMainWindow):
    def __init__(self):
        super().__init__()

        # -------------------------------
        # UI Setup
        # -------------------------------
        self.setWindowTitle("Real-Time Rigid Body 3D Position and Rotation")
        self.setGeometry(100, 100, 1200, 800)

        # Pause state for commutator control
        self.is_paused = False

        # -------------------------------
        # Serial Communication Config
        # -------------------------------
        self.SERIAL_PORT = "COM6"
        self.BAUDRATE = 9600
        self.DEGREES_PER_TURN = 360

        try:
            self.ser = serial.Serial(self.SERIAL_PORT, self.BAUDRATE, timeout=1)
            time.sleep(2)
        except serial.SerialException as e:
            self.ser = None
            QMessageBox.critical(self, "Serial Error", f"{e}")

        # -------------------------------
        # Main Layout
        # -------------------------------
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QVBoxLayout()
        self.central_widget.setLayout(self.layout)

        # -------------------------------
        # 3D Visualization Setup
        # -------------------------------
        self.plot_widget = gl.GLViewWidget()
        self.plot_widget.setFixedSize(800, 600)
        self.layout.addWidget(self.plot_widget)
        self.plot_widget.setCameraPosition(distance=10)

        grid = gl.GLGridItem()
        grid.scale(0.1, 0.1, 0.1)
        self.plot_widget.addItem(grid)

        self.traj_line = gl.GLLinePlotItem()
        self.plot_widget.addItem(self.traj_line)

        # -------------------------------
        # UI Labels
        # -------------------------------
        self.position_label = QLabel("Position: X=0.00, Y=0.00, Z=0.00")
        self.layout.addWidget(self.position_label)

        self.rotation_label = QLabel("Accumulated Rotation: 0.00 deg")
        self.layout.addWidget(self.rotation_label)

        self.status_label = QLabel("Status: Running")
        self.layout.addWidget(self.status_label)

        self.pause_button = QPushButton("Pause Commutator")
        self.pause_button.clicked.connect(self.toggle_pause_resume)
        self.layout.addWidget(self.pause_button)

        # -------------------------------
        # Data Buffers
        # -------------------------------
        self.positions = []
        self.data_lock = Lock()

        # Quaternion tracking
        self.q_prev = None

        # Accumulated rotation (core logic from second code)
        self.accum_angle = 0.0

        # Control parameters
        self.ANGLE_THRESHOLD = 90.0
        self.MAX_VALID_DELTA = 40.0
        self.GAIN = 1.0

        # -------------------------------
        # Timer for UI updates
        # -------------------------------
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_chart)
        self.timer.start(100)

        # -------------------------------
        # NatNet streaming thread
        # -------------------------------
        self.streaming_client = NatNetClient()
        self.streaming_client.rigid_body_listener = self.receive_rigid_body_frame
        self.client_thread = Thread(target=self.streaming_client.run)
        self.client_thread.start()

    # -------------------------------
    # Quaternion math utilities
    # -------------------------------
    def quat_mul(self, a, b):
        """Multiply two quaternions"""
        w1, x1, y1, z1 = a
        w2, x2, y2, z2 = b
        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        ])

    def quat_conj(self, q):
        """Compute quaternion conjugate"""
        w, x, y, z = q
        return np.array([w, -x, -y, -z])

    def ensure_continuity(self, q, q_prev):
        """Avoid quaternion sign flip (q and -q represent same rotation)"""
        if q_prev is not None and np.dot(q, q_prev) < 0:
            q = -q
        return q

    def delta_yaw(self, q_now, q_prev):
        """Compute incremental rotation projected onto Y-axis (yaw-like)"""
        dq = self.quat_mul(q_now, self.quat_conj(q_prev))

        w, x, y, z = dq
        v = np.array([x, y, z])
        norm_v = np.linalg.norm(v)

        if norm_v < 1e-12:
            return 0.0

        axis = v / norm_v
        angle = 2.0 * np.degrees(np.arctan2(norm_v, w))

        # Project rotation axis onto Y axis
        up = np.array([0, 1, 0])
        sign = np.sign(np.dot(axis, up))

        return angle * (1.0 if sign >= 0 else -1.0)

    # -------------------------------
    # Send JSON command to commutator
    # -------------------------------
    def send_rotation_command(self, angle):
        """Send rotation command in JSON format"""
        if self.ser is None or not self.ser.is_open:
            return

        turns = angle / self.DEGREES_PER_TURN

        json_state = {
            "enable": True,
            "led": True,
            "turn": turns,
            "print": True
        }

        try:
            self.ser.write((json.dumps(json_state) + "\n").encode("utf-8"))
        except Exception as e:
            print(f"Serial Error: {e}")

    # -------------------------------
    # Pause / Resume control
    # -------------------------------
    def toggle_pause_resume(self):
        """Toggle commutator state"""
        self.is_paused = not self.is_paused

        if self.is_paused:
            self.status_label.setText("Status: Paused")
            self.pause_button.setText("Resume")
        else:
            self.status_label.setText("Status: Running")
            self.pause_button.setText("Pause")

    # -------------------------------
    # NatNet callback (core logic)
    # -------------------------------
    def receive_rigid_body_frame(self, rigid_body_id, position, orientation):
        """Handle incoming rigid body data"""

        if self.is_paused:
            return

        # Convert NatNet quaternion (x,y,z,w) → (w,x,y,z)
        ox, oy, oz, ow = orientation
        q = np.array([ow, ox, oy, oz])
        q = q / np.linalg.norm(q)

        # Ensure continuity to avoid sudden flips
        q = self.ensure_continuity(q, self.q_prev)

        if self.q_prev is None:
            self.q_prev = q
            return

        # Compute incremental yaw rotation
        delta = self.delta_yaw(q, self.q_prev) * self.GAIN
        self.q_prev = q

        # Reject abnormal spikes
        if abs(delta) > self.MAX_VALID_DELTA:
            return

        # Accumulate rotation
        self.accum_angle += delta
        self.accum_angle = float(np.clip(self.accum_angle, -360, 360))

        # Trigger compensation only when threshold reached
        if abs(self.accum_angle) >= self.ANGLE_THRESHOLD:
            self.send_rotation_command(self.accum_angle)

            # Remove only threshold portion (keep residual)
            self.accum_angle -= np.sign(self.accum_angle) * self.ANGLE_THRESHOLD

        # Save position for visualization
        with self.data_lock:
            self.positions.append(position)
            if len(self.positions) > 100:
                self.positions.pop(0)

        # Update UI text (safe enough at low frequency)
        self.position_label.setText(
            f"Position: X={position[0]:.2f}, Y={position[1]:.2f}, Z={position[2]:.2f}"
        )

        self.rotation_label.setText(
            f"Accumulated Rotation: {self.accum_angle:.2f} deg"
        )

    # -------------------------------
    # Update 3D trajectory plot
    # -------------------------------
    def update_chart(self):
        """Update trajectory visualization"""

        with self.data_lock:
            if not self.positions:
                return
            data = list(self.positions)

        points = np.array(data)
        self.traj_line.setData(pos=points, color=(1, 0, 0, 1), width=2)

    # -------------------------------
    # Cleanup on exit
    # -------------------------------
    def closeEvent(self, event):
        """Handle application close"""

        if self.ser and self.ser.is_open:
            self.ser.close()

        self.streaming_client.shutdown()
        self.client_thread.join()

        event.accept()


# -------------------------------
# Entry point
# -------------------------------
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = RealTimeDataAnalyzer3D()
    window.show()
    sys.exit(app.exec_())
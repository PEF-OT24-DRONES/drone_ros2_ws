import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, CompressedImage
from mavros_msgs.msg import State  # Importa el mensaje State de MAVROS
import cv2
import sys
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QLabel, QVBoxLayout, QTabWidget, QWidget, QProgressBar, QPushButton
)
from PyQt6.QtCore import QTimer, QUrl
from PyQt6.QtGui import QImage, QPixmap
from PyQt6.QtWebEngineWidgets import QWebEngineView
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np


class ChargingStationInterface(Node):
    def __init__(self):
        super().__init__('charging_station_interface')

        # QoS compatible con MAVROS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Suscripción al tópico de batería de MAVROS
        self.subscription_battery = self.create_subscription(
            BatteryState, '/mavros/battery', self.battery_callback, qos_profile
        )

        # Suscripción al tópico de estado del dron
        self.subscription_state = self.create_subscription(
            State, '/mavros/state', self.state_callback, qos_profile
        )

        # Suscripción al tópico de imagen comprimida
        self.subscription_camera = self.create_subscription(
            CompressedImage, '/aruco_detection/compressed', self.camera_callback, qos_profile
        )

        # Variables para almacenar los datos del dron
        self.is_charging = False
        self.battery_percentage = 0.0
        self.drone_connected = False
        self.drone_armed = False
        self.drone_guided = False
        self.manual_input = False
        self.flight_mode = "UNKNOWN"
        self.latest_frame = None  # Almacena el último frame recibido

    def battery_callback(self, msg):
        """Callback para recibir datos de la batería."""
        self.battery_percentage = msg.percentage * 100.0  # Convertir a porcentaje

    def state_callback(self, msg):
        """Callback para recibir el estado del dron."""
        self.drone_connected = msg.connected
        self.drone_armed = msg.armed
        self.drone_guided = msg.guided
        self.manual_input = msg.manual_input
        self.flight_mode = msg.mode

    def camera_callback(self, msg):
        """Callback para recibir y procesar las imágenes comprimidas."""
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.latest_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)


class MainWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node

        self.setWindowTitle("Charging Station Interface")
        self.setGeometry(200, 200, 800, 600)

        # Crear el widget de pestañas
        tabs = QTabWidget()

        # Estilo general para las pestañas y widgets
        tabs.setStyleSheet("""
            QTabWidget::pane {border: 2px solid #EEEEEE; border-radius: 5px; padding: 5px;}
            QTabBar::tab {background: #DDDDDD; color: #333333; border: 2px solid #EEEEEE; border-radius: 5px; padding: 5px;}
            QTabBar::tab:selected {background: #EEEEEE; color: #333333;}
        """)

        titles = "font-size: 24px; font-weight: bold; color: #DDDDDD;"

        # Pestaña de Información de la Estación de Carga
        station_tab = QWidget()
        station_layout = QVBoxLayout()
        station_title = QLabel("Charging Station Information")
        station_title.setStyleSheet(titles)
        station_layout.addWidget(station_title)

        self.station_status = QLabel("Status: Available")
        self.station_status.setStyleSheet("font-size: 18px; font-weight: bold; color: green; padding: 10px;")
        station_layout.addWidget(self.station_status)

        self.station_capacity = QLabel("Capacity: 1 drone per station")
        self.station_capacity.setStyleSheet("font-size: 18px; padding: 10px;")
        station_layout.addWidget(self.station_capacity)
        station_tab.setLayout(station_layout)
        tabs.addTab(station_tab, "Charging Station")

        # Pestaña de Información del Dron
        drone_tab = QWidget()
        drone_layout = QVBoxLayout()
        drone_title = QLabel("Drone Information")
        drone_title.setStyleSheet(titles)
        drone_layout.addWidget(drone_title)

        # Barra de progreso de la batería del dron
        self.battery_progress = QProgressBar()
        self.battery_progress.setValue(0)
        self.battery_progress.setFormat("Battery: %p%")
        self.battery_progress.setStyleSheet("""
            QProgressBar {border: 2px solid grey; border-radius: 5px; text-align: center;}
            QProgressBar::chunk {background-color: green; border-radius: 5px;}
        """)
        drone_layout.addWidget(self.battery_progress)

        # Estado de conexión del dron
        self.drone_connection_status = QLabel("Connection: Not connected")
        self.drone_connection_status.setStyleSheet("font-size: 18px; padding: 10px;")
        drone_layout.addWidget(self.drone_connection_status)

        # Estado armado
        self.drone_armed_status = QLabel("Armed: No")
        self.drone_armed_status.setStyleSheet("font-size: 18px; padding: 10px;")
        drone_layout.addWidget(self.drone_armed_status)

        # Modo de vuelo
        self.flight_mode_status = QLabel("Flight Mode: UNKNOWN")
        self.flight_mode_status.setStyleSheet("font-size: 18px; padding: 10px;")
        drone_layout.addWidget(self.flight_mode_status)

        # Control manual
        self.manual_input_status = QLabel("Manual Input: No")
        self.manual_input_status.setStyleSheet("font-size: 18px; padding: 10px;")
        drone_layout.addWidget(self.manual_input_status)

        # Video en tiempo real de la cámara
        self.camera_label = QLabel(self)
        self.camera_label.setFixedSize(640, 360)
        self.camera_label.setStyleSheet("border: 2px solid #EEEEEE;")
        drone_layout.addWidget(self.camera_label)

        drone_tab.setLayout(drone_layout)
        tabs.addTab(drone_tab, "Drone")

        # Pestaña de Mapa
        map_tab = QWidget()
        map_layout = QVBoxLayout()
        self.map_view = QWebEngineView()
        map_file_path = os.path.expanduser("~/drone_ros2_ws/src/charging_station/charging_station/map.html")
        #self.map_view.setUrl(QUrl.fromLocalFile(os.path.abspath(map_file_path)))
        #map_layout.addWidget(self.map_view)
        #map_tab.setLayout(map_layout)
        tabs.addTab(map_tab, "Map")

        # Pestaña de Integración con Otras Estaciones
        integration_tab = QWidget()
        integration_layout = QVBoxLayout()
        integration_title = QLabel("Integrated Stations")
        integration_title.setStyleSheet(titles)
        integration_layout.addWidget(integration_title)

        self.stations_summary = QLabel("Summary: 1 Station Available")
        self.stations_summary.setStyleSheet("font-size: 18px; padding: 10px;")
        integration_layout.addWidget(self.stations_summary)
        integration_tab.setLayout(integration_layout)
        tabs.addTab(integration_tab, "Integration")

        # Pestaña de Control Remoto
        control_tab = QWidget()
        control_layout = QVBoxLayout()
        control_title = QLabel("Remote Control")
        control_title.setStyleSheet(titles)
        control_layout.addWidget(control_title)

        self.start_charge_button = QPushButton("Start Charging")
        self.start_charge_button.setStyleSheet("""
            QPushButton {
                font-size: 16px; font-weight: bold; color: white; background-color: green; border-radius: 10px; padding: 10px;
            }
            QPushButton:hover {background-color: #228B22;}
        """)

        self.stop_charge_button = QPushButton("Stop Charging")
        self.stop_charge_button.setStyleSheet("""
            QPushButton {
                font-size: 16px; font-weight: bold; color: white; background-color: red; border-radius: 10px; padding: 10px;
            }
            QPushButton:hover {background-color: #8B0000;}
        """)

        self.start_charge_button.clicked.connect(self.start_charging)
        self.stop_charge_button.clicked.connect(self.stop_charging)
        control_layout.addWidget(self.start_charge_button)
        control_layout.addWidget(self.stop_charge_button)
        control_tab.setLayout(control_layout)
        tabs.addTab(control_tab, "Remote Control")

        # Establecer el widget de pestañas como el central
        self.setCentralWidget(tabs)

        # Timer para actualizar la interfaz
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_interface)
        self.timer.start(100)

        # Inicializar cámara
        self.cap = cv2.VideoCapture(0)

    def update_interface(self):
        """Actualiza los elementos de la interfaz en tiempo real."""
        # Actualiza el progreso de la batería
        battery_level = self.node.battery_percentage
        self.battery_progress.setValue(int(battery_level))

        if battery_level > 50:
            self.battery_progress.setStyleSheet("QProgressBar::chunk {background-color: green;}")
        elif 20 <= battery_level <= 50:
            self.battery_progress.setStyleSheet("QProgressBar::chunk {background-color: orange;}")
        else:
            self.battery_progress.setStyleSheet("QProgressBar::chunk {background-color: red;}")

        # Actualiza el estado de conexión del dron
        self.drone_connection_status.setText(f"Connection: {'Yes' if self.node.drone_connected else 'No'}")
        self.drone_armed_status.setText(f"Armed: {'Yes' if self.node.drone_armed else 'No'}")
        self.flight_mode_status.setText(f"Flight Mode: {self.node.flight_mode}")
        self.manual_input_status.setText(f"Manual Input: {'Yes' if self.node.manual_input else 'No'}")

        # Actualiza la imagen de la cámara
        if self.node.latest_frame is not None:
            frame = self.node.latest_frame
            h, w, ch = frame.shape
            bytes_per_line = ch * w
            qimg = QImage(frame.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
            self.camera_label.setPixmap(QPixmap.fromImage(qimg))

    def start_charging(self):
        self.node.is_charging = True

    def stop_charging(self):
        self.node.is_charging = False

    def closeEvent(self, event):
        """Libera los recursos de la cámara al cerrar."""
        self.cap.release()
        event.accept()


def main():
    rclpy.init()
    ros2_node = ChargingStationInterface()
    app = QApplication(sys.argv)
    main_window = MainWindow(ros2_node)
    main_window.show()

    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(ros2_node, timeout_sec=0.1))
    timer.start(100)

    sys.exit(app.exec())
    rclpy.shutdown()


if __name__ == '__main__':
    main()

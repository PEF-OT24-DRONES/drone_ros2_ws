import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, CompressedImage
from mavros_msgs.msg import State  # Importa el mensaje State de MAVROS
import cv2
import sys
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QLabel, QVBoxLayout, QTabWidget, QWidget, QProgressBar, QPushButton, QLineEdit, QHBoxLayout
)
from mavros_msgs.srv import CommandBool
from PyQt6.QtCore import QTimer, QUrl
from PyQt6.QtGui import QImage, QPixmap
from PyQt6.QtWebEngineWidgets import QWebEngineView
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
from std_msgs.msg import String
from PyQt6.QtCore import Qt
# Importa QTextEdit para el command log
from PyQt6.QtWidgets import QTextEdit
import time
from PyQt6.QtWidgets import QLabel
from PyQt6.QtGui import QImage, QPixmap
import cv2
import numpy as np

class InteractiveMapWidget(QLabel):
    def __init__(self, map_image_path):
        super().__init__()
        self.setFixedSize(800, 400)  # Tamaño del mapa
        self.map_image_path = map_image_path
        self.base_image = cv2.imread(self.map_image_path)  # Cargar imagen base del mapa
        self.map_image = self.base_image.copy()  # Copia para dibujar rutas y puntos
        self.route_points = []  # Lista de puntos de ruta
        self.update_map()  # Inicializa el mapa

    def update_map(self, points=None):
        """Actualiza el mapa con puntos de ruta y líneas conectadas."""
        self.map_image = self.base_image.copy()  # Reinicia la imagen al fondo original

        # Dibuja líneas entre puntos consecutivos
        if len(self.route_points) > 1:
            for i in range(len(self.route_points) - 1):
                start_point = self.route_points[i]
                end_point = self.route_points[i + 1]

                # Ajusta las coordenadas para compensar el desplazamiento visual
                adjusted_start = (start_point[0], start_point[1] + 115)
                adjusted_end = (end_point[0], end_point[1] + 115)
                
                cv2.line(
                    self.map_image,
                    adjusted_start,
                    adjusted_end,
                    (255, 0, 0),  # Azul para las líneas
                    2,  # Grosor de la línea
                )

        # Dibuja puntos de ruta en el mapa
        if points:
            for point in points:
                x, y = point
                cv2.circle(self.map_image, (x, y+115), 10, (0, 0, 255), -1)  # Puntos en rojo

        # Convierte la imagen para PyQt
        height, width, channel = self.map_image.shape
        bytes_per_line = 3 * width
        qimage = QImage(self.map_image.data, width, height, bytes_per_line, QImage.Format.Format_RGB888)
        self.setPixmap(QPixmap.fromImage(qimage))

    def mousePressEvent(self, event):
        """Evento de clic del mouse para agregar un punto en el mapa."""
        x = event.pos().x()
        y = event.pos().y()
        self.route_points.append((x, y))  # Agrega el punto a la lista
        self.update_map(self.route_points)  # Actualiza el mapa con el nuevo punto



class ChargingStationInterface(Node):
    def __init__(self):
        super().__init__('charging_station_interface')

        # QoS compatible con MAVROS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publicador para comandos del motor
        self.motor_control_publisher = self.create_publisher(String, "serial_motor_control", 10)

        # Suscripción al tópico de batería de MAVROS
        self.subscription_battery = self.create_subscription(
            BatteryState, '/mavros/battery', self.battery_callback, qos_profile
        )

        # Suscripción al tópico de estado del dron
        self.subscription_state = self.create_subscription(
            State, '/mavros/state', self.state_callback, qos_profile
        )

        # Variables para almacenar imágenes
        self.normal_frame = None  # Almacena la imagen de la cámara normal
        self.aruco_frame = None  # Almacena la imagen con detección de ArUco

        # Suscripción al tópico de imagen de la cámara normal
        self.subscription_camera = self.create_subscription(
            CompressedImage, '/camera_image/compressed', self.camera_callback, qos_profile
        )

        # Suscripción al tópico de imagen comprimida de detección de ArUco
        self.subscription_aruco = self.create_subscription(
            CompressedImage, '/aruco_detection/compressed', self.aruco_callback, qos_profile
        )

        # Servicio para armar/desarmar el dron
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        while not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando al servicio /mavros/cmd/arming...')

        # SUbscriber al charging station status
        self.subscription_charging = self.create_subscription(String, 'charging_station_status', self.station_listener_callback, qos_profile)

        # Variables para almacenar los datos del dron
        self.is_charging = False
        self.battery_percentage = 0.0
        self.drone_connected = False
        self.drone_armed = False
        self.drone_guided = False
        self.manual_input = False
        self.flight_mode = "UNKNOWN"
        self.latest_frame = None  # Almacena el último frame recibido
        # Inicializa el estado de la estación de carga
        self.charging_station_status = "CLOSED"
        # Funciones para los botones de la pestaña del drone


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
        """Callback para recibir y procesar las imágenes comprimidas de la cámara normal."""
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.normal_frame = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)  # Imagen en grises

    def aruco_callback(self, msg):
        """Callback para recibir y procesar las imágenes comprimidas con detección de ArUco."""
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.aruco_frame = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)  # Imagen en grises

    def station_listener_callback(self, msg):
        """Callback para actualizar el estado de la estación de carga."""
        if msg.data == "OPEN":
            self.charging_station_status = "OPEN"
            self.get_logger().info("Charging station is now OPEN.")
        elif msg.data == "CLOSED":
            self.charging_station_status = "CLOSED"
            self.get_logger().info("Charging station is now CLOSED.")
        else:
            self.get_logger().warning(f"Unknown station status received: {msg.data}")

    def arm_drone(self):
        """Método para armar el dron."""
        arm_cmd = CommandBool.Request()
        arm_cmd.value = True

        future = self.arm_client.call_async(arm_cmd)
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info("Dron armado con éxito.")
        else:
            self.get_logger().error("Error al armar el dron.")

class MainWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node

        self.setWindowTitle("Charging Station Interface")
        # Establecer tamaño de la ventana a pantalla completa
        self.showMaximized()
        #self.setGeometry(200, 200, 650, 600)

        self.view_mode = "normal"  # Modo de visualización de la cámara
        self.camera_on = False  # Estado de la cámara
        self.charging_station_status = "CLOSED"
        
        # Crear el widget de pestañas
        tabs = QTabWidget()

        # Estilo general para las pestañas y widgets
        tabs.setStyleSheet("""
            QTabWidget::pane {border: 2px solid #EEEEEE; border-radius: 5px; padding: 5px;}
            QTabBar::tab {background: #DDDDDD; color: #333333; border: 2px solid #EEEEEE; border-radius: 5px; padding: 5px;}
            QTabBar::tab:selected {background: #EEEEEE; color: #333333;}
        """)

        titles = """
            font-size: 28px;
            font-weight: bold;
            color: #ffffff;
            background-color: #444444;
            padding: 10px;
            border-radius: 5px;
            text-align: center;
        """

        # Modificando la pestaña Charging Station
        station_tab = QWidget()
        station_layout = QVBoxLayout()

        # Título de la pestaña
        station_title = QLabel("Charging Station Control")
        station_title.setStyleSheet("""
            font-size: 24px;
            font-weight: bold;
            color: white;
            background-color: #2B2B2B;
            padding: 10px;
            border-radius: 5px;
            text-align: center;
        """)
        station_layout.addWidget(station_title)

        # Indicadores superiores en un layout horizontal
        top_layout = QHBoxLayout()

        # Indicador de estado de la estación
        self.station_status = QLabel("Status: Available")
        self.station_status.setStyleSheet("""
            font-size: 16px;
            font-weight: bold;
            color: #00CC00;
            background-color: #333333;
            padding: 10px;
            border: 1px solid #00CC00;
            border-radius: 5px;
            text-align: center;
        """)
        self.station_status.setAlignment(Qt.AlignmentFlag.AlignCenter)
        top_layout.addWidget(self.station_status)

        # Indicador de temperatura
        self.temperature_label = QLabel("Temperature: 25°C")
        self.temperature_label.setStyleSheet("""
            font-size: 16px;
            color: white;
            background-color: #333333;
            padding: 10px;
            border: 1px solid #555555;
            border-radius: 5px;
            text-align: center;
        """)
        self.temperature_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        top_layout.addWidget(self.temperature_label)

        # Indicador de velocidad del viento
        self.wind_speed_label = QLabel("Wind Speed: 5.2 m/s")
        self.wind_speed_label.setStyleSheet("""
            font-size: 16px;
            color: white;
            background-color: #333333;
            padding: 10px;
            border: 1px solid #555555;
            border-radius: 5px;
            text-align: center;
        """)
        self.wind_speed_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        top_layout.addWidget(self.wind_speed_label)

        # Indicador de humedad
        self.humidity_label = QLabel("Humidity: 60%")
        self.humidity_label.setStyleSheet("""
            font-size: 16px;
            color: white;
            background-color: #333333;
            padding: 10px;
            border: 1px solid #555555;
            border-radius: 5px;
            text-align: center;
        """)
        self.humidity_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        top_layout.addWidget(self.humidity_label)

        station_layout.addLayout(top_layout)

        # Sección para el temporizador y control del temporizador
        timer_control_layout = QHBoxLayout()

        # Entrada de tiempo para el temporizador
        self.time_input = QLineEdit()
        self.time_input.setPlaceholderText("Enter time in seconds...")
        self.time_input.setStyleSheet("""
            QLineEdit {
                font-size: 16px;
                color: white;
                background-color: #1E1E1E;
                padding: 8px;
                border: 1px solid #555555;
                border-radius: 5px;
            }
        """)
        self.time_input.setFixedWidth(500)
        timer_control_layout.addWidget(self.time_input)

        # Botón para iniciar el temporizador
        self.start_timer_button = QPushButton("Start Timer")
        self.start_timer_button.setStyleSheet("""
            QPushButton {
                font-size: 16px;
                font-weight: bold;
                color: white;
                background-color: #555555;
                border: 1px solid #777777;
                border-radius: 5px;
                padding: 8px;
            }
            QPushButton:hover {
                background-color: #666666;
            }
        """)
        self.start_timer_button.clicked.connect(self.start_timer)
        timer_control_layout.addWidget(self.start_timer_button)

        # Botón para reiniciar el temporizador
        self.reset_timer_button = QPushButton("Reset Timer")
        self.reset_timer_button.setStyleSheet("""
            QPushButton {
                font-size: 16px;
                font-weight: bold;
                color: white;
                background-color: #CC5500;
                border: 1px solid #994400;
                border-radius: 5px;
                padding: 8px;
            }
            QPushButton:hover {
                background-color: #B24400;
            }
        """)
        self.reset_timer_button.clicked.connect(self.reset_timer)
        timer_control_layout.addWidget(self.reset_timer_button)

        station_layout.addLayout(timer_control_layout)

        # Reloj digital para mostrar el temporizador
        self.timer_display = QLabel("00:00")
        self.timer_display.setStyleSheet("""
            font-size: 36px;
            font-weight: bold;
            color: white;
            background-color: #1E1E1E;
            padding: 15px;
            border: 1px solid #555555;
            border-radius: 5px;
            text-align: center;
        """)
        self.timer_display.setAlignment(Qt.AlignmentFlag.AlignCenter)
        station_layout.addWidget(self.timer_display)

        # Modificación de la sección de programación de rutas
        routes_title = QLabel("Flight Route Planner")
        routes_title.setStyleSheet("""
            font-size: 18px;
            font-weight: bold;
            color: white;
            background-color: #2B2B2B;
            padding: 5px;
            border-radius: 5px;
            text-align: center;
        """)
        station_layout.addWidget(routes_title)

        # Layout horizontal para el mapa y los botones
        map_buttons_layout = QHBoxLayout()

        # Contenedor para el mapa interactivo
        map_image_path = os.path.expanduser("~/drone_ros2_ws/src/charging_station/charging_station/udem_map.png")
        self.interactive_map = InteractiveMapWidget(map_image_path)
        self.interactive_map.setStyleSheet("""
            border: 1px solid #555555;
            border-radius: 5px;
        """)
        map_buttons_layout.addWidget(self.interactive_map)

        # Layout vertical para los botones y el contador
        button_info_layout = QVBoxLayout()

        # Contador de puntos actuales
        self.route_point_count = QLabel("Location Points: 0")
        self.route_point_count.setStyleSheet("""
            font-size: 20px;
            font-weight: bold;
            color: white;
            background-color: #2B2B2B;
            padding: 5px;
            border-radius: 5px;
            text-align: center;
        """)
        button_info_layout.addWidget(self.route_point_count)
        #Funcion para actualizar el contador de puntos

        # Botón para borrar todos los puntos
        self.clear_points_button = QPushButton("Clear All Points")
        self.clear_points_button.setStyleSheet("""
            QPushButton {
                font-size: 14px;
                font-weight: bold;
                color: white;
                background-color: #CC5500;
                border: 1px solid #994400;
                border-radius: 5px;
                padding: 8px;
            }
            QPushButton:hover {
                background-color: #B24400;
            }
        """)
        self.clear_points_button.clicked.connect(self.clear_route_points)
        button_info_layout.addWidget(self.clear_points_button)

        # Botón para guardar puntos
        self.save_points_button = QPushButton("Save Points")
        self.save_points_button.setStyleSheet("""
            QPushButton {
                font-size: 14px;
                font-weight: bold;
                color: white;
                background-color: #28a745;
                border: 1px solid #218838;
                border-radius: 5px;
                padding: 8px;
            }
            QPushButton:hover {
                background-color: #218838;
            }
        """)
        self.save_points_button.clicked.connect(self.save_route_points)
        button_info_layout.addWidget(self.save_points_button)

        # Añade el layout de botones al lado derecho del mapa
        map_buttons_layout.addLayout(button_info_layout)
        station_layout.addLayout(map_buttons_layout)


        # Historial de comandos enviados
        log_title = QLabel("Command Log")
        log_title.setStyleSheet("""
            font-size: 18px;
            font-weight: bold;
            color: white;
            background-color: #2B2B2B;
            padding: 5px;
            border-radius: 5px;
            text-align: center;
        """)
        station_layout.addWidget(log_title)

        self.command_log = QTextEdit()
        self.command_log.setReadOnly(True)
        self.command_log.setStyleSheet("""
            QTextEdit {
                font-size: 14px;
                color: white;
                background-color: #1E1E1E;
                padding: 10px;
                border: 1px solid #555555;
                border-radius: 5px;
            }
        """)
        station_layout.addWidget(self.command_log)

        station_tab.setLayout(station_layout)
        tabs.addTab(station_tab, "Charging Station")


        # Temporizador para la interfaz
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_interface)
        self.timer.start(10)

        # Temporizador para el temporizador del usuario
        self.user_timer = QTimer()
        self.user_timer.timeout.connect(self.update_timer)


        # Pestaña de Información del Dron
        drone_tab = QWidget()
        drone_layout = QVBoxLayout()

        # Título de la pestaña
        drone_title = QLabel("Drone Information")
        drone_title.setStyleSheet("""
            font-size: 24px;
            font-weight: bold;
            color: white;
            background-color: #2B2B2B;
            padding: 10px;
            border-radius: 5px;
            text-align: center;
        """)
        drone_layout.addWidget(drone_title)

        # Pestaña de Información del Dron
        drone_tab = QWidget()
        drone_layout = QVBoxLayout()

        # Título de la pestaña
        drone_title = QLabel("Drone Information")
        drone_title.setStyleSheet("""
            font-size: 24px;
            font-weight: bold;
            color: white;
            background-color: #2B2B2B;
            padding: 10px;
            border-radius: 5px;
            text-align: center;
        """)
        drone_layout.addWidget(drone_title)

        # Información del dron en un layout horizontal
        info_layout = QVBoxLayout()

        info_title = QLabel("Drone Status")
        info_title.setStyleSheet("""
            font-size: 18px;
            font-weight: bold;
            color: white;
            background-color: #2B2B2B;
            padding: 5px;
            border-radius: 5px;
            text-align: center;
        """)
        info_layout.addWidget(info_title)

        # Lista de etiquetas de estado del dron
        self.drone_connection_status = QLabel("Connection: No")
        self.drone_armed_status = QLabel("Armed: No")
        self.flight_mode_status = QLabel("Flight Mode: UNKNOWN")
        self.manual_input_status = QLabel("Manual Input: No")

        for label in [self.drone_connection_status, self.drone_armed_status,
                    self.flight_mode_status, self.manual_input_status]:
            label.setStyleSheet("""
                font-size: 16px;
                color: white;
                background-color: #1E1E1E;
                padding: 10px;
                border: 1px solid #555555;
                border-radius: 5px;
                text-align: left;
            """)
            info_layout.addWidget(label)

        drone_layout.addLayout(info_layout)

        # Espacio entre secciones
        drone_layout.addSpacing(15)

        # Barra de progreso de la batería del dron
        battery_layout = QVBoxLayout()
        battery_label = QLabel("Battery Status")
        battery_label.setStyleSheet("""
            font-size: 18px;
            font-weight: bold;
            color: white;
            background-color: #2B2B2B;
            padding: 5px;
            border-radius: 5px;
            text-align: center;
        """)
        battery_layout.addWidget(battery_label)

        self.battery_progress = QProgressBar()
        self.battery_progress.setStyleSheet("""
            QProgressBar {
                border: 2px solid #555555;
                border-radius: 5px;
                text-align: center;
                color: white;
                font-size: 16px;
                background-color: #1E1E1E;
                height: 40px;
            }
            QProgressBar::chunk {
                background-color: qlineargradient(
                    spread:pad, x1:0, y1:0, x2:1, y2:0,
                    stop:0 #00CC00, stop:1 #FFD369
                );
                border-radius: 5px;
            }
        """)
        battery_layout.addWidget(self.battery_progress)
        drone_layout.addLayout(battery_layout)

        # Layout para cámara y botones
        camera_layout = QVBoxLayout()

        # Video en tiempo real de la cámara
        self.camera_label = QLabel(self)
        self.camera_label.setFixedSize(400, 300)
        self.camera_label.setStyleSheet("""
            border: 3px solid #555555;
            border-radius: 10px;
            background-color: #1E1E1E;
        """)
        camera_layout.addWidget(self.camera_label)

        # Botones para la cámara en un layout horizontal
        button_layout = QHBoxLayout()

        self.camera_toggle_button = QPushButton("Camera ON")
        self.camera_toggle_button.setStyleSheet("""
                QPushButton {
                    font-size: 18px;
                    font-weight: bold;
                    color: white;
                    background: qlineargradient(
                        x1:0, y1:0, x2:1, y2:1,
                        stop:0 #28A745, stop:1 #218838
                    );
                    border: 2px solid #1E7E34;
                    border-radius: 10px;
                    padding: 10px;
                    margin-bottom: 10px;
                }
                QPushButton:hover {
                    background: qlineargradient(
                        x1:0, y1:0, x2:1, y2:1,
                        stop:0 #34CE57, stop:1 #28A745
                    );
                }
            """)
        self.camera_toggle_button.clicked.connect(self.toggle_camera)
        button_layout.addWidget(self.camera_toggle_button)

        self.view_switch_button = QPushButton("Normal View")
        self.view_switch_button.setStyleSheet("""
                QPushButton {
                    font-size: 18px;
                    font-weight: bold;
                    color: white;
                    background: qlineargradient(
                        x1:0, y1:0, x2:1, y2:1,
                        stop:0 #F39C12, stop:1 #D35400
                    );
                    border: 2px solid #E67E22;
                    border-radius: 10px;
                    padding: 10px;
                    margin-bottom: 10px;
                }
                QPushButton:hover {
                    background: qlineargradient(
                        x1:0, y1:0, x2:1, y2:1,
                        stop:0 #F8C471, stop:1 #F39C12
                    );
                }
            """)


        self.view_switch_button.clicked.connect(self.toggle_camera_view)
        button_layout.addWidget(self.view_switch_button)

        camera_layout.addLayout(button_layout)
        drone_layout.addLayout(camera_layout)

        # Espacio entre secciones
        drone_layout.addSpacing(20)

        # Finalizar diseño de la pestaña
        drone_tab.setLayout(drone_layout)
        tabs.addTab(drone_tab, "Drone")


        # Modificar el layout de la pestaña "Remote Control"
        control_tab = QWidget()
        control_layout = QVBoxLayout()

        # Título principal
        control_title = QLabel("Remote Control")
        control_title.setStyleSheet("""
            font-size: 24px;
            font-weight: bold;
            color: white;
            background-color: #2B2B2B;
            padding: 10px;
            border-radius: 5px;
            text-align: center;
        """)
        control_layout.addWidget(control_title)

        # Descripción
        control_description = QLabel("Manage your charging station and handle emergency situations.")
        control_description.setStyleSheet("""
            font-size: 16px;
            color: #cccccc;
            background-color: #333333;
            padding: 10px;
            margin-bottom: 15px;
            border-radius: 5px;
        """)
        control_description.setAlignment(Qt.AlignmentFlag.AlignCenter)
        control_layout.addWidget(control_description)

        # Agregar espacio entre elementos
        control_layout.addSpacing(70)

        # Botones de control en un layout horizontal
        button_layout = QVBoxLayout()

        self.open_charge_button = QPushButton("Open Charging Station")
        self.open_charge_button.setStyleSheet("""
            QPushButton {
                font-size: 18px;
                font-weight: bold;
                color: white;
                background: qlineargradient(
                    x1:0, y1:0, x2:1, y2:1,
                    stop:0 #28a745, stop:1 #218838
                );
                border: 2px solid #218838;
                border-radius: 10px;
                padding: 10px;
            }
            QPushButton:hover {
                background: qlineargradient(
                    x1:0, y1:0, x2:1, y2:1,
                    stop:0 #34ce57, stop:1 #28a745
                );
            }
        """)
        self.open_charge_button.clicked.connect(self.open_charging)
        button_layout.addWidget(self.open_charge_button)

        self.close_charge_button = QPushButton("Close Charging Station")
        self.close_charge_button.setStyleSheet("""
            QPushButton {
                font-size: 18px;
                font-weight: bold;
                color: white;
                background: qlineargradient(
                    x1:0, y1:0, x2:1, y2:1,
                    stop:0 #FF9F1A, stop:1 #FF6F00
                );
                border: 2px solid #E67E22;
                border-radius: 10px;
                padding: 10px;
            }
            QPushButton:hover {
                background: qlineargradient(
                    x1:0, y1:0, x2:1, y2:1,
                    stop:0 #FFC371, stop:1 #FF9F1A
                );
            }
        """)
        self.close_charge_button.clicked.connect(self.close_charging)
        button_layout.addWidget(self.close_charge_button)

        self.stop_charge_button = QPushButton("EMERGENCY STOP")
        self.stop_charge_button.setStyleSheet("""
            QPushButton {
                font-size: 20px;
                font-weight: bold;
                color: white;
                background: qlineargradient(
                    x1:0, y1:0, x2:1, y2:1,
                    stop:0 #FF4D4D, stop:1 #C82333
                );
                border: 2px solid #C82333;
                border-radius: 10px;
                padding: 15px;
            }
            QPushButton:hover {
                background: qlineargradient(
                    x1:0, y1:0, x2:1, y2:1,
                    stop:0 #FF6666, stop:1 #FF1A1A
                );
            }
        """)
        self.stop_charge_button.clicked.connect(self.emergency_stop)
        button_layout.addWidget(self.stop_charge_button)

        control_layout.addLayout(button_layout)

        # Historial de logs
        log_title = QLabel("Activity Log")
        log_title.setStyleSheet("""
            font-size: 18px;
            font-weight: bold;
            color: white;
            background-color: #444444;
            padding: 10px;
            border-radius: 5px;
            margin-top: 15px;
        """)
        control_layout.addWidget(log_title)

        self.command_log_remote = QTextEdit()
        self.command_log_remote.setReadOnly(True)
        self.command_log_remote.setStyleSheet("""
            QTextEdit {
                font-size: 14px;
                color: white;
                background-color: #1E1E1E;
                padding: 10px;
                border: 1px solid #555555;
                border-radius: 5px;
                height: 100px;
            }
        """)
        control_layout.addWidget(self.command_log_remote)

        control_tab.setLayout(control_layout)
        tabs.addTab(control_tab, "Remote Control")


        # Establecer el widget de pestañas como el central
        self.setCentralWidget(tabs)

        # Timer para actualizar la interfaz
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_interface)
        self.timer.start(100)
        

    def update_interface(self):
        """Actualiza los elementos de la interfaz en tiempo real."""
        # Actualiza el progreso de la batería
        battery_level = self.node.battery_percentage
        self.battery_progress.setValue(int(battery_level))

        if battery_level > 70:
            self.battery_progress.setStyleSheet("""
            QProgressBar {
                border: 2px solid #555555;
                border-radius: 5px;
                text-align: center;
                color: white;
                font-size: 16px;
                background-color: #1E1E1E;
                height: 40px;
            }
            QProgressBar::chunk {
                background-color: qlineargradient(
                    spread:pad, x1:0, y1:0, x2:1, y2:0,
                    stop:0 #00CC00, stop:1 #FFD369
                );
                border-radius: 5px;
            }
        """)
        elif 40 <= battery_level <= 70:
            self.battery_progress.setStyleSheet("""
            QProgressBar {
                border: 2px solid #555555;
                border-radius: 5px;
                text-align: center;
                color: white;
                font-size: 16px;
                background-color: #1E1E1E;
                height: 40px;
            }
            QProgressBar::chunk {
                background-color: qlineargradient(
                    spread:pad, x1:0, y1:0, x2:1, y2:0,
                    stop:0 #00CC00, stop:1 #FFD369
                );
                border-radius: 5px;
            }
        """)
        else:
            self.battery_progress.setStyleSheet("""
            QProgressBar {
                border: 2px solid #555555;
                border-radius: 5px;
                text-align: center;
                color: white;
                font-size: 16px;
                background-color: #1E1E1E;
                height: 40px;
            }
            QProgressBar::chunk {
                background-color: qlineargradient(
                    spread:pad, x1:0, y1:0, x2:1, y2:0,
                    stop:0 #00CC00, stop:1 #FFD369
                );
                border-radius: 5px;
            }
        """)

        # Actualiza el estado de conexión del dron
        self.drone_connection_status.setText(f"Connection: {'Yes' if self.node.drone_connected else 'No'}")
        self.drone_armed_status.setText(f"Armed: {'Yes' if self.node.drone_armed else 'No'}")
        self.flight_mode_status.setText(f"Flight Mode: {self.node.flight_mode}")
        self.manual_input_status.setText(f"Manual Input: {'Yes' if self.node.manual_input else 'No'}")

        # Actualiza la imagen de la cámara dependiendo del modo de vista
        if self.camera_on:
            frame = None
            if self.view_mode == "normal" and self.node.normal_frame is not None:
                frame = self.node.normal_frame
            elif self.view_mode == "aruco" and self.node.aruco_frame is not None:
                frame = self.node.aruco_frame

            if frame is not None:
                # Verifica si la imagen tiene 2 o 3 dimensiones
                if len(frame.shape) == 2:  # Escala de grises
                    height, width = frame.shape
                    bytes_per_line = width
                    qimage = QImage(frame.data, width, height, bytes_per_line, QImage.Format.Format_Grayscale8)
                elif len(frame.shape) == 3:  # RGB
                    height, width, channel = frame.shape
                    bytes_per_line = 3 * width
                    qimage = QImage(frame.data, width, height, bytes_per_line, QImage.Format.Format_RGB888)
                else:
                    return  # Formato desconocido, no mostrar nada

                # Actualiza el QLabel con la imagen
                self.camera_label.setPixmap(QPixmap.fromImage(qimage))

    def start_charging(self):
        self.node.is_charging = True

    def stop_charging(self):
        self.node.is_charging = False

    def trigger_auto_open(self):
        """Disminuye el tiempo restante y abre la estación automáticamente al llegar a 0."""
        self.remaining_time -= 1
        if self.remaining_time > 0:
            self.timer_label.setText(f"Timer: {self.remaining_time} seconds remaining")
        else:
            self.timer.stop()
            self.timer_label.setText("Timer: Not Set")
            self.open_charging()  # Llama a la función para abrir la estación automáticamente

    def start_timer(self):
        """Inicia el temporizador con el tiempo proporcionado por el usuario."""
        try:
            self.remaining_time = int(self.time_input.text())  # Obtener el tiempo del input
            if self.remaining_time > 0:
                self.node.get_logger().info(f'Timer started with {self.remaining_time} seconds')
                self.user_timer.start(1000)  # Configura el temporizador correcto para que haga 'tick' cada segundo
                self.time_input.setEnabled(False)  # Desactiva el input mientras corre el temporizador
                self.start_timer_button.setEnabled(False)  # Desactiva el botón mientras corre el temporizador
                self.update_timer_display()  # Actualiza la pantalla del temporizador inmediatamente
        except ValueError:
            self.timer_display.setText("Invalid Input")  # Mostrar error si el input no es válido

    def reset_timer(self):
        """Resetea el temporizador a su estado inicial."""
        self.user_timer.stop()
        self.timer_display.setText("00:00")
        self.node.get_logger().info('Timer reset.')
        self.command_log.append("Timer reset.")
        self.time_input.setEnabled(True)
        self.start_timer_button.setEnabled(True)

    def update_timer(self):
        """Actualiza la pantalla del temporizador y verifica el final."""
        if self.remaining_time > 0:
            self.remaining_time -= 1  # Decrementa el tiempo
            self.update_timer_display()  # Actualiza la pantalla del temporizador
        else:
            self.user_timer.stop()  # Detiene el temporizador correcto
            self.timer_display.setText("Time's Up!")  # Mensaje final
            self.time_input.setEnabled(True)  # Rehabilita el input
            self.start_timer_button.setEnabled(True)  # Rehabilita el botón
            
            # Llama a la función para abrir la estación
            self.open_charging()

            # Espera hasta que el Arduino confirme que la estación está abierta
            QTimer.singleShot(10000, self.arm_drone)


    def open_charging(self):
        """Publica comando para abrir la estación de carga."""
        msg = String()
        msg.data = "1"  # Comando para abrir la estación
        self.node.motor_control_publisher.publish(msg)
        self.node.get_logger().info('Comando enviado: Abrir estación (1)')
        self.command_log.append("Charging station opened.")
        self.command_log_remote.append("Charging station opened.")

    def close_charging(self):
        """Publica comando para cerrar la estación de carga."""
        msg = String()
        msg.data = "2"  # Comando para cerrar la estación
        self.node.motor_control_publisher.publish(msg)
        self.node.get_logger().info('Comando enviado: Cerrar estación (2)')
        self.command_log.append("Charging station closed.")
        self.command_log_remote.append("Charging station closed.")


    def update_timer_display(self):
        """Muestra el tiempo restante en formato mm:ss."""
        minutes, seconds = divmod(self.remaining_time, 60)
        self.timer_display.setText(f"{minutes:02}:{seconds:02}")
        

    def emergency_stop(self):
        """Publica comando para detener la estación de carga de emergencia."""
        msg = String()
        msg.data = "0"  # Comando para detener de emergencia
        self.node.motor_control_publisher.publish(msg)
        self.node.get_logger().info('Comando enviado: Detención de emergencia (0)')
        self.command_log_remote.append("Emergency stop command sent.")


    def closeEvent(self, event):
        """Libera los recursos de la cámara al cerrar."""
        self.cap.release()
        event.accept()

    def toggle_camera(self):
        """Activa o desactiva la cámara."""
        if not self.camera_on:
            # Encender la cámara
            self.camera_on = True
            self.camera_toggle_button.setText("Camera OFF")
            self.camera_toggle_button.setStyleSheet("""
                QPushButton {
                    font-size: 18px;
                    font-weight: bold;
                    color: white;
                    background: qlineargradient(
                        x1:0, y1:0, x2:1, y2:1,
                        stop:0 #FF5A5A, stop:1 #C82333
                    );
                    border: 2px solid #A41E1E;
                    border-radius: 10px;
                    padding: 10px;
                    margin-bottom: 10px;
                }
                QPushButton:hover {
                    background: qlineargradient(
                        x1:0, y1:0, x2:1, y2:1,
                        stop:0 #FF7F7F, stop:1 #FF5A5A
                    );
                }
            """)
            # Mostrar un mensaje temporal o imagen cuando la cámara está encendida pero sin feed
            self.camera_label.setText("")
            self.camera_label.setStyleSheet("""
                border: 3px solid #555555;
                border-radius: 10px;
                background-color: #1E1E1E;
            """)
        else:
            # Apagar la cámara
            self.camera_on = False
            self.camera_toggle_button.setText("Camera ON")
            self.camera_toggle_button.setStyleSheet("""
                QPushButton {
                    font-size: 18px;
                    font-weight: bold;
                    color: white;
                    background: qlineargradient(
                        x1:0, y1:0, x2:1, y2:1,
                        stop:0 #28A745, stop:1 #218838
                    );
                    border: 2px solid #1E7E34;
                    border-radius: 10px;
                    padding: 10px;
                    margin-bottom: 10px;
                }
                QPushButton:hover {
                    background: qlineargradient(
                        x1:0, y1:0, x2:1, y2:1,
                        stop:0 #34CE57, stop:1 #28A745
                    );
                }
            """)
            # Cambiar el contenido del QLabel a un fondo vacío
            self.camera_label.setPixmap(QPixmap())  # Limpia cualquier imagen
            self.camera_label.setText("     ")  # Texto para indicar que la cámara está apagada
            self.camera_label.setStyleSheet("""
                font-size: 18px;
                color: #FFFFFF;
                background-color: #1E1E1E;
                text-align: center;
                border: 3px solid #555555;
                border-radius: 10px;
            """)



    def toggle_camera_view(self):
        """Cambia entre vistas de la cámara."""
        if self.view_mode == "normal":
            self.view_mode = "aruco"
            self.view_switch_button.setText("Pose Detection View")
            self.view_switch_button.setStyleSheet("""
                QPushButton {
                    font-size: 18px;
                    font-weight: bold;
                    color: white;
                    background: qlineargradient(
                        x1:0, y1:0, x2:1, y2:1,
                        stop:0 #4A90E2, stop:1 #34495E
                    );
                    border: 2px solid #2C3E50;
                    border-radius: 10px;
                    padding: 10px;
                    margin-bottom: 10px;
                }
                QPushButton:hover {
                    background: qlineargradient(
                        x1:0, y1:0, x2:1, y2:1,
                        stop:0 #5DADE2, stop:1 #4A90E2
                    );
                }
            """)
            # Aquí podrías cambiar el tópico de la cámara o realizar la detección
        else:
            self.view_mode = "normal"
            self.view_switch_button.setText("Normal View")
            self.view_switch_button.setStyleSheet("""
                QPushButton {
                    font-size: 18px;
                    font-weight: bold;
                    color: white;
                    background: qlineargradient(
                        x1:0, y1:0, x2:1, y2:1,
                        stop:0 #F39C12, stop:1 #D35400
                    );
                    border: 2px solid #E67E22;
                    border-radius: 10px;
                    padding: 10px;
                    margin-bottom: 10px;
                }
                QPushButton:hover {
                    background: qlineargradient(
                        x1:0, y1:0, x2:1, y2:1,
                        stop:0 #F8C471, stop:1 #F39C12
                    );
                }
            """)
            # Aquí podrías cambiar el tópico de la cámara o realizar la detección


    def clear_route_points(self):
        """Borra todos los puntos de ruta del mapa."""
        self.interactive_map.route_points = []  # Vacía la lista de puntos
        self.interactive_map.update_map()  # Actualiza el mapa sin puntos
        self.route_point_count.setText("Points: 0")  # Actualiza el contador
        self.command_log.append("All route points cleared.")

    def save_route_points(self):
        """Guarda la lista de puntos de ruta en un archivo de texto."""
        try:
            save_path = os.path.expanduser("~/route_points.txt")
            with open(save_path, "w") as file:
                for point in self.interactive_map.route_points:
                    file.write(f"{point[0]}, {point[1]}\n")
            self.command_log.append(f"Route points saved to {save_path}.")
        except Exception as e:
            self.command_log.append(f"Error saving points: {e}")

    def add_route_point_programmatically(self, x, y):
        """Agrega un punto de ruta al mapa de forma programada."""
        self.interactive_map.route_points.append((x, y))
        self.interactive_map.update_map(self.interactive_map.route_points)
        self.route_point_count.setText(f"Points: {len(self.interactive_map.route_points)}")  # Actualiza el contador
        self.command_log.append(f"Added point programmatically at ({x}, {y}).")

    def arm_drone(self):
        """Conecta el botón con el método del nodo ROS."""
        self.command_log.append("Sending Arm request")
        self.node.arm_drone()
        QTimer.singleShot(20000, self.close_charging)



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

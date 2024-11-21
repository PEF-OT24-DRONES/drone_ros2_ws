import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32

class DroneState(Node):

    def __init__(self):
        super().__init__('drone_state')
        
        # QoS compatible con MAVROS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Cambiar a RELIABLE si es necesario
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.subscription = self.create_subscription(
            BatteryState,
            '/mavros/battery',
            self.bateria_callback,
            qos_profile)
        self.publisher_ = self.create_publisher(Float32, '/nivel_bateria', 10)

    def bateria_callback(self, msg):
        nivel_bateria = msg.percentage * 100.0  # Convertimos a porcentaje (0-100)
        mensaje_bateria = Float32()
        mensaje_bateria.data = nivel_bateria
        self.publisher_.publish(mensaje_bateria)
        self.get_logger().info(f'Nivel de batería: {nivel_bateria:.2f}%')

def main(args=None):
    rclpy.init(args=args)
    drone_state = DroneState()
    rclpy.spin(drone_state)
    drone_state.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


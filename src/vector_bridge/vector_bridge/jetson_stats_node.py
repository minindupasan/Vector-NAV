import rclpy
from rclpy.node import Node
from vector_interfaces.msg import SystemStats
import psutil
import socket
import os

class JetsonStatsNode(Node):
    def __init__(self):
        super().__init__('jetson_stats_node')
        self.publisher_ = self.create_publisher(SystemStats, '/system_stats/jetson', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.get_logger().info('Jetson Stats Node started')

    def get_ip_address(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:
            return "127.0.0.1"

    def get_temperature(self):
        try:
            # Jetson specific
            if os.path.exists("/sys/class/thermal/thermal_zone1/temp"):
                with open("/sys/class/thermal/thermal_zone1/temp", "r") as f:
                    return int(f.read()) / 1000.0
            return 0.0
        except Exception:
            return 0.0

    def get_gpu_usage(self):
        try:
            # Jetson specific
            if os.path.exists("/sys/devices/gpu.0/load"):
                with open("/sys/devices/gpu.0/load", "r") as f:
                    return float(f.read()) / 10.0
            return 0.0
        except Exception:
            return 0.0

    def timer_callback(self):
        msg = SystemStats()
        msg.host_name = "vector-jetson"
        msg.ip_address = self.get_ip_address()
        
        vm = psutil.virtual_memory()
        msg.cpu_usage = float(psutil.cpu_percent())
        msg.memory_usage = float(vm.percent)
        msg.memory_used_gb = float(vm.used) / (1024**3)
        msg.memory_total_gb = float(vm.total) / (1024**3)
        
        msg.gpu_usage = float(self.get_gpu_usage())
        msg.temperature = float(self.get_temperature())
        
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JetsonStatsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

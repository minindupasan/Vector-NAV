import rclpy
from rclpy.node import Node
from vector_interfaces.msg import SystemStats, BatteryStats
import psutil
import socket
import os
import numpy as np

# Try to import board/busio/ads1115 but don't fail if they are missing
try:
    import board
    import busio
    import adafruit_ads1x15.ads1115 as ADS
    from adafruit_ads1x15.analog_in import AnalogIn
    HAS_ADS = True
except ImportError:
    HAS_ADS = False

# 3S LiPo thresholds
_VOLT_TABLE = [12.60, 12.30, 12.12, 11.94, 11.76, 11.55, 11.31, 11.10, 10.80, 10.20, 9.60, 8.40]
_PCT_TABLE  = [100,   90,    80,    70,    60,    50,    40,    30,    20,    10,    5,   0  ]
DIVIDER_RATIO = 4.397

class SystemStatsNode(Node):
    def __init__(self):
        super().__init__('system_stats_node')
        self.stats_pub = self.create_publisher(SystemStats, '/system_stats/pi', 10)
        self.batt_pub = self.create_publisher(BatteryStats, '/battery', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)
        
        self.has_battery_sensor = False
        if HAS_ADS:
            try:
                self.i2c = busio.I2C(board.SCL, board.SDA)
                self.ads = ADS.ADS1115(self.i2c, address=0x48)
                self.ads.gain = 1
                self.chan = AnalogIn(self.ads, 0)
                self.has_battery_sensor = True
                self.get_logger().info('ADS1115 battery sensor initialized')
            except Exception as e:
                self.get_logger().warn(f'Could not initialize ADS1115: {e}')
        else:
            self.get_logger().warn('Adafruit ADS1115 libraries not found')
            
        self.get_logger().info('System Stats Node started')

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
            # Raspberry Pi specific
            if os.path.exists("/sys/class/thermal/thermal_zone0/temp"):
                with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
                    return int(f.read()) / 1000.0
            return 0.0
        except Exception:
            return 0.0

    def voltage_to_percent(self, voltage: float) -> float:
        return float(np.interp(voltage, _VOLT_TABLE[::-1], _PCT_TABLE[::-1]))

    def timer_callback(self):
        # Publish System Stats
        s_msg = SystemStats()
        s_msg.host_name = "vector-rpi"
        s_msg.ip_address = self.get_ip_address()
        s_msg.cpu_usage = float(psutil.cpu_percent())
        s_msg.memory_usage = float(psutil.virtual_memory().percent)
        s_msg.temperature = float(self.get_temperature())
        self.stats_pub.publish(s_msg)
        
        # Publish Battery Stats if available
        if self.has_battery_sensor:
            try:
                b_msg = BatteryStats()
                battery_v = self.chan.voltage * DIVIDER_RATIO
                b_msg.voltage = float(battery_v)
                b_msg.percentage = float(self.voltage_to_percent(battery_v))
                self.batt_pub.publish(b_msg)
            except Exception as e:
                self.get_logger().error(f'Battery read failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = SystemStatsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

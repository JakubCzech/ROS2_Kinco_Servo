import asyncio
import serial
from rclpy.node import Node
from std_msgs.msg import String
import rclpy


class KincoDriver(Node):
    def __init__(self):
        super().__init__(self.__class__.__name__)
        self.declare_parameters(
            namespace="",
            parameters=[
                ("port", "/dev/ttyUSB0"),
                ("baudrate", 115200),
                ("timeout", 0.1),
                ("target_high_topic", "target_high"),
                ("state_topic", "state"),
            ],
        )
        port = self.get_parameter("port").value
        baudrate = self.get_parameter("baudrate").value
        timeout = self.get_parameter("timeout").value
        self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
        self._target_high = self.create_subscription(
            String,
            self.get_parameter("target_high_topic").get_parameter_value().string_value,
            self._target_high_callback,
            10,
        )
        self._state_publisher = self.create_publisher(
            String,
            self.get_parameter("state_topic").get_parameter_value().string_value,
            10,
        )

    async def read_async(self, num_bytes):
        loop = asyncio.get_running_loop()
        return await loop.run_in_executor(None, self.ser.read, num_bytes)

    async def write_async(self, data):
        loop = asyncio.get_running_loop()
        return await loop.run_in_executor(None, self.ser.write, data)

    def _target_high_callback(self, msg):
        # TODO: convert msg from dict to rs232 protocol
        self.write_async(msg.data.encode())

    def close(self):
        self.ser.close()


def main(args=None):
    rclpy.init(args=args)
    kinco_driver = KincoDriver()
    try:
        rclpy.spin(kinco_driver)
    except KeyboardInterrupt:
        pass
    finally:
        kinco_driver.close()
        kinco_driver.destroy_node()
        rclpy.shutdown()

import asyncio
import json
import threading
from rclpy.node import Node
from kinco_driver.utils import RS232Telegram
from std_msgs.msg import Int64, String
import rclpy
from serial_asyncio import open_serial_connection


class KincoDriver(Node):
    def __init__(self):
        super().__init__(self.__class__.__name__)
        self.declare_parameters(
            namespace="",
            parameters=[
                ("port", "/dev/ttyUSB0"),
                ("baudrate", 38400),
                ("target_high_topic", "target_high"),
                ("state_topic", "state"),
                ("frequency", 10.0),
            ],
        )
        port = self.get_parameter("port").value
        baudrate = self.get_parameter("baudrate").value
        self._publish_frequency = 1.0 / self.get_parameter("frequency").value

        self._target_high = self.create_subscription(
            String,
            self.get_parameter("target_high_topic").get_parameter_value().string_value,
            self._target_high_callback,
            10,
        )
        self._state_publisher = self.create_publisher(
            Int64,
            self.get_parameter("state_topic").get_parameter_value().string_value,
            10,
        )
        self.get_logger().debug(f"Timer frequency: {self._publish_frequency}")
        self.loop = asyncio.get_event_loop()

        self._init_params()

        self.loop.create_task(self.connect(port, baudrate))
        self.loop.create_task(self.pos_reading_loop())
        # run loop without blocking
        loop_thread = threading.Thread(target=self.loop.run_forever).start()
        self._is_connected.wait()
        self.create_timer(0.1, self._timer_callback)

        self.get_logger().info("Initialized")

    def _init_params(self):
        self.telegram = RS232Telegram(id=1)
        self._position: int = None
        self.lock = asyncio.Lock()
        self._is_connected = threading.Event()

    @property
    def position(self):
        if not self._position:
            raise ValueError("Position is not readed")
        return self._position

    def _target_high_callback(self, msg: String):
        try:
            _msg = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error("Not valid json")
        else:
            self._start_servo(dir=_msg.get("direction"), speed=_msg.get("speed"))
            self.get_logger().info(
                f"Start servo, direction: {_msg.get('direction')}, speed: {_msg.get('speed')}"
            )

    def _start_servo(self, dir, speed):
        pass

    def _timer_callback(self):
        self.get_logger().debug(f"Position from timer: {self._position}")
        self._state_publisher.publish(Int64(data=self._position))

    async def connect(self, port, baudrate):
        self.get_logger().info(f"Connecting to {port} with baudrate {baudrate}")
        async with self.lock:
            self.reader, self.writer = await open_serial_connection(
                url=port, baudrate=baudrate
            )
        self.get_logger().info("Connected")
        self._is_connected.set()

    async def pos_reading_loop(self):
        self.get_logger().debug("Starting reading loop")
        while True:
            async with self.lock:
                self.writer.write(self.telegram._read_pos())
                data = await self.reader.read(10)

            try:
                __position = self._pos_handler(data)
            except ValueError:
                self.get_logger().error("Not Readed data")
            except IndexError:
                self.get_logger().error("Index Error")
            else:
                self.get_logger().debug(f"Position: {self._position}")
                self._position = __position
            finally:
                await asyncio.sleep(self._publish_frequency)

    def _pos_handler(self, data):
        if data[1] == 0x43:
            return int.from_bytes(data[5:9], byteorder="little", signed=True)
        elif data[1] == 0x80:
            raise IndexError("Index Error")
        else:
            raise ValueError("Not Readed data")


def main(args=None):
    rclpy.init(args=args)
    try:
        kinco_driver = KincoDriver()
        rclpy.spin(kinco_driver)
    except KeyboardInterrupt:
        kinco_driver.destroy_node()
    finally:
        rclpy.shutdown()

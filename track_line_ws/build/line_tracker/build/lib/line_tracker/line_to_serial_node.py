import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class LineToSerialNode(Node):
    def __init__(self):
        super().__init__('line_to_serial_node')

        # Set up the serial connection (update port as needed)
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
            self.get_logger().info("✅ Serial connection opened at /dev/ttyUSB0")
        except serial.SerialException:
            self.get_logger().error("❌ Could not open serial port!")
            self.ser = None

        # Subscribe to /line_tracker
        self.subscription = self.create_subscription(
            String,
            'line_tracker',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        raw = msg.data
        command = raw.strip().upper()
        self.get_logger().info(f"🧼 Cleaned command: {repr(command)}")

        serial_cmd = None

        if "RIGHT" in command:
            serial_cmd = "R\n"
        elif "LEFT" in command:
            serial_cmd = "L\n"
        elif "STRAIGHT" in command:
            serial_cmd = "F\n"
        elif "STOP" in command or "LOST" in command:
            serial_cmd = "S\n"

        if self.ser and serial_cmd:
            self.ser.write(serial_cmd.encode())
            self.get_logger().info(f"📤 Sent over serial: {serial_cmd.strip()}")
        else:
            self.get_logger().warn(f"⚠️ Ignored unknown or invalid message: {command}")


    def destroy_node(self):
        if self.ser:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LineToSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

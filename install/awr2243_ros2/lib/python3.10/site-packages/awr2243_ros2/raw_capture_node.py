import os
import socket
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray, MultiArrayLayout, MultiArrayDimension

class DCAWriter(Node):
    def __init__(self):
        super().__init__('awr2243_raw_capture')
        self.declare_parameter('pc_ip', os.getenv('PC_IP', '192.168.33.180'))
        self.declare_parameter('data_port', int(os.getenv('DCA1000_DATA_PORT', '4098')))
        self.declare_parameter('output_bin', '/tmp/awr2243_capture.bin')
        self.declare_parameter('publish_packets', False)

        self.pc_ip = self.get_parameter('pc_ip').get_parameter_value().string_value
        self.data_port = self.get_parameter('data_port').get_parameter_value().integer_value
        self.output_bin = self.get_parameter('output_bin').get_parameter_value().string_value
        self.publish_packets = self.get_parameter('publish_packets').get_parameter_value().bool_value

        os.makedirs(os.path.dirname(self.output_bin), exist_ok=True)
        self._f = open(self.output_bin, 'wb')

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
        self.sock.bind((self.pc_ip, self.data_port))

        self.pub = self.create_publisher(ByteMultiArray, 'awr2243/raw_packets', 10) if self.publish_packets else None
        self.bytes_written = 0
        self.running = True
        self.get_logger().info(f'Listening UDP {self.pc_ip}:{self.data_port}, writing to {self.output_bin}')
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        layout = MultiArrayLayout(dim=[MultiArrayDimension(label='bytes', size=0, stride=0)])
        while rclpy.ok() and self.running:
            try:
                data, _ = self.sock.recvfrom(8192)
                self._f.write(data)
                self.bytes_written += len(data)
                if self.pub:
                    msg = ByteMultiArray()
                    msg.layout = layout
                    msg.data = list(data)
                    self.pub.publish(msg)
            except Exception as e:
                self.get_logger().error(f'UDP error: {e}')
                break
        self._f.flush()

    def destroy_node(self):
        try:
            self.running = False
            self.sock.close()
            self._f.close()
            self.get_logger().info(f'Total bytes written: {self.bytes_written}')
        except Exception:
            pass
        super().destroy_node()

def main():
    rclpy.init()
    node = DCAWriter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

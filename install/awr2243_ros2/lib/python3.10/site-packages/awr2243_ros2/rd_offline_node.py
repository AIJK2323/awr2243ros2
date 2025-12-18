import os
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayLayout, MultiArrayDimension

# Expect ekurtgl repo on PYTHONPATH
try:
    from radar_utils.RDC_extract_2243 import RDC_extract_2243
except Exception as e:
    raise RuntimeError("Add AWR2243-Radar-GUI repo root to PYTHONPATH so radar_utils.RDC_extract_2243 is importable") from e

class RDOffline(Node):
    def __init__(self):
        super().__init__('awr2243_rd_offline')
        self.declare_parameter('input_bin', '/tmp/awr2243_capture.bin')
        self.input_bin = self.get_parameter('input_bin').get_parameter_value().string_value
        self.pub = self.create_publisher(Float32MultiArray, 'awr2243/range_doppler', 10)
        self.timer = self.create_timer(0.1, self.once)
        self.done = False
        self.get_logger().info(f'Will process {self.input_bin} once and publish RD map.')

    def once(self):
        if self.done:
            return
        self.done = True
        try:
            RDC, params = RDC_extract_2243(self.input_bin)  # shape: [NTS, numChirps, numRX] complex
            rd_mag = self.compute_rd(RDC, params)
            rows, cols = rd_mag.shape
            layout = MultiArrayLayout(dim=[
                MultiArrayDimension(label='range', size=rows, stride=rows*cols),
                MultiArrayDimension(label='doppler', size=cols, stride=cols),
            ])
            msg = Float32MultiArray(layout=layout, data=rd_mag.astype(np.float32).ravel().tolist())
            self.pub.publish(msg)
            self.get_logger().info(f'Published RD map {rows}x{cols} from {self.input_bin}')
        except Exception as e:
            self.get_logger().error(f'Failed to process {self.input_bin}: {e}')

    @staticmethod
    def compute_rd(RDC, params):
        # RDC: [range_samples (NTS), chirps, rx]
        data = RDC  # complex
        # Combine RX channels non-coherently (magnitude sum) for simplicity
        data_combined = np.sum(data, axis=2)  # shape [NTS, chirps]
        # Windowing
        rng_win = np.hanning(data_combined.shape[0])[:, None]
        dop_win = np.hanning(data_combined.shape[1])[None, :]
        X = data_combined * rng_win * dop_win
        # FFT range (axis 0) then Doppler (axis 1)
        R = np.fft.fft(X, n=params.get('RANGE_FFT_SIZE', data_combined.shape[0]), axis=0)
        RD = np.fft.fftshift(np.fft.fft(R, axis=1), axes=1)
        RD_mag = np.abs(RD)
        # Keep positive ranges only (first half of range bins)
        RD_mag = RD_mag[:RD_mag.shape[0]//2, :]
        return RD_mag / (RD_mag.max() + 1e-9)

def main():
    rclpy.init()
    node = RDOffline()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

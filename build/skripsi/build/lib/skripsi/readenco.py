#!/usr/bin/env python3
"""
Tilt Encoder Reader (OP3)
Hanya MEMBACA posisi servo head_tilt dalam DERAJAT (90 = center).
Tidak mengirim perintah ke servo.

Pakai:
    python3 head_encoder_reader.py
    python3 head_encoder_reader.py --rate 5
    python3 head_encoder_reader.py --topic /robotis/present_joint_states
"""
import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState

# Samakan dengan program kontrol kepala
CENTER_INPUT = 90.0   # 90 = center (0 rad). Ubah ke 0.0 kalau mau center = 0
TILT_SIGN = 1.0       # ubah ke -1.0 kalau arah terbalik

JOINT = 'head_tilt'


class TiltReader(Node):

    def __init__(self, topic, rate_hz):
        super().__init__('tilt_encoder_reader')
        self.tilt_rad = None
        self.msg_count = 0
        self.warned = False
        self.start_time = self.get_clock().now()

        self.create_subscription(JointState, topic, self.joint_callback, 10)
        self.create_timer(1.0 / rate_hz, self.print_loop)
        self.get_logger().info(f'Membaca {JOINT} dari {topic}  (Ctrl+C untuk keluar)')

    def joint_callback(self, msg: JointState):
        self.msg_count += 1
        if JOINT in msg.name:
            self.tilt_rad = msg.position[msg.name.index(JOINT)]

    def print_loop(self):
        if self.tilt_rad is None:
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            if elapsed > 3.0 and not self.warned:
                self.warned = True
                self.get_logger().warn(
                    f'Belum ada data {JOINT}. Cek: ros2 topic list | grep joint '
                    'dan pastikan op3_manager jalan.')
            return

        tilt_deg = math.degrees(self.tilt_rad) * TILT_SIGN + CENTER_INPUT
        print(f'\rTILT = {tilt_deg:7.2f}°   ', end='', flush=True)


def main(args=None):
    argv = remove_ros_args(args if args is not None else sys.argv)[1:]

    parser = argparse.ArgumentParser(description='Baca tilt servo kepala OP3 (derajat)')
    parser.add_argument('--topic', default='/robotis/present_joint_states')
    parser.add_argument('--rate', type=float, default=10.0, help='frekuensi print (Hz)')
    opts = parser.parse_args(argv)

    rclpy.init(args=args)
    node = TiltReader(opts.topic, max(opts.rate, 0.1))

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
import math
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

# ── Batas (radian) ──
PAN_MIN,  PAN_MAX  = -1.2, 1.2


# Input terminal: 90 = center (0 rad)
CENTER_INPUT = 90.0
TILT_MIN = math.radians(20.0 - CENTER_INPUT)    # -70° = -1.2217 rad
TILT_MAX = math.radians(160.0 - CENTER_INPUT)   # +70° = +1.2217 rad

# Arah: ubah ke -1.0 kalau arah gerak terbalik
PAN_SIGN  = 1.0
TILT_SIGN = 1.0

# Kecepatan maksimum per tick 0.05 s (rad)
MAX_STEP = 0.04


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def input_to_rad(input_deg, sign):
    """input 90 -> 0 rad, input 0 -> -90°, input 180 -> +90°."""
    return math.radians((input_deg - CENTER_INPUT) * sign)


def rad_to_input(rad, sign):
    return math.degrees(rad) * sign + CENTER_INPUT


class HeadTerminalControl(Node):

    def __init__(self):
        super().__init__('head_terminal_control')

        self.module_pub = self.create_publisher(
            String, '/robotis/enable_ctrl_module', 10)
        self.head_pub = self.create_publisher(
            JointState, '/robotis/head_control/set_joint_states', 10)

        self.head_enabled = False

        self.pan = 0.0
        self.tilt = 0.0
        self.target_pan = 0.0
        self.target_tilt = 0.0 - 5.0

        self._enable_timer = self.create_timer(0.3, self._delayed_enable)
        self.create_timer(0.05, self.control_loop)

        self.get_logger().info('HeadTerminalControl READY')

    # ── Module ──────────────────────────────────────────────
    def _delayed_enable(self):
        self._enable_timer.cancel()
        self.enable_head_module()

    def enable_head_module(self):
        msg = String()
        msg.data = 'head_control_module'
        self.module_pub.publish(msg)
        self.head_enabled = True

    # ── Target dari terminal (input 0..180, 90 = center) ────
    def set_target_input(self, pan_in=None, tilt_in=None):
        if pan_in is not None:
            self.target_pan = clamp(input_to_rad(pan_in, PAN_SIGN),
                                    PAN_MIN, PAN_MAX)
        if tilt_in is not None:
            self.target_tilt = clamp(input_to_rad(tilt_in, TILT_SIGN),
                                     TILT_MIN, TILT_MAX)

        self.enable_head_module()

        print(f'  target -> pan={rad_to_input(self.target_pan, PAN_SIGN):.1f}  '
              f'tilt={rad_to_input(self.target_tilt, TILT_SIGN):.1f}')

    def center(self):
        self.set_target_input(CENTER_INPUT, CENTER_INPUT)

    # ── Loop 20 Hz ──────────────────────────────────────────
    def control_loop(self):
        if not self.head_enabled:
            return

        self.pan += clamp(self.target_pan - self.pan, -MAX_STEP, MAX_STEP)
        self.tilt += clamp(self.target_tilt - self.tilt, -MAX_STEP, MAX_STEP)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['head_pan', 'head_tilt']
        msg.position = [self.pan, self.tilt]
        self.head_pub.publish(msg)

    def status(self):
        print(f'  sekarang: pan={rad_to_input(self.pan, PAN_SIGN):.1f}  '
              f'tilt={rad_to_input(self.tilt, TILT_SIGN):.1f}  '
              f'| target: pan={rad_to_input(self.target_pan, PAN_SIGN):.1f}  '
              f'tilt={rad_to_input(self.target_tilt, TILT_SIGN):.1f}')


def parse_number(text):
    v = float(text)
    if not math.isfinite(v):
        raise ValueError
    return v


def print_help():
    pan_lo = rad_to_input(PAN_MIN, PAN_SIGN)
    pan_hi = rad_to_input(PAN_MAX, PAN_SIGN)
    tilt_lo = rad_to_input(TILT_MIN, TILT_SIGN)
    tilt_hi = rad_to_input(TILT_MAX, TILT_SIGN)
    print('=' * 52)
    print('            HEAD TERMINAL CONTROL')
    print('=' * 52)
    print('Input 0..180, 90 = center')
    print('  <pan> <tilt>   contoh: 120 70')
    print('  p <nilai>      pan saja,  contoh: p 120')
    print('  t <nilai>      tilt saja, contoh: t 70')
    print('  center         kembali ke 90 90')
    print('  enable         kirim ulang enable head module')
    print('  status         tampilkan posisi')
    print('  h              bantuan')
    print('  q              keluar')
    print(f'Batas pan : {min(pan_lo, pan_hi):.0f} s/d {max(pan_lo, pan_hi):.0f}')
    print(f'Batas tilt: {min(tilt_lo, tilt_hi):.0f} s/d {max(tilt_lo, tilt_hi):.0f}')
    print('=' * 52)


def main(args=None):
    rclpy.init(args=args)
    node = HeadTerminalControl()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print_help()

    try:
        while rclpy.ok():
            try:
                raw = input('head> ').strip().lower()
            except EOFError:
                break

            if not raw:
                continue

            parts = raw.split()
            cmd = parts[0]

            if cmd in ('q', 'quit', 'exit'):
                break
            if cmd in ('h', 'help'):
                print_help()
                continue
            if cmd == 'center':
                node.center()
                continue
            if cmd == 'enable':
                node.enable_head_module()
                print('  head module di-enable ulang')
                continue
            if cmd == 'status':
                node.status()
                continue

            try:
                if cmd == 'p' and len(parts) == 2:
                    node.set_target_input(pan_in=parse_number(parts[1]))
                elif cmd == 't' and len(parts) == 2:
                    node.set_target_input(tilt_in=parse_number(parts[1]))
                elif len(parts) == 2:
                    node.set_target_input(parse_number(parts[0]),
                                          parse_number(parts[1]))
                else:
                    raise ValueError
            except ValueError:
                print('  Input salah. Ketik "h" untuk bantuan.')

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
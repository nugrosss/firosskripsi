
#!/usr/bin/env python3
"""
OP3 WALK + POSE BUTTON CONTROL
ROS2 Jazzy / rclpy

Tombol:
- USER  : Bergantian POSE 80 dan POSE 15
- START : Mulai jalan / berhenti manual

Mode berjalan:
- time  : Berjalan berdasarkan waktu
- steps : Berjalan berdasarkan jumlah langkah

Run:
source /home/aroc1/RobotisSoccerYawNew/install/setup.bash
python3 op3_walk_button.py

Monitoring:
ros2 topic echo /op3/walked_cm
"""

import sys
import time
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String, Float32, Int32, Bool
from op3_walking_module_msgs.msg import WalkingParam
from robotis_controller_msgs.srv import SetModule


# ==================================================
# KONFIGURASI ROBOT
# ==================================================

# Parameter gerakan maju
X_MOVE_AMPLITUDE = 0.02
YAW_AMP_DEG = 0.0

# Waktu gait
PERIOD_TIME = 0.700

# Estimasi jarak setiap langkah (cm)
CM_PER_STEP = 2.0

# Frekuensi publish parameter
REPUBLISH_HZ = 2.0

# Debounce tombol
BUTTON_DEBOUNCE = 0.3

# ==================================================
# MODE BERJALAN
# ==================================================

WALK_MODE = "steps"
# Pilihan:
# "time"  = berdasarkan waktu
# "steps" = berdasarkan jumlah langkah

WALK_DURATION_SEC = 10.0
TARGET_STEPS = 10

# ==================================================
# POSE
# ==================================================

POSE_NAIK = 80
POSE_TURUN = 15

# Waktu menunggu pergantian module
MODULE_DELAY = 0.5


class OP3WalkButton(Node):

    def __init__(self):

        super().__init__('op3_walk_button')

        # ------------------------------------------
        # QOS
        # ------------------------------------------

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )

        btn_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )

        # ------------------------------------------
        # PUBLISHER
        # ------------------------------------------

        self.module_pub = self.create_publisher(
            String,
            '/robotis/enable_ctrl_module',
            qos
        )

        self.command_pub = self.create_publisher(
            String,
            '/robotis/walking/command',
            qos
        )

        self.param_pub = self.create_publisher(
            WalkingParam,
            '/robotis/walking/set_params',
            qos
        )

        self.walked_pub = self.create_publisher(
            Float32,
            '/op3/walked_cm',
            10
        )

        # Publisher pose
        self.pose_page_pub = self.create_publisher(
            Int32,
            '/robotis/action/page_num',
            10
        )

        self.pose_start_pub = self.create_publisher(
            Bool,
            '/robotis/action/start_action',
            10
        )

        # ------------------------------------------
        # SERVICE
        # ------------------------------------------

        self.set_module_client = self.create_client(
            SetModule,
            '/robotis/set_present_ctrl_modules'
        )

        # ------------------------------------------
        # SUBSCRIBER
        # ------------------------------------------

        self.button_sub = self.create_subscription(
            String,
            '/robotis/open_cr/button',
            self._on_button,
            btn_qos
        )

        # ------------------------------------------
        # STATUS
        # ------------------------------------------

        self._walking = False
        self._starting = False

        self._pose_is_up = False
        self._pose_busy = False

        self._steps_done = 0
        self._walked_cm = 0.0

        self._walk_start_time = None
        self._active_param = None

        self._last_button = {}

        # ------------------------------------------
        # TIMER
        # ------------------------------------------

        self._republish_timer = None
        self._step_timer = None
        self._duration_timer = None
        self._pose_timer = None

        self.get_logger().info(
            "OP3 WALK + POSE BUTTON CONTROL READY"
        )

    # ==================================================
    # PARAMETER WALKING
    # ==================================================

    def build_param(self, x_amp=0.0, yaw_amp_deg=0.0):

        p = WalkingParam()

        p.init_x_offset = -0.020
        p.init_y_offset = 0.030
        p.init_z_offset = 0.035

        p.init_roll_offset = 0.0
        p.init_pitch_offset = 0.0
        p.init_yaw_offset = 0.0

        p.hip_pitch_offset = math.radians(15.0)

        p.period_time = float(PERIOD_TIME)
        p.dsp_ratio = 0.20
        p.step_fb_ratio = 0.28

        # Gerakan maju
        p.x_move_amplitude = float(x_amp)

        # Tidak bergerak ke samping
        p.y_move_amplitude = 0.0

        p.z_move_amplitude = 0.060

        # Tidak melakukan rotasi
        p.angle_move_amplitude = (
            float(yaw_amp_deg) * 0.017453292519943295
        )

        p.move_aim_on = False

        # Balance
        p.balance_enable = True

        p.balance_hip_roll_gain = 0.35
        p.balance_knee_gain = 0.30
        p.balance_ankle_roll_gain = 0.70
        p.balance_ankle_pitch_gain = 0.90

        p.y_swap_amplitude = 0.028
        p.z_swap_amplitude = 0.006

        p.arm_swing_gain = 0.6
        p.pelvis_offset = 0.0

        p.p_gain = 0
        p.i_gain = 0
        p.d_gain = 0

        return p

    # ==================================================
    # TIMER HELPER
    # ==================================================

    def _schedule_once(self, delay, callback):

        holder = {}

        def execute():

            timer = holder.get('timer')

            if timer is not None:
                timer.cancel()
                self.destroy_timer(timer)

            callback()

        timer = self.create_timer(delay, execute)
        holder['timer'] = timer

        return timer

    # ==================================================
    # ENABLE MODULE
    # ==================================================

    def _request_module(self, module_name, callback):

        req = SetModule.Request()
        req.module_name = module_name

        if not self.set_module_client.service_is_ready():

            self.get_logger().warn(
                f"Service tidak tersedia. "
                f"Menggunakan topic untuk {module_name}"
            )

            msg = String()
            msg.data = module_name
            self.module_pub.publish(msg)

            self._schedule_once(
                MODULE_DELAY,
                callback
            )

            return

        future = self.set_module_client.call_async(req)

        future.add_done_callback(
            lambda f: self._module_result(
                f,
                module_name,
                callback
            )
        )

    def _module_result(self, future, module_name, callback):

        try:
            result = future.result()

        except Exception as e:

            self.get_logger().error(
                f"Gagal mengaktifkan module: {e}"
            )

            result = None

        if result is None or not result.result:

            self.get_logger().warn(
                f"Service gagal. Fallback topic: {module_name}"
            )

            msg = String()
            msg.data = module_name
            self.module_pub.publish(msg)

        else:

            self.get_logger().info(
                f"Module aktif: {module_name}"
            )

        self._schedule_once(
            MODULE_DELAY,
            callback
        )

    # ==================================================
    # BUTTON CALLBACK
    # ==================================================

    def _on_button(self, msg):

        data = msg.data.strip().lower()

        now = time.monotonic()

        last = self._last_button.get(data, 0.0)

        if now - last < BUTTON_DEBOUNCE:
            return

        self._last_button[data] = now

        if data == 'user':

            self._toggle_pose()

        elif data == 'start':

            self._toggle_walk()

    # ==================================================
    # POSE CONTROL
    # ==================================================

    def _toggle_pose(self):

        if self._walking or self._starting:

            self.get_logger().warn(
                "Robot sedang berjalan. Pose ditolak."
            )

            return

        if self._pose_busy:

            self.get_logger().warn(
                "Robot masih memproses pose."
            )

            return

        if self._pose_is_up:

            page = POSE_TURUN

        else:

            page = POSE_NAIK

        self._pose_busy = True

        self.get_logger().info(
            f"Mengaktifkan action module untuk POSE {page}"
        )

        self._request_module(
            "action_module",
            lambda: self._publish_pose(page)
        )

    def _publish_pose(self, page):

        # Publish nomor halaman pose
        msg = Int32()
        msg.data = int(page)

        self.pose_page_pub.publish(msg)

        # Beri jeda agar page_num diterima action module
        self._pose_timer = self._schedule_once(
            0.1,
            lambda: self._start_pose(page)
        )

    def _start_pose(self, page):

        msg = Bool()
        msg.data = True

        self.pose_start_pub.publish(msg)

        self._pose_is_up = (page == POSE_NAIK)

        self._pose_busy = False

        self.get_logger().info(
            f"ROBOT MELAKUKAN POSE {page}"
        )

    # ==================================================
    # WALK BUTTON
    # ==================================================

    def _toggle_walk(self):

        if self._pose_busy:

            self.get_logger().warn(
                "Pose masih diproses."
            )

            return

        if self._starting:

            self.get_logger().warn(
                "Masih proses mengaktifkan walking module."
            )

            return

        if self._walking:

            self._stop_walk()

        else:

            self._start_walk()

    # ==================================================
    # START WALK
    # ==================================================

    def _start_walk(self):

        if WALK_MODE not in ("time", "steps"):

            self.get_logger().error(
                "WALK_MODE harus 'time' atau 'steps'"
            )

            return

        if WALK_MODE == "steps" and TARGET_STEPS <= 0:

            self.get_logger().error(
                "TARGET_STEPS harus lebih dari 0"
            )

            return

        if WALK_MODE == "time" and WALK_DURATION_SEC <= 0:

            self.get_logger().error(
                "WALK_DURATION_SEC harus lebih dari 0"
            )

            return

        self._steps_done = 0
        self._walked_cm = 0.0

        self._starting = True

        self.get_logger().info(
            "Memulai proses walking module..."
        )

        self._request_module(
            "walking_module",
            self._continue_start
        )

    def _continue_start(self):

        if not self._starting:
            return

        # Parameter awal
        idle = self.build_param(0.0, 0.0)

        self.param_pub.publish(idle)

        # Parameter berjalan maju
        self._active_param = self.build_param(
            X_MOVE_AMPLITUDE,
            YAW_AMP_DEG
        )

        self.param_pub.publish(
            self._active_param
        )

        # Mulai berjalan
        self._send_command("start")

        self._walking = True
        self._starting = False

        self._walk_start_time = time.monotonic()

        # Publish ulang parameter
        self._republish_timer = self.create_timer(
            1.0 / REPUBLISH_HZ,
            self._republish_cb
        )

        # Hitung langkah
        self._step_timer = self.create_timer(
            PERIOD_TIME / 2.0,
            self._on_step
        )

        # Timer durasi
        if WALK_MODE == "time":

            self._duration_timer = self.create_timer(
                0.05,
                self._check_duration
            )

        self.get_logger().info(
            "ROBOT MULAI BERJALAN MAJU"
        )

        self.get_logger().info(
            f"Mode       : {WALK_MODE}"
        )

        if WALK_MODE == "steps":

            self.get_logger().info(
                f"Target     : {TARGET_STEPS} langkah"
            )

        else:

            self.get_logger().info(
                f"Durasi     : {WALK_DURATION_SEC} detik"
            )

    # ==================================================
    # WALKING COMMAND
    # ==================================================

    def _send_command(self, cmd):

        msg = String()
        msg.data = cmd

        self.command_pub.publish(msg)

    def _republish_cb(self):

        if self._active_param is not None:

            self.param_pub.publish(
                self._active_param
            )

    # ==================================================
    # STEP COUNTER
    # ==================================================

    def _on_step(self):

        if not self._walking:
            return

        self._steps_done += 1

        self._walked_cm = (
            self._steps_done * CM_PER_STEP
        )

        msg = Float32()
        msg.data = float(self._walked_cm)

        self.walked_pub.publish(msg)

        sys.stdout.write(
            f"\r[LANGKAH {self._steps_done}] "
            f"JARAK {self._walked_cm:.1f} cm"
        )

        sys.stdout.flush()

        # Target jumlah langkah tercapai
        if (
            WALK_MODE == "steps"
            and self._steps_done >= TARGET_STEPS
        ):

            self.get_logger().info(
                "\nTarget langkah tercapai."
            )

            self._stop_walk()

    # ==================================================
    # TIME COUNTER
    # ==================================================

    def _check_duration(self):

        if not self._walking:
            return

        if self._walk_start_time is None:
            return

        elapsed = (
            time.monotonic() - self._walk_start_time
        )

        if elapsed >= WALK_DURATION_SEC:

            self.get_logger().info(
                "Waktu berjalan tercapai."
            )

            self._stop_walk()

    # ==================================================
    # CLEAR TIMER
    # ==================================================

    def _clear_timers(self):

        timer_names = (
            "_republish_timer",
            "_step_timer",
            "_duration_timer"
        )

        for attr in timer_names:

            timer = getattr(self, attr)

            if timer is not None:

                timer.cancel()

                self.destroy_timer(timer)

                setattr(self, attr, None)

    # ==================================================
    # STOP WALK
    # ==================================================

    def _stop_walk(self):

        if not self._walking:
            return

        # Hentikan semua timer
        self._clear_timers()

        # Perintah berhenti
        self._send_command("stop")

        self._active_param = None

        self._walking = False
        self._starting = False

        elapsed = 0.0

        if self._walk_start_time is not None:

            elapsed = (
                time.monotonic() - self._walk_start_time
            )

        self._walk_start_time = None

        # Publish jarak terakhir
        final = Float32()
        final.data = float(self._walked_cm)

        self.walked_pub.publish(final)

        # Tampilkan hasil
        print("\n")
        print("=" * 40)
        print("       HASIL PERGERAKAN ROBOT")
        print("=" * 40)

        print(
            f"Mode          : {WALK_MODE}"
        )

        print(
            f"Total langkah : {self._steps_done}"
        )

        print(
            f"Jarak total   : {self._walked_cm:.1f} cm"
        )

        print(
            f"Waktu berjalan: {elapsed:.2f} detik"
        )

        print("=" * 40)

        self.get_logger().info(
            "ROBOT BERHENTI"
        )

    # ==================================================
    # EMERGENCY STOP
    # ==================================================

    def emergency_stop(self):

        self._clear_timers()

        self._send_command("stop")

        self._active_param = None

        self._walking = False
        self._starting = False

        self.get_logger().warn(
            "EMERGENCY STOP!"
        )

        print("\n")
        print("=" * 40)
        print("       EMERGENCY STOP")
        print("=" * 40)

        print(
            f"Total langkah : {self._steps_done}"
        )

        print(
            f"Jarak total   : {self._walked_cm:.1f} cm"
        )

        print("=" * 40)


# ==================================================
# MAIN
# ==================================================

def main(args=None):

    rclpy.init(args=args)

    node = OP3WalkButton()

    node.get_logger().info(
        "======================================"
    )

    node.get_logger().info(
        "USER  : TOGGLE POSE 80 / 15"
    )

    node.get_logger().info(
        "START : TOGGLE WALK / STOP"
    )

    node.get_logger().info(
        f"WALK MODE : {WALK_MODE}"
    )

    node.get_logger().info(
        "======================================"
    )

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        node.emergency_stop()

    finally:

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == '__main__':

    main()

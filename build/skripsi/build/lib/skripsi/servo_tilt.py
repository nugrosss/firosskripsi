import math
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class ServoTilt(Node):
    """Node ROS2 untuk mengontrol servo tilt robot berdasarkan input derajat."""

    def __init__(self):
        super().__init__('servo_tilt')

        # Parameter serial ke Arduino/mikrokontroler
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('min_degree', 0.0)
        self.declare_parameter('max_degree', 180.0)
        self.declare_parameter('center_degree', 90.0)
        self.declare_parameter('use_serial', True)

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.min_degree = self.get_parameter('min_degree').get_parameter_value().double_value
        self.max_degree = self.get_parameter('max_degree').get_parameter_value().double_value
        self.center_degree = self.get_parameter('center_degree').get_parameter_value().double_value
        self.use_serial = self.get_parameter('use_serial').get_parameter_value().bool_value

        self.current_degree = self.center_degree
        self.ser = None

        if self.use_serial:
            self._connect_serial()

        # Subscribe input derajat dari node lain (misal: vision / manual)
        self.subscription = self.create_subscription(
            Float32,
            '/servo/tilt_degree',
            self.tilt_degree_callback,
            10
        )

        # Publish status sudut aktual
        self.pub_status = self.create_publisher(Float32, '/servo/tilt_status', 10)

        self.get_logger().info(
            f'ServoTilt siap | Port={self.port} | '
            f'Range={self.min_degree:.0f}-{self.max_degree:.0f}° | '
            f'Serial={"ON" if self.use_serial else "OFF (simulasi)"}'
        )

    def _connect_serial(self):
        """Buka koneksi serial ke mikrokontroler."""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1.0
            )
            self.get_logger().info(f'Koneksi serial terbuka: {self.port}')
        except serial.SerialException as e:
            self.ser = None
            self.get_logger().error(
                f'Gagal buka serial {self.port}: {e}. '
                f'Program berjalan dalam mode simulasi.'
            )

    def clamp_degree(self, degree):
        """Batas derajat agar tetap dalam rentang aman servo."""
        return max(self.min_degree, min(self.max_degree, float(degree)))

    def degree_to_pulse(self, degree):
        """
        Konversi derajat ke lebar pulsa mikrosedik (500-2500 us).
        Servo standar: 0° = 500us, 90° = 1500us, 180° = 2500us
        """
        normalized = (degree - self.min_degree) / (self.max_degree - self.min_degree)
        pulse = 500 + normalized * 2000
        return int(round(pulse))

    def move_servo(self, degree):
        """Perintahkan servo tilt ke sudut tertentu."""
        safe_degree = self.clamp_degree(degree)

        if abs(safe_degree - degree) > 0.01:
            self.get_logger().warn(
                f'Derajat {degree:.1f}° di luar rentang, '
                f'dibatasi ke {safe_degree:.1f}°'
            )

        self.current_degree = safe_degree
        pulse = self.degree_to_pulse(safe_degree)

        if self.ser and self.ser.is_open:
            # Format kirim ke Arduino: "T<derajat>\n"
            command = f'T{safe_degree:.2f}\n'
            try:
                self.ser.write(command.encode('utf-8'))
                self.get_logger().info(
                    f'Servo tilt → {safe_degree:.1f}° '
                    f'(pulse={pulse}us) | kirim: {command.strip()}'
                )
            except serial.SerialException as e:
                self.get_logger().error(f'Gagal kirim perintah serial: {e}')
        else:
            self.get_logger().info(
                f'[SIMULASI] Servo tilt → {safe_degree:.1f}° '
                f'(pulse={pulse}us)'
            )

        # Publish status
        msg = Float32()
        msg.data = float(safe_degree)
        self.pub_status.publish(msg)

    def tilt_degree_callback(self, msg):
        """Callback saat menerima input derajat dari topic."""
        degree = msg.data
        self.get_logger().info(f'Menerima input: {degree:.1f}°')
        self.move_servo(degree)

    def manual_input_loop(self):
        """Mode input manual lewat keyboard untuk pengujian."""
        print('\n=== Kontrol Servo Tilt (Input Manual) ===')
        print(f'Rentang: {self.min_degree:.0f}° - {self.max_degree:.0f}°')
        print('Ketik derajat (contoh: 45), atau "q" untuk keluar\n')

        while rclpy.ok():
            # Proses callback ROS2 di background
            rclpy.spin_once(self, timeout_sec=0.1)

            try:
                user_input = input('Derajat: ').strip()
            except EOFError:
                break

            if user_input.lower() == 'q':
                break

            try:
                degree = float(user_input)
                self.move_servo(degree)
            except ValueError:
                print('Input tidak valid! Masukkan angka derajat.\n')

    def destroy_node(self):
        if self.ser and self.ser.is_open:
            # Kembalikan ke posisi tengah sebelum tutup
            self.move_servo(self.center_degree)
            self.ser.close()
            self.get_logger().info('Koneksi serial ditutup')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = ServoTilt()

    try:
        # Mode 1: spin saja (kontrol dari topic /servo/tilt_degree)
        # Mode 2: manual input (uncomment jika ingin input keyboard)
        # node.manual_input_loop()

        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

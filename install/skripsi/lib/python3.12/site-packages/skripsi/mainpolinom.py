import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class KalibrasiJarak(Node):

    def __init__(self):
        super().__init__('kalibrasi_jarak')

        self.subscription = self.create_subscription(
            Float32,
            '/vision/hasil_servo',
            self.hasil_servo_callback,
            10
        )
        self.publisher = self.create_publisher(
            Float32,
            '/vision/hasil_kalibrasi',
            10
        )
        self.get_logger().info('KalibrasiJarak node siap ??')

    def koreksi_jarak(self, jarak_deteksi):
        return (
            0.001919 * jarak_deteksi**2
            + 0.9222 * jarak_deteksi
            + 3.526
        )

    def hasil_servo_callback(self, msg):
        jarak_deteksi = msg.data

        # Masukkan hasil servo ke fungsi koreksi
        jarak_koreksi = self.koreksi_jarak(jarak_deteksi) 
        # self.get_logger().info(
        #         f'Deteksi: {jarak_deteksi:.2f} cm | ')
        if jarak_koreksi < 14:
            jarak_koreksi -= 2
        elif jarak_koreksi < 20:
            jarak_koreksi -= 1
        elif jarak_koreksi > 50:
            jarak_koreksi -= 1
        elif jarak_koreksi > 20:
            jarak_koreksi += 1

        # self.get_logger().info(
        #     # f'Deteksi: {jarak_deteksi:.2f} cm | '
        #     f'jarak: {jarak_koreksi:.2f} cm'
        # )
        print(f"Jarak: {jarak_koreksi:.2f} cm")
        # Publish the corrected distance
        self.publisher.publish(Float32(data=jarak_koreksi))

def main(args=None):
    rclpy.init(args=args)

    node = KalibrasiJarak()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
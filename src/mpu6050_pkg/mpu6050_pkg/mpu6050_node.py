#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import String
import time

# Publica mensagens do tipo String no tópico 'mpu6050_topic'

class MPU6050Read(Node):
    def __init__(self):
        super().__init__('mpu6050_node')

        # parametros
        port = '/dev/ttyUSB0'
        baudrate = 9600

        # publisher, serial config, timer
        self.publisher_ = self.create_publisher(String, 'mpu6050_topic', 10)
        self.ser_ = serial.Serial(port, baudrate, timeout = 1)

        time.sleep(2)
        # for _ in range(5):
        #     self.ser_.readline()

        self.timer_ = self.create_timer(0.1, self.serial_read)
        self.get_logger().info(
            f"mpu6050_node iniciado, publicando em mpu6050_topic")

    def serial_read(self):
        try:
            if self.ser_.in_waiting > 0:
                data = self.ser_.readline().decode('utf-8').rstrip()

                if data:
                    msg = String()
                    msg.data = data
                    self.publisher_.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Erro na leitura: {e}")
        

def main(args=None):
    rclpy.init(args=args)
    serial_node = MPU6050Read()
    rclpy.spin(serial_node)
    serial_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
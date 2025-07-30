#!/usr/bin/env python3  
import rclpy  
import numpy as np
from ahrs.filters import Madgwick
from rclpy.node import Node  
from sensor_msgs.msg import Imu  
import serial 
import time 

# Variaveis globais
GRAVITIY = 9.80665

ACCEL_2G = 16384.0
ACCEL_4G = 8192.0
ACCEL_8G = 4096.0
ACCEL_16G = 2048.0

GYRO_250DEG = 131.0
GYRO_500DEG = 65.5
GYRO_1000DEG = 32.8
GYRO_2000DEG = 16.4
 
class MPU6050SerialNode(Node):  
    def __init__(self):
        super().__init__('mpu6050_serial_node')  

        # parametros 
        port = '/dev/ttyUSB0'  
        baudrate = 9600        

        # publisher, serial config, timer
        self.publisher_ = self.create_publisher(Imu, 'imu', 10)  
        self.serial_port = serial.Serial(port, baudrate, timeout=1)  

        time.sleep(2)
        for _ in range(5):
            self.serial_port.readline()

        self.timer = self.create_timer(0.02, self.read_and_publish)  
        self.get_logger().info(
            f"MPU6050 Serial Node iniciado, publicando em imu") 

        self.madgwick = Madgwick()
        self.q = np.array([1.0, 0.0, 0.0, 0.0]) 

        self.gyro_bias = np.zeros(3)
        self.calibrate_gyro()

    def calibrate_gyro(self):
        samples = 100
        bias = np.zeros(3)
        for _ in range(samples):
            line = self.serial_port.readline().decode().strip()
            if not line:
                continue
            values = [x.strip() for x in line.split(',')]
            if len(values) != 6:
                continue
            _, _, _, gx, gy, gz = [float(x) for x in values]
            bias += np.array([gx, gy, gz])
        self.gyro_bias = bias / samples
        self.get_logger().info(f"Bias do giroscópio: {self.gyro_bias}")

    def read_and_publish(self):
        line = self.serial_port.readline().decode().strip()  # Lê porta serial, converte bytes em string, remove espaços
        
        if not line:
            return 
        
        self.get_logger().info(f"Linha recebida: {line}") # Escreve no log a linha recebida da serial 

        try:
            values = [x.strip() for x in line.split(',')] # Divide a linha por vírgulas e remove espaços            

            if len(values) != 6:
                raise ValueError(f"Linha serial inválida: {line}") # Verifica se há exatamente 6 valores
            ax, ay, az, gx, gy, gz = [float(x) for x in values] # Converte os valores lidos para float            

            imu_msg = Imu()  # Cria uma nova mensagem do tipo Imu 

            # self.madgwick = Madgwick()
            # self.q = np.array([1.0, 0.0, 0.0, 0.0])

            acc = np.array([ax, ay, az])
            gyr = np.array([gx, gy, gz]) - self.gyro_bias

            self.q = self.madgwick.updateIMU(gyr=gyr, acc=acc, q=self.q)

            imu_msg.orientation.w = self.q[0]
            imu_msg.orientation.x = self.q[1]
            imu_msg.orientation.y = self.q[2]
            imu_msg.orientation.z = self.q[3]

            imu_msg.linear_acceleration.x = (ax / ACCEL_2G) * GRAVITIY
            imu_msg.linear_acceleration.y = (ay / ACCEL_2G) * GRAVITIY
            imu_msg.linear_acceleration.z = (az / ACCEL_2G) * GRAVITIY 
            imu_msg.angular_velocity.x = (gx / GYRO_250DEG) * np.pi / 180
            imu_msg.angular_velocity.y = (gy / GYRO_250DEG) * np.pi / 180
            imu_msg.angular_velocity.z = (gz / GYRO_250DEG) * np.pi / 180
            imu_msg.header.stamp = self.get_clock().now().to_msg()  # Timestamp da mensagem
            imu_msg.header.frame_id = 'imu_link'                    # Frame de referência

            self.publisher_.publish(imu_msg) # Publica a mensagem IMU no tópico 'imu'            

        except Exception as e:
            self.get_logger().warn(f"Erro ao ler serial: {e}")  

    # def calibrate_sensor(self, num_samples=100):
    # accel_samples = []
    # gyro_samples = []

    # for _ in range(num_samples):
    #     accel = self.sensor.get_accel_data()
    #     gyro = self.sensor.get_gyro_data()
    #     accel_samples.append([accel['x'], accel['y'], accel['z']])
    #     gyro_samples.append([gyro['x'], gyro['y'], gyro['z']])
    #     rclpy.spin_once(self, timeout_sec=0.01)

    # accel_avg = np.mean(accel_samples, axis=0)
    # gyro_bias = np.mean(gyro_samples, axis=0)

    # # Subtract gravity from Z so we preserve it later
    # gravity = 9.80665  # Standard gravity in m/s²
    # accel_bias = np.array([accel_avg[0], accel_avg[1], accel_avg[2] - gravity])

    # return accel_bias, gyro_bias
            
 
def main(args=None):
    rclpy.init(args=args)  
    node = MPU6050SerialNode()  
    rclpy.spin(node) 
    node.destroy_node()  
    rclpy.shutdown()  

if __name__ == '__main__':
    main()
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray

# Recebe mensagens de 'mpu6050_topic' e aplica filtros nos dados

# Variaveis globais
GRAVITIY = 9.80665

PI = 3.14159

ACCEL_SCALE_MODIFIER_2G = 16384.0
ACCEL_SCALE_MODIFIER_4G = 8192.0
ACCEL_SCALE_MODIFIER_8G = 4096.0
ACCEL_SCALE_MODIFIER_16G = 2048.0

GYRO_SCALE_MODIFIER_250DEG = 131.0
GYRO_SCALE_MODIFIER_500DEG = 65.5
GYRO_SCALE_MODIFIER_1000DEG = 32.8
GYRO_SCALE_MODIFIER_2000DEG = 16.4

ROLL = 0.0
PITCH = 0.0
YAW = 0.0


# Este node lê os dados do mpu6050_node e fitra os dados
class MPU6050FilterNode(Node):
    def __init__(self):
        super().__init__('mpu6050_filter_node')

        self.subscriber_ = self.create_subscription(
            String, 'mpu6050_topic', self.filter_callback, 10)

        self.get_logger().info(
            "mpu6050_filter_node iniciado, lendo de mpu6050_topic")
        
        self.publisher_filter_ax = self.create_publisher(Float32MultiArray, 'filter_topic', 10)

    def filter_callback(self, msg):
        try:
            values = msg.data.split(',')
            if len(values) < 6:
                self.get_logger().error("Dados insuficientes recebidos: {msg.data}")
                return
            
            # converte para float e filtra os primeiros 6 valores
            float_values = [float(val.strip()) for val in values[:6]]

            # resultado do filtro para ax
            res1 = self.filter_ax(float_values[0])
            res2 = self.filter_ay(float_values[1])
            res3 = self.filter_az(float_values[2])
            res4 = self.filter_gx(float_values[3])
            res5 = self.filter_gy(float_values[4])
            res6 = self.filter_gz(float_values[5])

            # publica o resultado
            self.publish_res(res1, res2, res3, res4, res5, res6)

        except ValueError as e:
            self.get_logger().error(f"Erro ao converter dados: {str(e)}")

        except Exception as e:
            self.get_logger().error(f"Erro inesperado: {str(e)}")

    # Publica o resultado do filtro
    def publish_res(self, res1, res2, res3, res4, res5, res6):
        msg_res = Float32MultiArray()
        msg_res.data = [res1, res2, res3, res4, res5, res6]
        self.publisher_filter_ax.publish(msg_res)

    # Converte aceleracao linear para o SI    
    def filter_ax(self, value):
        result = (float(value) / ACCEL_SCALE_MODIFIER_2G) * GRAVITIY
        self.get_logger().info(f"ax: {result:.2f}")
        return result
    
    def filter_ay(self, value):
        result = (float(value) / ACCEL_SCALE_MODIFIER_2G) * GRAVITIY
        self.get_logger().info(f"ay: {result:.2f}")
        return result
    
    def filter_az(self, value):
        result = (float(value) / ACCEL_SCALE_MODIFIER_2G) * GRAVITIY
        self.get_logger().info(f"az: {result:.2f}")
        return result
    
    # Converte velocidade angular para graus por segundo    
    def filter_gx(self, value):
        result = float(value) / GYRO_SCALE_MODIFIER_250DEG
        self.get_logger().info(f"gx: {result:.2f}")
        return result
    
    def filter_gy(self, value):
        result = float(value) / GYRO_SCALE_MODIFIER_250DEG
        self.get_logger().info(f"gy: {result:.2f}")
        return result
    
    def filter_gz(self, value):
        result = float(value) / GYRO_SCALE_MODIFIER_250DEG
        self.get_logger().info(f"gz: {result:.2f}")
        return result
    

def main(args=None):
    rclpy.init(args=args)
    node = MPU6050FilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3  
import rclpy  
import numpy as np
from rclpy.node import Node  
from sensor_msgs.msg import Imu  
# from ahrs.filters import Madgwick
# from std_msgs.msg import String, Int32, Float32, Float32MultiArray
import serial 
import time 

# madgwick = Madgwick()
# acc = np.array([ax, ay, az])

class imuSubscriber(Node):
    def __init__(self):
        super().__init__('imu_subscriber')

        # parametros 
        port = '/dev/ttyUSB0'  
        baudrate = 9600        

        # publisher, serial config, timer
        self.publisher_ = self.create_subscription(
            Imu, 'imu', self.filter_callback, 10)  

        self.get_logger().info(
            "imu_subscriber node iniciado, lendo de imu")  
        
        self.publisher_filter = self.create_publisher(Imu, 'filter_topic', 10)

    def filter_callback(self, msg: Imu):
        try:
            # self.get_logger().info(msg.data)

            ax = msg.linear_acceleration.x
            ay = msg.linear_acceleration.y
            az = msg.linear_acceleration.z

            gx = msg.angular_velocity.x
            gy = msg.angular_velocity.y
            gz = msg.angular_velocity.z

            self.get_logger().info(
                f"Aceleração linear: x={ax:.2f}, y={ay:.2f}, z={az:.2f} | "
                f"Velocidade angular: x={gx:.2f}, y={gy:.2f}, z={gz:.2f}")

        except Exception as e:
            self.get_logger().error(f"Erro: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = imuSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
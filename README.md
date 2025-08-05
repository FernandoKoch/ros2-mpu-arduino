# ROS2 MPU6050 Arduino Serial Read


## __1. Overview__
Repositório de testes com o sensor inercial MPU6050 em ROS2.<br/>
Lendo os dados do sensor IMU a partir de um Arduino que envia esses dados via serial para um nó ROS.<br/>
Desenvolvido para ROS2 Humble, em Ubuntu 22.04 e Arduino UNO.<br/>

## __2. Código para o Arduino__
1. Cole o código encontrado na pasta ``mpu6050_arduino`` em seu Arduino IDE e faça upload para a placa<br/>

2. Abra o monitor serial e espere a calibração do sensor. Mantenha o sensor virado para cima durante a calibração<br/>
Os valores brutos aparecerão após alguns segundos<br/>

3. Feche o monitor serial, a porta ``dev/ttyUSB0`` não deve estar em uso para rodar as aplicações ROS<br/>

4. agora, será possível rodar os programas dentro do ambiente do ROS<br/>


## __3. Descrição dos códigos__
1. mpu6050_node.py:<br/>
Lê via serial os dados de aceleração linear e velocidade angular e publica no tópico  ``/mpu6050_topic ``<br/>
Publica o dado cru no tópico em mensagens do tipo String<br/>
Comando para rodar o nó: ``ros2 run mpu6050_pkg mpu6050_node``<br/>
Comando para observar o tópico: ``ros2 topic echo /mpu6050_topic``<br/>

2. mpu6050_filter_node.py:<br/>
Assinante do tópico  ``/imu `` e publica dados de conversões no tópico ``/filter_topic``<br/>
Faz conversões de dados crus para o SI (Sistema Internacional de Unidades) e publica mensagens do tipo Imu<br/>
Comando para rodar o nó: ``ros2 run mpu6050_pkg mpu6050_filter_node``<br/>
Comando para observar o tópico: ``ros2 topic echo /filter_topic``<br/>

3. mpu6050_serial_node.py:<br/>
Lê via serial os dados de aceleração linear e velocidade angular e publica no tópico  ``/imu ``<br/>
Faz conversões de dados crus para o SI (Sistema Internacional de Unidades) e publica mensagens do tipo Imu<br/>
Comando para rodar o nó: ``ros2 run mpu6050_pkg mpu6050_serial_node``<br/>
Comando para observar o tópico: ``ros2 topic echo /imu``<br/>

4. Demais códigos:<br/>
Testes mal sucedidos a fim de converter os dados do sensor IMU em posição e orientação no espaço<br/>
#include <Wire.h>

// Endereço I2C do MPU6050
const int MPU_ADDR = 0x68;

// Variáveis para armazenar os dados do sensor
int ax, ay, az;
int gx, gy, gz;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  setupMPU();
}

void loop() {
  readMPU();
  printData();
  // delay(100); // Ajuste conforme necessidade
}

void setupMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // Registro PWR_MGMT_1
  Wire.write(0);     // Acorda o sensor
  Wire.endTransmission(true);
}

void readMPU() {
  // Solicita os dados do sensor (14 registros)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // Registro dos dados do acelerômetro
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true); // Solicita 14 bytes

  // Armazena os valores lidos
  ax = Wire.read() << 8 | Wire.read();
  ay = Wire.read() << 8 | Wire.read();
  az = Wire.read() << 8 | Wire.read();
  gx = Wire.read() << 8 | Wire.read();
  gy = Wire.read() << 8 | Wire.read();
  gz = Wire.read() << 8 | Wire.read();
}

void printData() {
  Serial.print(ax); Serial.print(",");
  Serial.print(ay); Serial.print(",");
  Serial.print(az); Serial.print(",");
  Serial.print(gx); Serial.print(",");
  Serial.print(gy); Serial.print(",");
  Serial.println(gz);

  delay(20);
}
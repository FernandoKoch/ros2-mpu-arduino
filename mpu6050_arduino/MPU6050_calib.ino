#include <Wire.h>

const int MPU_ADDR = 0x68;

// Variáveis para os dados do sensor
int ax, ay, az;
int gx, gy, gz;

// Offsets de calibração
int ax_offset = 0, ay_offset = 0, az_offset = 0;
int gx_offset = 0, gy_offset = 0, gz_offset = 0;

// Valor esperado para gravidade no eixo Z (em repouso)
const int GRAVITY = 16384; // ±2g scale: 16384 = 1g

void setup() {
  Serial.begin(9600);
  Wire.begin();
  setupMPU();
  
  // Calibrar o sensor
  calibrateMPU();
}

void loop() {
  readMPU();
  printData();
  delay(100);
}

void setupMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  
  // Configurar fundo de escala do acelerômetro (opcional)  
  // Wire.beginTransmission(MPU_ADDR);
  // Wire.write(0x1C); // Registro ACCEL_CONFIG
  // Wire.write(0x00); // ±2g (default)
  // Wire.endTransmission(true);
}

void calibrateMPU() {
  int amostras = 500;
  long ax_soma = 0, ay_soma = 0, az_soma = 0;
  long gx_soma = 0, gy_soma = 0, gz_soma = 0;
  
  for (int i = 0; i < amostras; i++) {
    readRawMPU();
    ax_soma += ax;
    ay_soma += ay;
    az_soma += az;
    gx_soma += gx;
    gy_soma += gy;
    gz_soma += gz;
    delay(10);
  }
  
  // Calcular offsets
  ax_offset = ax_soma / amostras;
  ay_offset = ay_soma / amostras;
  az_offset = az_soma / amostras;
  // az_offset = (az_soma / amostras) - GRAVITY; //(opcional)
  gx_offset = gx_soma / amostras;
  gy_offset = gy_soma / amostras;
  gz_offset = gz_soma / amostras;
}

void readRawMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  ax = Wire.read() << 8 | Wire.read();
  ay = Wire.read() << 8 | Wire.read();
  az = Wire.read() << 8 | Wire.read();
  gx = Wire.read() << 8 | Wire.read();
  gy = Wire.read() << 8 | Wire.read();
  gz = Wire.read() << 8 | Wire.read();
}

void readMPU() {
  readRawMPU();
  
  // Aplicar offsets
  ax -= ax_offset;
  ay -= ay_offset;
  az -= az_offset;
  gx -= gx_offset;
  gy -= gy_offset;
  gz -= gz_offset;
}

void printData() {
  Serial.print(ax); Serial.print(",");
  Serial.print(ay); Serial.print(",");
  Serial.print(az); Serial.print(",");
  Serial.print(gx); Serial.print(",");
  Serial.print(gy); Serial.print(",");
  Serial.println(gz);
}
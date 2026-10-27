#include <DevLab_MPU6050.h>

// IRAM_ATTR only exists on ESP32; define it as empty on other platforms.
#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

#define SDA_PIN 12
#define SCL_PIN 22
#define MPU6050_ADDR 0x68
#define INT_PIN 4

volatile uint32_t irqCount = 0;

DevLab_MPU6050 imu(Wire, MPU6050_ADDR);
DevLab_MPU6050::SensorData data;

void IRAM_ATTR onMPUInterrupt() {
  irqCount++;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(INT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(INT_PIN), onMPUInterrupt, RISING);

  Serial.println("Inicializando MPU6050...");

  if (!imu.begin(SDA_PIN, SCL_PIN, 400000)) {
    Serial.println("ERROR: MPU6050 no encontrado.");
    while (1) {
      delay(1000);
    }
  }

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x37);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x38);
  Wire.write(0x01);
  Wire.endTransmission();

  Serial.println("MPU6050 inicializado con interrupcion DATA_RDY.");
}

void loop() {
  if (imu.readData(data)) {
    Serial.print("aX = ");
    Serial.print(data.accX);
    Serial.print(" | aY = ");
    Serial.print(data.accY);
    Serial.print(" | aZ = ");
    Serial.print(data.accZ);

    Serial.print(" | temp = ");
    Serial.print(data.temperatureC, 2);

    Serial.print(" | gX = ");
    Serial.print(data.gyrX);
    Serial.print(" | gY = ");
    Serial.print(data.gyrY);
    Serial.print(" | gZ = ");
    Serial.print(data.gyrZ);

    Serial.print(" | irqCount = ");
    Serial.println(irqCount);
  } else {
    Serial.println("ERROR: fallo en la lectura del MPU6050.");
  }

  delay(500);
}

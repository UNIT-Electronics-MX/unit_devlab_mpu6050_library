#include <DevLab_MPU6050.h>

#define SDA_PIN 6
#define SCL_PIN 7
#define MPU6050_ADDR 0x68

DevLab_MPU6050 imu(Wire, MPU6050_ADDR);

SensorData data;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Inicializando MPU6050...");

  if (!imu.begin(SDA_PIN, SCL_PIN, 400000)) {
    Serial.println("ERROR: MPU6050 no encontrado.");
    while (1) {
      delay(1000);
    }
  }

  Serial.println("MPU6050 inicializado correctamente.");
}

void loop() {
  if (imu.readData(data)) {
    Serial.print("aX = ");
    Serial.print(data.accX / 16384.0f);
    Serial.print(" | aY = ");
    Serial.print(data.accY / 16384.0f);
    Serial.print(" | aZ = ");
    Serial.print(data.accZ /4096.0f);

    Serial.print(" | temp = ");
    Serial.print(data.temperatureC, 2);

    Serial.print(" | gX = ");
    Serial.print(data.gyrX);
    Serial.print(" | gY = ");
    Serial.print(data.gyrY);
    Serial.print(" | gZ = ");
    Serial.println(data.gyrZ);
  } else {
    Serial.println("ERROR: fallo en la lectura del MPU6050.");
  }

  delay(1000);
}

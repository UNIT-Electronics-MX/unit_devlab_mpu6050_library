#include <DevLab_MPU6050.h>

#define SDA_PIN 6
#define SCL_PIN 7
#define MPU6050_ADDR 0x68
#define INT_PIN 4

volatile uint32_t irqCount = 0;

DevLab_MPU6050 imu(Wire, MPU6050_ADDR);
SensorData data;
interruptPinConfig intCfg{
  0,0,1,0,0,0,0
};

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

  imu.setIntPinCfg(intCfg);

  imu.enableInterrupt(INT_DATA_READY);


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
    uint8_t intStatus = imu.interruptStatus();
    Serial.println(intStatus,HEX);
  } else {
    Serial.println("ERROR: fallo en la lectura del MPU6050.");
  }

  delay(500);
}

#include "DevLab_MPU6050.h"

#define SDA_PIN 6
#define SCL_PIN 7

DevLab_MPU6050 imu;

uint8_t whoamireg = 0x00;
#define DEFAULT_ADDRESS 0x68

void setup() {
  Serial.begin(115200);

  delay(1000);

  if (!imu.begin(SDA_PIN, SCL_PIN,400000)) {
    Serial.println("ERROR: MPU6050 initialization failed.");

  }else{
    Serial.println("MPU6050 is working fine");

  }
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
  whoamireg = imu.WhoAmI();
  if(whoamireg == DEFAULT_ADDRESS){
      Serial.print("0x");
      Serial.println(whoamireg,HEX);
      delay(1000);
  }

}

# DevLab_MPU6050

Arduino library for the MPU6050 6-axis IMU using direct I2C register communication.

This library provides a lightweight and reliable object-oriented interface for:

- Accelerometer acquisition
- Gyroscope acquisition
- Temperature reading

Compatible with ESP32 and Arduino-compatible platforms.

---

# Features

- Direct register communication
- Object-oriented API
- Lightweight implementation
- High-speed I2C communication
- Supports custom I2C pins
- Optimized for embedded systems and rapid prototyping

---

# Supported Interface

| Interface | Support Status |
|---|---|
| I2C | Supported |

---

# Quick Start Example

```cpp
#include <DevLab_MPU6050.h>

#define SDA_PIN 12
#define SCL_PIN 22

DevLab_MPU6050 imu(Wire, 0x68);
DevLab_MPU6050::SensorData data;

void setup() {
  Serial.begin(115200);

  if (!imu.begin(SDA_PIN, SCL_PIN, 400000)) {
    Serial.println("MPU6050 initialization failed.");
    while (1);
  }
}

void loop() {
  if (imu.readData(data)) {
    Serial.print("ACC X: ");
    Serial.print(data.accX);

    Serial.print(" Y: ");
    Serial.print(data.accY);

    Serial.print(" Z: ");
    Serial.println(data.accZ);

    Serial.print("GYR X: ");
    Serial.print(data.gyrX);

    Serial.print(" Y: ");
    Serial.print(data.gyrY);

    Serial.print(" Z: ");
    Serial.println(data.gyrZ);

    Serial.print("Temperature: ");
    Serial.print(data.temperatureC, 2);
    Serial.println(" °C");
  }

  delay(200);
}
```

---

# Wiring Example

| MPU6050 | MCU |
|---|---|
| SDA | SDA |
| SCL | SCL |
| VCC | 3.3V |
| GND | GND |
| AD0 | GND (0x68) or 3.3V (0x69) |

---

# Compatibility

| MCU Platform | Status |
|---|---|
| ESP32 | Tested |
| Arduino-compatible boards | Compatible |

---

# Folder Structure

```text
DevLab_MPU6050/
├── examples/
│   └── I2C/
│       └── MPU6050_BasicRead/
├── src/
│   ├── DevLab_MPU6050.h
│   └── DevLab_MPU6050.cpp
├── library.properties
└── README.md
```

---

# Version

| Parameter | Value |
|---|---|
| Library Name | DevLab_MPU6050 |
| Version | 1.0.0 |
| Communication | I2C |
| Architecture | Cross-platform |

---

# Author

UNIT Electronics

---

# License

MIT License

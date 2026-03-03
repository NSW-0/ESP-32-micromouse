# ESP-32-micromouse
Arduino codes to use while building a micromouse to solve maze
SYSTEM CONNECTIONS (ESP32 + L298N + SENSORS)


---

Motors to L298N



Motor A

M1 → OUT1

M2 → OUT2


Motor B

M1 → OUT3

M2 → OUT4



---

L298N to ESP32



Motor A Control

ENA → GPIO 25

IN1 → GPIO 26

IN2 → GPIO 27


Motor B Control

ENB → GPIO 19

IN3 → GPIO 18

IN4 → GPIO 17



---

Motor Encoders to ESP32



Motor A Encoder

VCC → 3.3V (ESP32)

GND → GND

C1 → GPIO 34

C2 → GPIO 35


Motor B Encoder

VCC → 3.3V (ESP32)

GND → GND

C1 → GPIO 32

C2 → GPIO 33



---

Power Connections (With Batteries)



Battery to L298N

Battery (+) → 12V (Vs) on L298N

Battery (–) → GND on L298N


L298N to ESP32

5V → ESP32 5V

GND (shared) → ESP32 GND


Important: All grounds must be connected together.


---

VL53L0X Sensors (3x) to ESP32



Power

VCC → 3.3V

GND → GND


I2C

SDA → GPIO 21

SCL → GPIO 22


XSHUT pins

Left → GPIO 16

Front → GPIO 4

Right → GPIO 5



---

MPU-6050 to ESP32



Power

VCC → 3.3V

GND → GND


I2C (shared with VL53L0X)

SDA → GPIO 21

SCL → GPIO 22



---

I2C Summary

GPIO 21 → SDA (VL53L0X + MPU6050)

GPIO 22 → SCL (VL53L0X + MPU6050)


All I2C devices share 3.3V and GND.

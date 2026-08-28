# Self-balaning-robot
A self-balancing robot is essentially a mechanized inverted pendulum that uses electronic vestibular feedback (IMU) to mimic the human reflex of stepping beneath a falling center of mass.
# 🤖 Two-Wheeled Self-Balancing Robot

A two-wheeled self-balancing robot based on the classic **Inverted Pendulum** control problem. Built with an **Arduino Uno**, an **MPU6050 6-DOF IMU** (running on-chip Digital Motion Processor / DMP), and an **L298N Dual H-Bridge Motor Driver** powering geared DC BO motors.

---<img width="1242" height="2208" alt="IMG_5718 JPG" src="https://github.com/user-attachments/assets/aa615b7b-159e-436a-9424-abd291291e92" />



## 📸 Chassis & Hardware Layout

The physical robot features a 3-tier cardboard chassis structure:
* **Top Deck:** MPU6050 6-Axis IMU sensor mounted centrally for angle and acceleration measurement.
* **Middle Deck:** Arduino Uno Microcontroller and L298N Motor Driver module.
* **Bottom Deck:** Dual 18650 Li-ion battery pack, SPST rocker switch, and twin yellow DC BO gear motors with high-traction wheels.

---

## 🛠️ Hardware Components

| Component | Quantity | Description |
| :--- | :---: | :--- |
| **Arduino Uno** | 1 | Main microcontroller unit (ATMega328P) |
| **MPU6050 Module** | 1 | 6-axis Gyroscope + Accelerometer with onboard DMP |
| **L298N Motor Driver** | 1 | Dual H-Bridge DC motor driver |
| **Yellow BO Gear Motors** | 2 | Dual-shaft geared DC motors with rubber wheels |
| **18650 Li-ion Batteries** | 2 | 3.7V cells in series (~7.4V nominal output) |
| **18650 Battery Holder** | 1 | 2-slot series holder |
| **SPST Power Switch** | 1 | Inline main power toggle switch |
| **Connecting Wires** | — | Jumper wires (Male-to-Female, Male-to-Male) |

---

## 🔌 Pin Connections & Wiring

### 1. MPU6050 to Arduino Uno
| MPU6050 Pin | Arduino Pin | Notes |
| :--- | :--- | :--- |
| **VCC** | `5V` | Regulated 5V rail |
| **GND** | `GND` | Common Ground |
| **SCL** | `A5` / `SCL` | I2C Clock line |
| **SDA** | `A4` / `SDA` | I2C Data line |
| **INT** | `D2` | External Interrupt 0 (essential for DMP FIFO data) |

### 2. L298N Motor Driver to Arduino Uno & Power
| L298N Pin | Connected To | Notes |
| :--- | :--- | :--- |
| **12V / VM** | Battery `(+)` via SPST Switch | High-current power for motors (~7.4V) |
| **GND** | Battery `(-)` & Arduino `GND` | **Must share a common ground** |
| **5V (Logic In/Out)**| Arduino `Vin` or `5V` | Powered from L298N onboard 5V regulator |
| **ENA** | Arduino `D11` | Left Motor Speed (PWM pin) |
| **IN1** | Arduino `D7` | Left Motor Direction 1 |
| **IN2** | Arduino `D6` | Left Motor Direction 2 |
| **IN3** | Arduino `D5` | Right Motor Direction 1 |
| **IN4** | Arduino `D4` | Right Motor Direction 2 |
| **ENB** | Arduino `D10` | Right Motor Speed (PWM pin) |

---

## 📦 Required Libraries

Install the following libraries via the Arduino Library Manager (`Sketch` > `Include Library` > `Manage Libraries...`):

1. **PID by Brett Beauregard** (`PID_v1.h`)
2. **I2Cdevlib (MPU6050)** by Jeff Rowberg (`I2Cdev.h` and `MPU6050_6Axis_MotionApps20.h`)
3. **LMotorController** library

---

## 💻 Arduino Firmware

```cpp
// Self Balancing Robot - Full Working Code
#include <PID_v1.h>
#include <LMotorController.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

#define MIN_ABS_SPEED 30

MPU6050 mpu;

// MPU control/status variables
bool dmpReady = false;          // Set true if DMP init was successful
uint8_t mpuIntStatus;           // Holds actual interrupt status byte from MPU
uint8_t devStatus;              // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;            // Expected DMP packet size (default 42 bytes)
uint16_t fifoCount;             // Count of all bytes currently in FIFO
uint8_t fifoBuffer[64];         // FIFO storage buffer

// Orientation / Motion variables
Quaternion q;                   // [w, x, y, z] quaternion container
VectorFloat gravity;            // [x, y, z] gravity vector
float ypr[3];                   // [yaw, pitch, roll] container

// PID Parameters
double originalSetpoint = 180.0; // Desired balance angle in degrees
double setpoint = originalSetpoint;
double input, output;

// PID Tuning Constants (Adjust these for your chassis weight & motors)
double Kp = 40.0;   
double Kd = 2.2;
double Ki = 220.0;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// Motor Speed Scaling Factors
double motorSpeedFactorLeft = 0.9;
double motorSpeedFactorRight = 0.9;

// Motor Controller Pin Definitions (L298N)
const int ENA = 11;
const int IN1 = 7;
const int IN2 = 6;
const int IN3 = 5;
const int IN4 = 4;
const int ENB = 10;

LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);

// MPU Interrupt routine
volatile bool mpuInterrupt = false; 
void dmpDataReady()
{
  mpuInterrupt = true;
}

void setup()
{
  Serial.begin(115200);

  // Initialize I2C bus
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  // Initialize MPU6050
  Serial.println(F("Initializing MPU6050..."));
  mpu.initialize();

  // Verify connection
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // Load and configure DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // Calibrated offsets (replace with your IMU calibration results)
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setZAccelOffset(0);

  if (devStatus == 0)
  {
    // Turn on DMP
    mpu.setDMPEnabled(true);

    // Enable Arduino external interrupt (Pin D2 corresponds to INT 0)
    attachInterrupt(digitalPinToInterrupt(2), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();

    // Setup PID controller
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-255, 255);

    Serial.println(F("DMP ready! System initialized."));
  }
  else
  {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void loop()
{
  if (!dmpReady) return;

  // Wait for MPU interrupt or available packet
  while (!mpuInterrupt && fifoCount < packetSize)
  {
    pid.Compute();
    motorController.move(output, MIN_ABS_SPEED);
  }

  // Reset interrupt flag and get status
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();

  // Handle FIFO overflow
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
  }
  // Process DMP Data
  else if (mpuIntStatus & 0x02)
  {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    // Extract Yaw/Pitch/Roll
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // Calculate current tilt angle in degrees (Pitch)
    input = (ypr[1] * 180.0 / M_PI) + 180.0;

    // Safety cutoff: Stop motors if tilted beyond +/- 30 degrees
    if (input < 150.0 || input > 210.0)
    {
      motorController.stop();
    }
    else
    {
      pid.Compute();
      motorController.move(output, MIN_ABS_SPEED);
    }
  }
}
\```

---

## ⚙️ Calibration & PID Tuning Guide

1. **IMU Calibration:**
   * Place the robot on a flat, level surface and run an MPU6050 calibration sketch to obtain accurate gyro and accelerometer offsets.
   * Update `mpu.setXGyroOffset()`, `mpu.setYGyroOffset()`, `mpu.setZGyroOffset()`, and `mpu.setZAccelOffset()` with your values.

2. **Finding the Balance Setpoint (`originalSetpoint`):**
   * Hold the robot vertically until it rests at its natural balance point without falling over. Note the serial monitor's reported angle and update `originalSetpoint` (typically around `180.0` or slightly offset depending on weight distribution).

3. **PID Parameter Tuning:**
   * Set `Ki = 0` and `Kd = 0`.
   * Increase `Kp` until the robot starts oscillating back and forth about the center balance point.
   * Increase `Kd` to dampen oscillations and smooth the response.
   * Add a small `Ki` to eliminate steady-state error and correct for chassis drift.

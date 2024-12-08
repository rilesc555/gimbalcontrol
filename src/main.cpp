#include <Arduino.h>
#include "BLDCMotor.h"
#include "sensors/MagneticSensorI2C.h"
#include "drivers/BLDCDriver3PWM.h"
// #include "communication/Commander.h"

#include <MPU6050_6Axis_MotionApps20.h>
#include <I2Cdev.h>

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
BLDCDriver3PWM driver = BLDCDriver3PWM(11, 10, 9, 8);
BLDCMotor motor = BLDCMotor(7, 15, 300);

// Commander command = Commander(Serial);
// void doMotor(char* cmd) { command.motor(&motor, cmd); }

MPU6050 mpu;
int const INTERRUPT_PIN = 2;
bool DMPReady = false;
uint8_t MPUIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint8_t FIFOBuffer[64];
Quaternion q;
VectorInt16 aa;
VectorInt16 gy;
VectorFloat gravity;
float ypr[3];
float euler[3];

volatile bool mpuInterrupt = false;
void dmpDataReady() { mpuInterrupt = true; }

void initializeMpu();

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Wire.setClock(400000);

  initializeMpu();
}

void loop() {
  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  }
}

void initializeMpu() {
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);

    if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    mpu.setDMPEnabled(true);

    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    MPUIntStatus = mpu.getIntStatus();

    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } 
}


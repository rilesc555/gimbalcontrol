#include <Arduino.h>
#include "BLDCMotor.h"
#include "drivers/BLDCDriver3PWM.h"
#include "sensors/MagneticSensorI2C.h"
// #include "communication/Commander.h"

#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

#include <Servo.h>

Servo pitchServo;
#define PITCHPIN 5

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
BLDCDriver3PWM driver = BLDCDriver3PWM(11, 10, 9, 8);
BLDCMotor motor = BLDCMotor(7, 15);

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
int loopCount;

volatile bool mpuInterrupt = false;
void dmpDataReady() {
  mpuInterrupt = true;
}

void initializeMpu();

void setup() {
  Serial.begin(115200);
  while (!Serial) {
  };
  Wire.setClock(400000);

  initializeMpu();

  sensor.init();
  // sensor.min_elapsed_time = 0.001;

  driver.voltage_power_supply = 12;
  driver.voltage_limit = 12;

  driver.init();

  motor.linkDriver(&driver);
  motor.linkSensor(&sensor);
  motor.controller = MotionControlType::velocity_openloop;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.voltage_limit = 3;
  motor.current_limit = .4;
  motor.LPF_velocity = 0.05;
  motor.motion_downsample = 10;
  motor.init();
  loopCount = 0;

  motor.initFOC();

  // pitchServo.attach(PITCHPIN);
}

void loop() {
  // if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
  //   mpu.dmpGetQuaternion(&q, FIFOBuffer);
  //   mpu.dmpGetGravity(&gravity, &q);
  //   mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  // }

  motor.loopFOC();

  if (++loopCount > 100) {
    motor.move(0);

    // Serial.print("P\t");
    // Serial.print(ypr[1]);
    // Serial.print("\tR\t");
    // Serial.print(ypr[2]);
    // Serial.print("\tA\t");
    Serial.print(sensor.getAngle() * 180.0 / PI);
    Serial.print("\tV\t");
    Serial.println(motor.shaftVelocity() * 180.0 / PI);
    loopCount = 0;
  }
  // pitchServo.write(ypr[1]*180/PI+90);
  // motor.move(ypr[2]);
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

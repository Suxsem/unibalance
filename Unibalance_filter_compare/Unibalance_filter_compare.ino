#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <PID_v1.h>
#include "Kalman.h"

#define SCL_PIN 4
#define SDA_PIN 5
#define INTERRUPT_PIN 15

MPU6050 mpu;
Kalman kalmanY;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

double angle;
double compl_angle;

unsigned long lastDebug = 0;
unsigned long lastSample = 0;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  Serial.begin(115200);
  Serial.println("start");
  
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  // initialize device
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // load and configure the DMP
  uint8_t devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXAccelOffset(-1700);
  mpu.setYAccelOffset(617);
  mpu.setZAccelOffset(1243);
  mpu.setXGyroOffset(104);
  mpu.setYGyroOffset(-26);
  mpu.setZGyroOffset(10);

  if (devStatus == 0) {
      // turn on the DMP, now that it's ready
      mpu.setDMPEnabled(true);

      // enable Arduino interrupt detection
      attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      dmpReady = true;
      
      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();

      fifoCount = 0;

      delay(5000); // Wait for sensor to stabilize

      kalmanY.setAngle(-270); // Set starting angle

      Serial.println("ok");
      
  } else {
    
    Serial.println("error");
    
  }

}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {

  // if programming failed, don't try to do anything
  if (!dmpReady)
    return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize);

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      fifoCount = 0;

  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;

      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      angle = ypr[1] * 180/M_PI;

      // Variables to store the values from the sensor readings
      int16_t ax, ay, az;
      int16_t gx, gy, gz;

      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

      double r_angle = (atan2(az, ax) * 180.0 / M_PI);
      double omega = 0.2 * gy;
    
      unsigned long nowMicro = micros();
      double dt_s = (double)(nowMicro - lastSample);
      lastSample = nowMicro;
    
      float K = 0.5;
      float A = K / (K + dt_s * 1000);
      compl_angle = A * (compl_angle + omega * dt_s * 0.000001) + (1 - A) * r_angle;
    
      double kal_angle = kalmanY.getAngle(r_angle, omega, dt_s);

      Serial.print("CMP:");
      Serial.println(90 - compl_angle, 5);
      Serial.print("KAL:");
      Serial.println(90 - kal_angle, 5);
      Serial.print("DMP:");
      Serial.println(angle, 5);

      
  }

}

#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <PID_v1.h>

#define NAME "Unibalance"

#define BATT_PIN A0
#define LED_PIN 2

#define SCL_PIN 1
#define SDA_PIN 3

#define MOT_L_F 13
#define MOT_L_B 12
#define MOT_R_F 4
#define MOT_R_B 5

#define INTERRUPT_PIN 15

#define pidPeriod 50 //ms

/* WIFI */
#define DEFAULTssid "Domo"
#define DEFAULTpsw "domopassword"

WiFiServer dbgServer(8888);
WiFiClient dbgClient;

WiFiUDP udp;

MPU6050 mpu;

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

unsigned long lastDebug = 0;
unsigned long lastOta = 0;

double kp = 90 / 10;
double kd = 14 / 10;
double ki = 80 / 10;

double maxV = 350;
double target = 75;

float steer = 0;
float tilt = 0;
double angle = target;
double desidered = target;

double output = 0;

inline int min(int a, int b) { return ((a)<(b) ? (a) : (b)); }

PID pid(&angle, &output, &desidered, kp, ki, kd, DIRECT);

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void ICACHE_RAM_ATTR dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  
  pinMode(LED_PIN, OUTPUT);
  pinMode(BATT_PIN, INPUT);

  pinMode(MOT_L_F, OUTPUT);
  pinMode(MOT_L_B, OUTPUT);
  pinMode(MOT_R_F, OUTPUT);
  pinMode(MOT_R_B, OUTPUT);
  
  analogWrite(MOT_L_F, 0);
  analogWrite(MOT_L_B, 0);
  analogWrite(MOT_R_F, 0);
  analogWrite(MOT_R_B, 0);

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

  /* WIFI */
  ArduinoOTA.setHostname(NAME);
  ArduinoOTA.onStart([]() {
    analogWrite(MOT_L_F, 0);
    analogWrite(MOT_L_B, 0);
    analogWrite(MOT_R_F, 0);
    analogWrite(MOT_R_B, 0);
  });
  WiFi.mode(WIFI_STA);
  WiFi.begin(DEFAULTssid, DEFAULTpsw);

  delay(2000);

  ArduinoOTA.begin();
  dbgServer.begin();
  while (!dbgClient) {
    ArduinoOTA.handle();
    yield();
    dbgClient = dbgServer.available();
  }

  if (devStatus == 0) {
      // turn on the DMP, now that it's ready
      mpu.setDMPEnabled(true);

      // enable Arduino interrupt detection
      //attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      dmpReady = true;
      
      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();

      fifoCount = 0;

      pid.SetSampleTime(pidPeriod);
      pid.SetOutputLimits(-maxV, maxV);

      delay(5000); // Wait for sensor to stabilize

      udp.begin(89);
            
      if (dbgClient && dbgClient.connected())
        dbgClient.println("BOOT COMPLETED");

  } else {
    
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)

      if (dbgClient && dbgClient.connected()) {      
        dbgClient.print("BOOT FAILED: ");
        dbgClient.println(devStatus);
      }

  }

}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void realLoop() {
  char dbgCommand = NULL;
  if (!dbgClient || !dbgClient.connected())
    dbgClient = dbgServer.available();

  if (udp.parsePacket()) {
    byte packetBuffer[4];

    udp.read(packetBuffer, 4);
    memcpy(&steer, packetBuffer, 4);
    
    steer *= 100;
    
    udp.read(packetBuffer, 4);
    memcpy(&tilt, packetBuffer, 4);

    tilt *= 50;
  }

  if (millis() - lastDebug > 1000) {
    if (dbgClient && dbgClient.connected()) {
      //dbgClient.println(angle);
      //dbgClient.println(output);
    }
    lastDebug = millis();
  }

  if (dbgClient && dbgClient.connected() && dbgClient.available()) {
    while (dbgClient.available()) {
      dbgCommand = dbgClient.read();
      if (dbgCommand == 'g') {
        pid.SetMode(AUTOMATIC);
      } else if (dbgCommand == 's') {
        pid.SetMode(MANUAL);
      } else if (dbgCommand == 'p') {
        int i;
        char inBuff[5];
        for (i=0; i<4; i++) {
          inBuff[i] = dbgClient.read();
        }
        inBuff[4] = NULL;
        kp = strtol(inBuff, NULL, 10) / 10;
        pid.SetTunings(kp, ki, kd);
      } else if (dbgCommand == 'd') {
        int i;
        char inBuff[5];
        for (i=0; i<4; i++) {
          inBuff[i] = dbgClient.read();
        }
        inBuff[4] = NULL;
        kd = strtol(inBuff, NULL, 10) / 10;
        pid.SetTunings(kp, ki, kd);
      } else if (dbgCommand == 'i') {
        int i;
        char inBuff[5];
        for (i=0; i<4; i++) {
          inBuff[i] = dbgClient.read();
        }
        inBuff[4] = NULL;
        ki = strtol(inBuff, NULL, 10) / 10;
        pid.SetTunings(kp, ki, kd);
      } else if (dbgCommand == 'm') {
        int i;
        char inBuff[5];
        for (i=0; i<4; i++) {
          inBuff[i] = dbgClient.read();
        }
        inBuff[4] = NULL;
        maxV = strtol(inBuff, NULL, 10);
        pid.SetOutputLimits(-maxV, maxV);
      } else if (dbgCommand == 't') {
        int i;
        char inBuff[5];
        for (i=0; i<4; i++) {
          inBuff[i] = dbgClient.read();
        }
        inBuff[4] = NULL;
        target = strtol(inBuff, NULL, 10);
      } else if (dbgCommand == 'b') {
        if (dbgClient && dbgClient.connected()) {
          noInterrupts();
          dbgClient.println(analogRead(BATT_PIN));
          yield();
          interrupts();
        }
      }
      yield();
    }
  }

  desidered = target + tilt;
  pid.Compute();

  double outputL;
  double outputR;
  if (pid.GetMode() == AUTOMATIC) {
    outputL = output + steer;
    outputR = output - steer;
  } else {
    outputL = 0;
    outputR = 0;
  }
  
  double absOutputL = abs(outputL);
  double absOutputR = abs(outputR);
  
  if (outputL < 0) {
    analogWrite(MOT_L_F, absOutputL);
    analogWrite(MOT_L_B, 0);
  } else {
    analogWrite(MOT_L_F, 0);
    analogWrite(MOT_L_B, absOutputL);
  }
  if (outputR < 0) {
    analogWrite(MOT_R_F, absOutputR);
    analogWrite(MOT_R_B, 0);
  } else {
    analogWrite(MOT_R_F, 0);
    analogWrite(MOT_R_B, absOutputR);
  }

}

void loop() {

  // if programming failed, don't try to do anything
  if (!dmpReady) {
    while(true) {
      ArduinoOTA.handle();
      yield();
    }
  }

  // wait for packet(s) available
  while (fifoCount < packetSize) {

    realLoop();
    
    if (millis() - lastOta > 1000) {
      ArduinoOTA.handle();
      lastOta = millis();
    }

    yield();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
    
  }

  // check for overflow (this should never happen unless our code is too inefficient)
  if (fifoCount == 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      fifoCount = 0;

  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else {

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
      
  }

}

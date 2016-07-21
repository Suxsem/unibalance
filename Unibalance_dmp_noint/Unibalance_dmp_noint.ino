#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

#define NAME "Unibalance"

#define LED_PIN 2

#define SCL_PIN 4 //D2
#define SDA_PIN 5 //D1

//#define MOT_L_F 13
//#define MOT_L_B 12
#define MOT_L_F 1
#define MOT_L_B 3
#define MOT_R_F 1
#define MOT_R_B 3

#define pidPeriod 0.01

/* WIFI */
#define DEFAULTssid NAME
#define DEFAULTpsw "unibalance"

WiFiServer dbgServer(88);
WiFiClient dbgClient;

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

double errSum, prevErr;
double dErr;
double output;

unsigned long lastDebug = 0;
unsigned long lastPid = 0;
unsigned long lastOta = 0;

double kp = 1780;
double kd = 330;
double ki = 50;
double maxV = 350;
double target = 90;

double angle = target;

boolean dontgo = true;

inline int min(int a, int b) { return ((a)<(b) ? (a) : (b)); }

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {

  pinMode(MOT_L_F, OUTPUT);
  pinMode(MOT_L_B, OUTPUT);
  pinMode(MOT_R_F, OUTPUT);
  pinMode(MOT_R_B, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  analogWrite(MOT_L_F, 0);
  analogWrite(MOT_L_B, 0);
  analogWrite(MOT_R_F, 0);
  analogWrite(MOT_R_B, 0);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  // initialize device
  mpu.initialize();

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
  WiFi.mode(WIFI_AP);
  WiFi.softAP(DEFAULTssid, DEFAULTpsw);

  delay(2000);

  ArduinoOTA.begin();
  dbgServer.begin();
  while (!dbgClient)
    dbgClient = dbgServer.available();

  if (devStatus == 0) {
      // turn on the DMP, now that it's ready
      mpu.setDMPEnabled(true);

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      dmpReady = true;

      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();

      fifoCount = 0;
      errSum = 0;

      delay(5000); // Wait for sensor to stabilize
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
  
  if (millis() - lastDebug > 1000) {
    if (dbgClient && dbgClient.connected()) {
      dbgClient.println(angle);
      //dbgClient.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    }
    lastDebug = millis();
  }

  if (dbgClient && dbgClient.connected() && dbgClient.available()) {
    while (dbgClient.available()) {
      dbgCommand = dbgClient.read();
      if (dbgCommand == 'g') {
        dontgo = false;
      } else if (dbgCommand == 's') {
        dontgo = true;
      } else if (dbgCommand == 'p') {
        int i;
        char inBuff[5];
        for (i=0; i<4; i++) {
          inBuff[i] = dbgClient.read();
        }
        inBuff[4] = NULL;
        kp = strtol(inBuff, NULL, 10);
      } else if (dbgCommand == 'd') {
        int i;
        char inBuff[5];
        for (i=0; i<4; i++) {
          inBuff[i] = dbgClient.read();
        }
        inBuff[4] = NULL;
        kd = strtol(inBuff, NULL, 10);
      } else if (dbgCommand == 'i') {
        int i;
        char inBuff[5];
        for (i=0; i<4; i++) {
          inBuff[i] = dbgClient.read();
        }
        inBuff[4] = NULL;
        ki = strtol(inBuff, NULL, 10);
      } else if (dbgCommand == 'm') {
        int i;
        char inBuff[5];
        for (i=0; i<4; i++) {
          inBuff[i] = dbgClient.read();
        }
        inBuff[4] = NULL;
        maxV = strtol(inBuff, NULL, 10);
      } else if (dbgCommand == 't') {
        int i;
        char inBuff[5];
        for (i=0; i<4; i++) {
          inBuff[i] = dbgClient.read();
        }
        inBuff[4] = NULL;
        target = strtol(inBuff, NULL, 10);
      }
    }
  }

  if (dontgo) {
    analogWrite(MOT_L_F, 0);
    analogWrite(MOT_L_B, 0);
    analogWrite(MOT_R_F, 0);
    analogWrite(MOT_R_B, 0);
    return;
  }

  unsigned long now = micros();
  double dt_p = (double)(now - lastPid) / 1000000.0;
  
  if(dt_p >= pidPeriod){
    double err = angle - target;
    errSum = errSum / 5 + err * dt_p;
    dErr = (err - prevErr) / dt_p;
    output = (double)kp/100 * err + (double)ki/1000 * errSum + (double)kd/100 * dErr;
    prevErr = err;
    lastPid = now;
    //LOutput = Output + Run_Speed + Turn_Speed;
    //ROutput = Output + Run_Speed - Turn_Speed;
    double absOutput = abs(output);
    unsigned int finalOut = map(min(1000, absOutput), 0, 1000, 0, maxV);
    if (output > 0) {
      analogWrite(MOT_L_F, finalOut);
      analogWrite(MOT_L_B, 0);
      analogWrite(MOT_R_F, finalOut);
      analogWrite(MOT_R_B, 0);
    } else if(output < 0) {
      analogWrite(MOT_L_F, 0);
      analogWrite(MOT_L_B, finalOut);
      analogWrite(MOT_R_F, 0);
      analogWrite(MOT_R_B, finalOut);
    } else{
      analogWrite(MOT_L_F, 0);
      analogWrite(MOT_L_B, 0);
      analogWrite(MOT_R_F, 0);
      analogWrite(MOT_R_B, 0);
    }
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

    yield();
    
    if (millis() - lastOta > 1000) {
      ArduinoOTA.handle();
      lastOta = millis();
    }

    realLoop();
    
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

      // orientation/motion vars
      Quaternion q;           // [w, x, y, z]         quaternion container
      VectorFloat gravity;    // [x, y, z]            gravity vector
      float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      angle = ypr[1] * 180/M_PI;
      
  }

}

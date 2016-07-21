#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter

#define NAME "Unibalance"

#define LED_PIN 2

#define SDA_PIN 4 //D2
#define SCL_PIN 5 //D1

#define MOT_L_F 13
#define MOT_L_B 12
#define MOT_L_P 14
#define MOT_R_F 16
#define MOT_R_B 3
#define MOT_R_P 1

#define INTERRUPT_PIN 15

#define pi 3.14159265359
#define pidPeriod 0.001

/* WIFI */
#define DEFAULTssid NAME
#define DEFAULTpsw "unibalance"

WiFiServer dbgServer(88);
WiFiClient dbgClient;

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle, gyroYzero; // Angle calculate using the gyro only
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint8_t i2cData[14]; // Buffer for I2C data

double errSum, prevErr;
double dErr;
double output;

unsigned long lastSample = 0;
unsigned long lastDebug = 0;
unsigned long lastPid = 0;

double kp = 1780;
double kd = 330;
double ki = 50;
double maxV = 350;
double angle_offset = 1;

boolean dontgo = true;

uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop);
uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop);
uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes);
inline int min(int a, int b) { return ((a)<(b) ? (a) : (b)); }
void calibrateGyro();

void setup() {

  pinMode(MOT_L_F, OUTPUT);
  pinMode(MOT_L_B, OUTPUT);
  pinMode(MOT_L_P, OUTPUT);
  pinMode(MOT_R_F, OUTPUT);
  pinMode(MOT_R_B, OUTPUT);
  pinMode(MOT_R_P, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  digitalWrite(MOT_L_F, LOW);
  digitalWrite(MOT_L_B, LOW);
  digitalWrite(MOT_L_P, LOW);
  digitalWrite(MOT_R_F, LOW);
  digitalWrite(MOT_R_B, LOW);
  digitalWrite(MOT_R_P, LOW);
  
  /* WIFI */
  ArduinoOTA.setHostname(NAME);
  ArduinoOTA.onStart([]() {
    digitalWrite(MOT_L_F, LOW);
    digitalWrite(MOT_L_B, LOW);
    digitalWrite(MOT_L_P, LOW);
    digitalWrite(MOT_R_F, LOW);
    digitalWrite(MOT_R_B, LOW);
    digitalWrite(MOT_R_P, LOW);
  });
  WiFi.mode(WIFI_AP);
  WiFi.softAP(DEFAULTssid, DEFAULTpsw);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000UL);

  delay(2000);

  ArduinoOTA.begin();
  dbgServer.begin();
  while (!dbgClient)
    dbgClient = dbgServer.available();

  if (dbgClient && dbgClient.connected())
    dbgClient.println("BOOT COMPLETE");

  /* test */
  //digitalWrite(MOT_L_B, HIGH);
  //analogWrite(MOT_L_P, 200);
  //digitalWrite(MOT_R_B, HIGH);
  //analogWrite(MOT_R_P, 100);
  
  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    if (dbgClient && dbgClient.connected())
    dbgClient.println("Error reading sensor");
    while(1)
      yield();
  }

  delay(100); // Wait for sensor to stabilize

  calibrateGyro();
  
  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)(i2cData[0] << 8) | i2cData[1];
  accY = (int16_t)(i2cData[2] << 8) | i2cData[3];
  accZ = (int16_t)(i2cData[4] << 8) | i2cData[5];

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
  #ifdef RESTRICT_PITCH // Eq. 25 and 26
    double roll  = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  #else // Eq. 28 and 29
    double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  #endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;

  errSum = 0;
  
  lastSample = micros();
}

void loop() {

  ArduinoOTA.handle();

  char dbgCommand = NULL;
  if (!dbgClient || !dbgClient.connected())
    dbgClient = dbgServer.available();

  /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (int16_t)(i2cData[6] << 8) | i2cData[7];
  gyroX = (int16_t)(i2cData[8] << 8) | i2cData[9];
  gyroY = (int16_t)(i2cData[10] << 8) | i2cData[11];
  gyroZ = (int16_t)(i2cData[12] << 8) | i2cData[13];

  double dt_s = (double)(micros() - lastSample) / 1000000; // Calculate delta time
  lastSample = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
  #ifdef RESTRICT_PITCH // Eq. 25 and 26
    double roll  = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  #else // Eq. 28 and 29
    double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  #endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = ((float)gyroX - gyroYzero) / 131.0; // Convert to deg/s

  #ifdef RESTRICT_PITCH
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
      kalmanX.setAngle(roll);
      kalAngleX = roll;
      gyroXangle = roll;
    } else
      kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt_s); // Calculate the angle using a Kalman filter
  
    if (abs(kalAngleX) > 90)
      gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt_s);
  #else
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
      kalmanY.setAngle(pitch);
      kalAngleY = pitch;
      gyroYangle = pitch;
    } else
      kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt_s); // Calculate the angle using a Kalman filter
  
    if (abs(kalAngleY) > 90)
      gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt_s); // Calculate the angle using a Kalman filter
  #endif

  gyroXangle += gyroXrate * dt_s; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt_s;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

  double angle = kalAngleY + 90 + angle_offset;
  // -----------
  
  if (millis() - lastDebug > 1000) {
    if (dbgClient && dbgClient.connected()) {
      //dbgClient.println(kalAngleX);
      //dbgClient.println(angle);
      /*dbgClient.println(output);
      dbgClient.println(prevErr);
      dbgClient.println(dErr);*/
      //dbgClient.println("-------------");
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
      } else if (dbgCommand == 'o') {
        int i;
        char inBuff[5];
        for (i=0; i<4; i++) {
          inBuff[i] = dbgClient.read();
        }
        inBuff[4] = NULL;
        angle_offset = strtol(inBuff, NULL, 10);
      }
    }
  }

  //if (abs(angle) < 8 || dontgo) {
  if (dontgo) {
    analogWrite(MOT_L_P, 0);
    analogWrite(MOT_R_P, 0);
    return;
  }

  unsigned long now = micros();
  double dt_p = (double)(now - lastPid) / 1000000.0;
  
  //if(dt_p >= pidPeriod){
    //double err = accelerometer_data[2] + 1500;
    double err = angle;
    errSum = errSum / 5 + err * dt_p;
    dErr = (err - prevErr) / dt_p;
    output = (double)kp/100 * err + (double)ki/1000 * errSum + (double)kd/100 * dErr;
    prevErr = err;
    lastPid = now;
    //LOutput = Output + Run_Speed + Turn_Speed;
    //ROutput = Output + Run_Speed - Turn_Speed;
    if (output > 0) {
      digitalWrite(MOT_L_F, HIGH);
      digitalWrite(MOT_L_B, LOW);
      digitalWrite(MOT_R_F, HIGH);
      digitalWrite(MOT_R_B, LOW);
    } else if(output < 0) {
      digitalWrite(MOT_L_F, LOW);
      digitalWrite(MOT_L_B, HIGH);
      digitalWrite(MOT_R_F, LOW);
      digitalWrite(MOT_R_B, HIGH);
    } else{
      digitalWrite(MOT_L_F, LOW);
      digitalWrite(MOT_L_B, LOW);
      digitalWrite(MOT_R_F, LOW);
      digitalWrite(MOT_R_B, LOW);
    }
    double absOutput = abs(output);
    unsigned int finalOut = map(min(1000, absOutput), 0, 1000, 0, maxV);
    analogWrite(MOT_L_P, finalOut);
    analogWrite(MOT_R_P, finalOut);
  //}

}

void calibrateGyro() {
  int16_t gyroYbuffer[25];
  for (uint8_t i = 0; i < 25; i++) {
    while (i2cRead(0x43, i2cData, 2));
    gyroYbuffer[i] = ((i2cData[0] << 8) | i2cData[1]);
    delay(10);
  }
  for (uint8_t i = 0; i < 25; i++)
    gyroYzero += gyroYbuffer[i];
  gyroYzero /= 25.0f;
}

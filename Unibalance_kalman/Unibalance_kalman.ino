#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include "Kalman.h"

#define NAME "Unibalance"

#define LED_PIN 2

#define SDA_PIN 4 //D2
#define SCL_PIN 5 //D1

#define MOT_L_F 13
#define MOT_L_B 12
#define MOT_L_P 14
#define MOT_R_F 15
#define MOT_R_B 3
#define MOT_R_P 1

#define Angle_offset -274 //-270 +4
#define pi 3.14159265359
#define Gry_offset 0
#define Gyr_Gain 0.2

#define pidPeriod 0.001

#define gyroAddress 0x68
#define adxlAddress 0x53

/* WIFI */
#define DEFAULTssid NAME
#define DEFAULTpsw "unibalance"

WiFiServer dbgServer(88);
WiFiClient dbgClient;

inline int min(int a, int b) { return ((a)<(b) ? (a) : (b)); }
void i2cWrite(uint8_t address, uint8_t registerAddress, uint8_t data);
uint8_t* i2cRead(uint8_t address, uint8_t registerAddress, uint8_t nbytes);
int16_t readGyroX();
int16_t readGyroY();
double getXangle();
double getYangle();
int16_t readAccX();
int16_t readAccY();
int16_t readAccZ();

Kalman kalmanY;
double gyroYangle = 0;
//double zeroValue[5] = { -200, 44, 660, 52.3, -18.5}; // Found by experimenting
double zeroValue[5] = { 0, 0, 0, 0, 0}; // Found by experimenting
uint8_t buffer[2]; // I2C buffer

double r_angle, omega, f_angle, rec_angle;
double errSum, prevErr;
double dErr;
double output;

unsigned long lastSample = 0;
unsigned long lastDebug = 0;
unsigned long lastPid = 0;

double kp = 180;
double kd = 120;
double ki = 0;

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
  
  i2cWrite(adxlAddress, 0x31, 1 << 2); // Full resolution mode
  i2cWrite(adxlAddress, 0x2D, 0x08); // Setup ADXL345 for constant measurement mode

  i2cWrite(gyroAddress, 0x16, 0x1A); // this puts your gyro at +-2000deg/sec  and 98Hz Low pass filter
  i2cWrite(gyroAddress, 0x15, 0x09); // this sets your gyro at 100Hz sample rate

  kalmanY.setAngle(-270); // Set starting angle
  lastSample = micros();
}

void loop() {

  ArduinoOTA.handle();

  char dbgCommand = NULL;
  if (!dbgClient || !dbgClient.connected())
    dbgClient = dbgServer.available();
  if (dbgClient && dbgClient.connected() && dbgClient.available())
    while (dbgClient.available())
      dbgCommand = dbgClient.read();

  if (dbgCommand)
    if (dbgClient && dbgClient.connected())
      dbgClient.println(dbgCommand);

  double gyroYrate = (((double)readGyroY() - zeroValue[4]) / 14.375);
  gyroYangle += gyroYrate * ((double)(micros() - lastSample) / 1000000); // Without any filter

  double accYangle = getYangle();

  double yAngle = -(kalmanY.getAngle(accYangle, gyroYrate, (double)(micros() - lastSample)) + Angle_offset); // calculate the angle using a Kalman filter

  lastSample = micros();

  if (millis() - lastDebug > 1000) {
    if (dbgClient && dbgClient.connected()) {
      //dbgClient.println(yAngle);
      /*dbgClient.println(output);
      dbgClient.println(prevErr);
      dbgClient.println(dErr);*/
      //dbgClient.println("-------------");
    }
    lastDebug = millis();
  }


  if (abs(yAngle) > 60) {
    analogWrite(MOT_L_P, 0);
    analogWrite(MOT_R_P, 0);
    return;
  }
  
  //test
  digitalWrite(LED_PIN, (yAngle) >= 0);

  unsigned long now = millis();
  double dt_p = (double)(now - lastPid) / 1000.0;
  
  if(dt_p >= pidPeriod){
    //double err = accelerometer_data[2] + 1500;
    double err = yAngle;
    errSum += err * dt_p;
    dErr = (err - prevErr) / dt_p;
    output = kp * err + ki * errSum + kd * dErr;
    prevErr = err;
    lastPid = now;
    //LOutput = Output + Run_Speed + Turn_Speed;
    //ROutput = Output + Run_Speed - Turn_Speed;
    if (output < 0) {
      digitalWrite(MOT_L_F, HIGH);
      digitalWrite(MOT_L_B, LOW);
      digitalWrite(MOT_R_F, HIGH);
      digitalWrite(MOT_R_B, LOW);
    } else if(output > 0) {
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
    unsigned int finalOut = map(min(1000, absOutput), 0, 1000, 50, 600);
    analogWrite(MOT_L_P, finalOut);
    analogWrite(MOT_R_P, finalOut);
  }

}

/* I2C */

void i2cWrite(uint8_t address, uint8_t registerAddress, uint8_t data) {
  Wire.beginTransmission(address);
  Wire.write(registerAddress);
  Wire.write(data);
  Wire.endTransmission();
}
uint8_t* i2cRead(uint8_t address, uint8_t registerAddress, uint8_t nbytes) {
  Wire.beginTransmission(address);
  Wire.write(registerAddress);
  Wire.endTransmission();
  Wire.beginTransmission(address);
  Wire.requestFrom(address, nbytes);
  for (uint8_t i = 0; i < nbytes; i++)
    buffer[i] = Wire.read();
  Wire.endTransmission();
  return buffer;
}
int16_t readGyroX() { // This really measures the y-axis of the gyro
  uint8_t* data = i2cRead(gyroAddress, 0x1F, 2);
  return ((data[0] << 8) | data[1]);
}
int16_t readGyroY() { // This really measures the x-axis of the gyro
  uint8_t* data = i2cRead(gyroAddress, 0x1D, 2);
  return ((data[0] << 8) | data[1]);
}
double getXangle() {
  double accXval = (double)readAccX() - zeroValue[0];
  double accZval = (double)readAccZ() - zeroValue[2];
  double angle = (atan2(accXval, accZval) + PI) * RAD_TO_DEG;
  return angle;
}
double getYangle() {
  double accYval = (double)readAccY() - zeroValue[1];
  double accZval = (double)readAccZ() - zeroValue[2];
  double angle = (atan2(accYval, accZval) + PI) * RAD_TO_DEG;
  return angle;
}
int16_t readAccX() {
  uint8_t* data = i2cRead(adxlAddress, 0x32, 2);
  return (data[0] | (data[1] << 8));
}
int16_t readAccY() {
  uint8_t* data = i2cRead(adxlAddress, 0x34, 2);
  return (data[0] | (data[1] << 8));
}
int16_t readAccZ() {
  uint8_t* data = i2cRead(adxlAddress, 0x36, 2);
  return (data[0] | (data[1] << 8));
}

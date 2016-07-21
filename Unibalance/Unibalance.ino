#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>
#include <Wire.h>

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

// accelerometer
#define ADXL345_ADDRESS (0xA6 >> 1)
#define ADXL345_REGISTER_XLSB (0x32)
#define ADXL_REGISTER_PWRCTL (0x2D)
#define ADXL_PWRCTL_MEASURE (1 << 3)
#define ADXL_REGISTER_DATA_FORMAT (0x31)
#define ADXL_DATA_FORMAT_FULLRES (1 << 2)


#define Angle_offset 4
#define pi 3.14159265359
#define Gry_offset 0
#define Gyr_Gain 0.2

#define pidPeriod 0.001

//gyro
#define ITG3200_ADDRESS (0xD0 >> 1)
//request burst of 6 bytes from this address
#define ITG3200_REGISTER_XMSB (0x1D)
#define ITG3200_REGISTER_DLPF_FS (0x16)
#define ITG3200_42HZ (0x03)
#define ITG3200_FULLSCALE (0x03 << 3)
#define ITG3200_256HZ (0x00)

/* WIFI */
#define DEFAULTssid NAME
#define DEFAULTpsw "unibalance"

WiFiServer dbgServer(88);
WiFiClient dbgClient;

inline int min(int a, int b) { return ((a)<(b) ? (a) : (b)); }
void i2c_write(int address, byte reg, byte data);
void i2c_read(int address, byte reg, int count, byte* data);
void read_adxl345();
void read_itg3200();

short accelerometer_data[3];
short gyro_data[3];

double r_angle, omega, f_angle, rec_angle;
double errSum, prevErr;
double dErr;
double output;

unsigned long lastSample = 0;
unsigned long lastDebug = 0;
unsigned long lastPid = 0;

double kp = 400;
double kd = 60;
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
  
  byte check = 0;

  // init accelerometer  
  i2c_write(ADXL345_ADDRESS, ADXL_REGISTER_PWRCTL, ADXL_PWRCTL_MEASURE);
  i2c_write(ADXL345_ADDRESS, ADXL_REGISTER_DATA_FORMAT, ADXL_DATA_FORMAT_FULLRES);
  i2c_read(ADXL345_ADDRESS, ADXL_REGISTER_PWRCTL, 1, &check);
  if (dbgClient && dbgClient.connected())
    dbgClient.println((unsigned int)check);

  //init gyro
  //set the scale to "Full Scale"
  i2c_write(ITG3200_ADDRESS, ITG3200_REGISTER_DLPF_FS, ITG3200_FULLSCALE | ITG3200_256HZ);
  i2c_read(ITG3200_ADDRESS, ITG3200_REGISTER_DLPF_FS, 1, &check);
  if (dbgClient && dbgClient.connected())
    dbgClient.println((unsigned int)check);

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

  byte data = 0;

  read_adxl345();
  read_itg3200();

  r_angle = (atan2(accelerometer_data[2], accelerometer_data[1]) * 180.0 / pi + Angle_offset);

  if (abs(r_angle) > 60) {
    analogWrite(MOT_L_P, 0);
    analogWrite(MOT_R_P, 0);
    return;
  }
  
  omega = Gyr_Gain * gyro_data[0] +  Gry_offset;

  unsigned long nowMicro = micros();
  double dt_s = (double)(nowMicro - lastSample) * 0.000001;
  lastSample = nowMicro;

  float K = 0.5;
  float A = K / (K + dt_s * 1000);
  f_angle = A * (f_angle + omega * dt_s) + (1 - A) * r_angle;

  //test
  digitalWrite(LED_PIN, (accelerometer_data[2] + 2000) >= 0);

  if (millis() - lastDebug > 1000) {
    if (dbgClient && dbgClient.connected()) {
      //dbgClient.println(accelerometer_data[2] + 6000);
      /*dbgClient.println(output);
      dbgClient.println(prevErr);
      dbgClient.println(dErr);
      dbgClient.println("-------------");*/
    }
    lastDebug = millis();
  }

  unsigned long now = millis();
  double dt_p = (double)(now - lastPid) / 1000.0;
  
  if(dt_p >= pidPeriod){
    //double err = accelerometer_data[2] + 1500;
    double err = f_angle;
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
    unsigned int finalOut = map(min(1000, absOutput), 0, 1000, 80, 600);
    analogWrite(MOT_L_P, finalOut);
    analogWrite(MOT_R_P, finalOut);
  }

}

/* I2C */

void i2c_write(int address, byte reg, byte data) {
  // Send output register address
  Wire.beginTransmission(address);
  Wire.write(reg);
  // Connect to device and send byte
  Wire.write(data); // low byte
  Wire.endTransmission();
}

void i2c_read(int address, byte reg, int count, byte* data) {
  int i = 0;

  // Send input register address
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission();
  // Connect to device and request bytes
  Wire.beginTransmission(address);
  Wire.requestFrom(address, count);
  while (Wire.available()) { // slave may send less than requested
    char c = Wire.read(); // receive a byte as character
    data[i] = c;
    i++;
  }
  Wire.endTransmission();
}

void read_adxl345() {
  byte bytes[6];
  memset(bytes, 0, 6);

  //read 6 bytes from the ADXL345
  i2c_read(ADXL345_ADDRESS, ADXL345_REGISTER_XLSB, 6, bytes);

  //now unpack the bytes
  for (int i = 0; i < 3; ++i) {
    accelerometer_data[i] = bytes[2*i] | (bytes[2*i + 1] << 8);
  }
}

void read_itg3200() {
  byte bytes[6];
  memset(bytes,0,6);

  //read 6 bytes from the ITG3200
  i2c_read(ITG3200_ADDRESS, ITG3200_REGISTER_XMSB, 6, bytes);  //now unpack the bytes
  for (int i=0;i<3;++i) {
    gyro_data[i] = bytes[2*i + 1] | (bytes[2*i] << 8);
  }
}

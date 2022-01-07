#include <MadgwickAHRS.h>
#include <Wire.h>

#define MPU6050_ADDR 0x68
#define MPU6050_AX  0x3B
#define MPU6050_AY  0x3D
#define MPU6050_AZ  0x3F
#define MPU6050_TP  0x41    //  data not used
#define MPU6050_GX  0x43
#define MPU6050_GY  0x45
#define MPU6050_GZ  0x47

#define PI (3.14159265358979323846264338)

Madgwick MadgwickFilter;

const float lpf = 0.95;
const float ACC_X_OFS = 0.0f, ACC_Y_OFS = 0.0f, ACC_Z_OFS = 0.0f;
const float ACC_X_GAIN = 16384.0f, ACC_Y_GAIN = 16384.0f, ACC_Z_GAIN = 16384.0f;
const float GYRO_X_OFS = 0.0f, GYRO_Y_OFS = 0.0f, GYRO_Z_OFS = 0.0f;
const float GYRO_X_GAIN = 131.0f, GYRO_Y_GAIN = 131.0f, GYRO_Z_GAIN = 131.0f;

short int raw_accX, raw_accY, raw_accZ;
short int raw_Temp;
short int raw_gyroX, raw_gyroY, raw_gyroZ;

float accX = 0.0f, accY = 0.0f, accZ = 0.0f;
float gyroX = 0.0f, gyroY = 0.0f, gyroZ = 0.0f;
float pre_gyroX = 0.0f, pre_gyroY = 0.0f, pre_gyroZ = 0.0f;
float raw_roll = 0.0f, raw_pitch = 0.0f;
float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
float rollFromA = 0.0f, pitchFromA = 0.0f;

unsigned long before_time = 0;


void setup() {
  pinMode(10, INPUT_PULLUP); //リセットボタン
  Serial.begin(115200);
  Serial.printf("pitch,roll,yaw\n");
  Wire.begin();
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission();

  MadgwickFilter.begin(100);
}

void loop() {
  //  send start address
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_AX);
  Wire.endTransmission();
  //  request 14bytes (int16 x 7)
  Wire.requestFrom(MPU6050_ADDR, 14);
  //  get 14bytes
  raw_accX = Wire.read() << 8;  raw_accX |= Wire.read();
  raw_accY = Wire.read() << 8;  raw_accY |= Wire.read();
  raw_accZ = Wire.read() << 8;  raw_accZ |= Wire.read();
  raw_Temp = Wire.read() << 8;  raw_Temp |= Wire.read();  //  (Temp-12421)/340.0 [degC]
  raw_gyroX = Wire.read() << 8; raw_gyroX |= Wire.read();
  raw_gyroY = Wire.read() << 8; raw_gyroY |= Wire.read();
  raw_gyroZ = Wire.read() << 8; raw_gyroZ |= Wire.read();

  //Filter
  accX = lpf * accX + (1 - lpf) * (raw_accX - ACC_X_OFS) / ACC_X_GAIN;
  accY = lpf * accY + (1 - lpf) * (raw_accY - ACC_Y_OFS) / ACC_Y_GAIN;
  accZ = lpf * accZ + (1 - lpf) * (raw_accZ - ACC_Z_OFS) / ACC_Z_GAIN;
  gyroX = lpf * gyroX + (1 - lpf) * (raw_gyroX - GYRO_X_OFS) / GYRO_X_GAIN;
  gyroY = lpf * gyroY + (1 - lpf) * (raw_gyroY - GYRO_Y_OFS) / GYRO_Y_GAIN;
  gyroZ = lpf * gyroZ + (1 - lpf) * (raw_gyroZ - GYRO_Z_OFS) / GYRO_Z_GAIN;
  //Serial.printf("%f  %f  %f\n",accX ,accY ,accZ);

  //Calc
  MadgwickFilter.updateIMU(gyroX, gyroY, gyroZ, accX, accY, accZ);
  float pitch = MadgwickFilter.getPitch();
  float roll = MadgwickFilter.getRoll();
  float yaw = MadgwickFilter.getYaw();
  Serial.printf("%f,%f,%f\n",pitch,roll,yaw);
  


  /*
  double dt = (micros() - before_time) / 1000000.0f;
  before_time = micros();
  double gx = (gyroX + pre_gyroX) / 2;
  double gy = (gyroY + pre_gyroY) / 2;
  double gz = (gyroZ + pre_gyroZ) / 2;

  rollFromA = atan2(accY, accZ) / PI * 180; //重力加速度から計算
  pitchFromA = atan2(accX, sqrt(accY * accY + accZ * accZ)) / PI * 180; //重力加速度から計算

  raw_roll += gx * dt;
  raw_pitch += gy * dt;
  yaw += gz * dt;
  //roll += raw_pitch * sin(gz * dt * PI / 180);
  //pitch -= raw_roll * sin(gz * dt * PI / 180);

  //roll = roll*0.98 + rollFromA*0.02;
  //pitch = pitch*0.98 + pitchFromA*0.02;

  pre_gyroX = gyroX;
  pre_gyroY = gyroY;
  pre_gyroZ = gyroZ;

  //Reset
  if (digitalRead(10)) {
    Serial.println("");
    Serial.println("-------------------RESET!-------------------");
    roll = 0;
    pitch = 0;
    yaw = 0;
  }

  //Serial.printf("%f %f\n",rollFromA ,pitchFromA);
  Serial.printf("%f %f\n", roll , pitch);
  */
}

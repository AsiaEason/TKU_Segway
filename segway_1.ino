#include <Thread.h>
#include <ThreadController.h>
#include "HX711.h"
#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter
//======================================================================================================================================================
#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
#define SERIAL_PRINT false

#define RIGHT_PWM        3
#define RIGHT_EN         2
#define RIGHT_DIR        4
#define RIGHT_CURR_SPEED A1
#define LEFT_PWM         6
#define LEFT_EN          5
#define LEFT_DIR         7
#define LEFT_CURR_SPEED  A0

#define MIN_PWM          25.6
#define MAX_PWM          230.4
#define MAX_SPEED        60

#define DT_RF            53
#define SCK_RF           52
#define DT_RR            51
#define SCK_RR           50
#define DT_RM            49
#define SCK_RM           48
#define DT_LR            47
#define SCK_LR           46
#define DT_LF            45
#define SCK_LF           44
#define DT_LM            43
#define SCK_LM           42

#define scale_factor_RF  92.44
#define scale_factor_RR  98.15
#define scale_factor_LR  98.4
#define scale_factor_LF  91.5
#define scale_factor_LM  33
#define scale_factor_RM  66 

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

float Inclination = 0;
float Inclination_old = 0;
float ang_vel = 0;
float acc_old = 0;

float cmd_theda = 0;
float ref_theda = 0;
float err_theda = 0;
float err_theda_old = 0;
float div_err_theda = 0;
float int_err_theda = 0;
float cmd_vel = 0;
float ref_vel = 0;
float err_vel = 0;
float err_vel_old = 0;
float div_err_vel = 0;
float int_err_vel = 0;
float turn_vel = 0;
bool turn_mode = false;
bool balance_mode = false;

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//======================================================================================================================================================


HX711 scale_RF, scale_RR, scale_LR, scale_LF, scale_LM, scale_RM;
ThreadController control = ThreadController();
Thread* sensorThread1 = new Thread();
Thread* sensorThread2 = new Thread();
Thread* sensorThread3 = new Thread();
Thread* sensorThread4 = new Thread();
Thread* sensorThread5 = new Thread();
Thread* sensorThread6 = new Thread();
Thread* motionThread  = new Thread();
Thread* imuThread  = new Thread();
Thread* weightThread = new Thread();
float weight_RF, weight_RR, weight_LR, weight_LF, weight_LM, weight_RM;
float total_weight, left_weight, right_weight, front_weight, real_weight;
int a=0;
int b=0;
int c=0;
float left, right, curr_l, curr_r;
float l = 0.0;
float r = 0.0;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void setup() {
  Serial.begin(115200);
  Wire.begin();
#if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

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
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();

//=========================================================================================================
  sensorThread1->setInterval(200);
  sensorThread1->onRun(readWeight1);
  sensorThread2->setInterval(200);
  sensorThread2->onRun(readWeight2);
  sensorThread3->setInterval(200);
  sensorThread3->onRun(readWeight3);
  sensorThread4->setInterval(200);
  sensorThread4->onRun(readWeight4);
  sensorThread5->setInterval(200);
  sensorThread5->onRun(readWeight5);
  sensorThread6->setInterval(200);
  sensorThread6->onRun(readWeight6);
  motionThread->setInterval(5);
  motionThread->onRun(MotorControl);
  imuThread->setInterval(10);
  imuThread->onRun(Imu);
  weightThread->setInterval(20);
  weightThread->onRun(weightValue);

  scale_RF.begin(DT_RF, SCK_RF);
  scale_RR.begin(DT_RR, SCK_RR);
  scale_LR.begin(DT_LR, SCK_LR);
  scale_LF.begin(DT_LF, SCK_LF);
  scale_LM.begin(DT_LM, SCK_LM);
  scale_RM.begin(DT_RM, SCK_RM);
  
  scale_RF.set_scale(scale_factor_RF);       // 設定比例參數
  scale_RF.tare();               // 歸零
  scale_RR.set_scale(scale_factor_RR);       // 設定比例參數
  scale_RR.tare();               // 歸零
  scale_LR.set_scale(scale_factor_LR);       // 設定比例參數
  scale_LR.tare();               // 歸零
  scale_LF.set_scale(scale_factor_LF);       // 設定比例參數
  scale_LF.tare();               // 歸零
  scale_LM.set_scale(scale_factor_LM);       // 設定比例參數
  scale_LM.tare();               // 歸零
  scale_RM.set_scale(scale_factor_RM);       // 設定比例參數
  scale_RM.tare();               // 歸零

  // Right Motor Initial
  pinMode(RIGHT_PWM,  OUTPUT);
  digitalWrite(RIGHT_EN,   HIGH);
  digitalWrite(RIGHT_DIR,  1);
  analogWrite(RIGHT_PWM, MIN_PWM);


  // Left Motor Initial
  pinMode(LEFT_PWM,  OUTPUT);
  digitalWrite(LEFT_EN,   HIGH);
  digitalWrite(LEFT_DIR,  0);
  analogWrite(LEFT_PWM, MIN_PWM);
  
  control.add(sensorThread1);
  control.add(sensorThread2);
  control.add(sensorThread3);
  control.add(sensorThread4);
  control.add(sensorThread5);
  control.add(sensorThread6);
  control.add(motionThread);
  control.add(imuThread);
  control.add(weightThread);
}

void loop(){
  control.run();
}

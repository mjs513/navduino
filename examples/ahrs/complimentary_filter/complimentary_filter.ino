#include "Adafruit_LSM6DS33.h"
#include "Adafruit_LIS3MDL.h"
#include "navduino.h"
#include "attitude.h"




Adafruit_LSM6DS33 lsm6ds33;
sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t temp;
Adafruit_LIS3MDL lis3mdl;
sensors_event_t mag;
compFilt ahrs;




float gyro_x_bias = -0.045967611;
float gyro_y_bias = 0.092546083;
float gyro_z_bias = 0.0813977905;




void setup()
{
  Serial.begin(115200);

  if (!lsm6ds33.begin_I2C(0b1101011))
  {
    Serial.println("Failed to find LSM6DS33 chip");
    while (1);
  }

  lsm6ds33.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
  lsm6ds33.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
  lsm6ds33.setAccelDataRate(LSM6DS_RATE_104_HZ);
  lsm6ds33.setGyroDataRate(LSM6DS_RATE_104_HZ);

  if (! lis3mdl.begin_I2C(0b0011110))
  {
    Serial.println("Failed to find LIS3MDL chip");
    while (1);
  }

  lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
  lis3mdl.setIntThreshold(500);
  lis3mdl.configInterrupt(false, false, true, // enable z axis
                          true, // polarity
                          false, // don't latch
                          true); // enabled!
}




void loop()
{
  lsm6ds33.getEvent(&accel, &gyro, &temp);
  lis3mdl.getEvent(&mag);

  float ax = accel.acceleration.x;
  float ay = accel.acceleration.y;
  float az = accel.acceleration.z;
  
  float gx = (gyro.gyro.x + gyro_x_bias) * 180 / M_PI;
  float gy = (gyro.gyro.y + gyro_y_bias) * 180 / M_PI;
  float gz = (gyro.gyro.z + gyro_z_bias) * 180 / M_PI;
  
  float mx = mag.magnetic.x;
  float my = mag.magnetic.y;
  float mz = mag.magnetic.z;
  
  ahrs.updateIMU(ax, ay, az, gx, gy, gz);

  Serial.print("Raw:");
  Serial.print(ax, 9);
  Serial.print(',');
  Serial.print(ay, 9);
  Serial.print(',');
  Serial.print(az, 9);
  Serial.print(',');
  Serial.print(gx, 9);
  Serial.print(',');
  Serial.print(gy, 9);
  Serial.print(',');
  Serial.print(gz, 9);
  Serial.print(',');
  Serial.print(mx, 9);
  Serial.print(',');
  Serial.print(my, 9);
  Serial.print(',');
  Serial.print(mz, 9);
  Serial.print(',');
  Serial.print(ahrs.getRoll(), 9);
  Serial.print(',');
  Serial.print(ahrs.getPitch(), 9);
  Serial.print(',');
  Serial.print(ahrs.getYaw(), 9);
  Serial.println();
  
  delay(10);
}
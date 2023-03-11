#include "Arduino.h"
#include "navduino.h"
#include "attitude.h"




// https://stackoverflow.com/a/11498248/9860973
double baseIMU::wrapAngle(const double& angle)
{
  double out = fmod(angle + 180, 360);

  if (out < 0)
      out += 360;
  
  return out - 180;
}




double baseIMU::wrapYaw(const double& angle)
{
  return fmod(angle, 360);
}




void compFilt::setAlpha(const double& _alpha)
{
  if (_alpha > 1)
    alpha = 1;
  else if (_alpha < 0)
    alpha = 0;
  else
    alpha = _alpha;
}




// https://www.hackster.io/hibit/complementary-filter-and-relative-orientation-with-mpu9250-d4f79d
void compFilt::updateIMU(const double& _ax,
                         const double& _ay,
                         const double& _az,
                         const double& _gx,
                         const double& _gy,
                         const double& _gz,
                         const double& _timestamp)
{
  ax = _ax;
  ay = _ay;
  az = _az;
  gx = _gx;
  gy = _gy;
  gz = _gz;

  if (_timestamp >= 0)
    timestamp = _timestamp;
  else
    timestamp = micros() / 1000000.0;
  
  dt = timestamp - prevTimestamp;
  
  double accelPitch = atan2(ax, sqrt(pow(ay, 2) + pow(az, 2)));
  double accelRoll  = atan2(ay, sqrt(pow(ax, 2) + pow(az, 2)));
  
  double gyroPitch = pitch + gx * dt;
  double gyroRoll  = roll  + gy * dt;
  double gyroYaw   = yaw   + gz * dt;

  roll  = ((1 - alpha) * gyroRoll)  + (alpha * accelRoll);
  pitch = ((1 - alpha) * gyroPitch) + (alpha * accelPitch);
  yaw   = gyroYaw;

  roll  = wrapAngle(roll);
  pitch = wrapAngle(pitch);
  yaw   = wrapYaw(yaw);

  euler << roll,
           pitch,
           yaw;
  
  dcm = angle2dcm(euler,
                  DEGREES,
                  NED_TO_BODY,
                  321);
  quat = dcm2quat(dcm);

  prevTimestamp = timestamp;
}

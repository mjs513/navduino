#pragma once
#include "Arduino.h"
#include "navduino.h"




class baseIMU
{
public:
  double      getRoll()  { return roll;  };
  double      getPitch() { return pitch; };
  double      getYaw()   { return yaw;   };
  Vector3f    getEuler() { return euler; };
  Matrix3f    getDCM()   { return dcm;   };
  Quaternionf getQuat()  { return quat;  };
  double      getDt()    { return dt;    };




protected:
  double ax;
  double ay;
  double az;
  double gx;
  double gy;
  double gz;
  double mx;
  double my;
  double mz;
  double timestamp;
  double prevTimestamp;
  double dt;
  double roll;
  double pitch;
  double yaw;
  Vector3f euler;
  Matrix3f dcm;
  Quaternionf quat;




  double wrapAngle(const double& angle);
  double wrapYaw(const double& angle);
};




class compFilt : public baseIMU
{
public:
  void setAlpha(const double& _alpha = 0.02);
  void updateIMU(const double& _ax,
                 const double& _ay,
                 const double& _az,
                 const double& _gx,
                 const double& _gy,
                 const double& _gz,
                 const double& _timestamp = -1);




private:
  double alpha;
};

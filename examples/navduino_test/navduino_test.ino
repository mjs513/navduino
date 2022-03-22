#include "navduino.h"


void setup()
{
  Serial.begin(115200);
  while(!Serial);

  Vector3f euler;
  euler << 45,  // x - roll
           10,  // y - pitch
           -60; // z - yaw

  Matrix3f a2dcm = angle2dcm(euler, DEGREES);

  Serial.println("angle2dcm");
  printMat3f(a2dcm);
  Serial.println();

  Vector3f dcm2a = dcm2angle(a2dcm, DEGREES);

  Serial.println("dcm2angle");
  printVec3f(dcm2a);
  Serial.println();

  Quaternionf a2quat = angle2quat(euler, DEGREES);
  
  Serial.println("angle2quat");
  printQuatf(a2quat);
  Serial.println();

  Vector3f quat2a = quat2angle(a2quat, DEGREES);
  
  Serial.println("quat2angle");
  printVec3f(quat2a);
  Serial.println();
}


void loop()
{
  
}

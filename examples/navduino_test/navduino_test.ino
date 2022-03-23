#include "navduino.h"


void setup()
{
  Serial.begin(115200);
  while(!Serial);

  Vector3f euler;
  euler << 45, // x - roll
           10, // y - pitch
          -60; // z - yaw

  Matrix3f a2dcm = angle2dcm(euler, DEGREES, 321, NED_TO_BODY);

  Serial.println("angle2dcm");
  printMat3f(a2dcm);
  Serial.println();

  Vector3f dcm2a = dcm2angle(a2dcm, DEGREES, 321, NED_TO_BODY);

  Serial.println("dcm2angle");
  printVec3f(dcm2a);
  Serial.println();

  Quaternionf a2quat = angle2quat(euler, DEGREES, 321, NED_TO_BODY);
  
  Serial.println("angle2quat");
  printQuatf(a2quat);
  Serial.println();

  Vector3f quat2a = quat2angle(a2quat, DEGREES, 321, NED_TO_BODY);
  
  Serial.println("quat2angle");
  printVec3f(quat2a);
  Serial.println();

  Matrix3f q2dcm = quat2dcm(a2quat);
  
  Serial.println("quat2dcm");
  printMat3f(q2dcm);
  Serial.println();

  Quaternionf dcm2q = dcm2quat(q2dcm);
  
  Serial.println("dcm2quat");
  printQuatf(dcm2q);
  Serial.println();

  Vector2f erad = earthrad(20, DEGREES);
  
  Serial.println("earthrad");
  printVec2f(erad);
  Serial.println();

  Vector3f lla;
  lla << 20,
        -30,
         250;
  
  Vector3f vned;
  vned << 100,
          200,
         -100;

  Vector3f llar = llarate(vned, lla, DEGREES);
  
  Serial.println("llarate");
  printVec3f(llar);
  Serial.println();

  Vector3f erate = earthrate(20, DEGREES);
  
  Serial.println("earthrate");
  printVec3f(erate, 10);
  Serial.println();

  Vector3f nrate = navrate(vned, lla, DEGREES);
  
  Serial.println("navrate");
  printVec3f(nrate, 10);
  Serial.println();

  Vector3f l2ecef = lla2ecef(lla, DEGREES);
  
  Serial.println("lla2ecef");
  printVec3f(l2ecef);
  Serial.println();

  Vector3f ecef2l = ecef2lla(l2ecef, DEGREES);
  
  Serial.println("ecef2lla");
  printVec3f(ecef2l);
  Serial.println();

  Vector3f lla_ref;
  lla_ref << 20.01,
            -30.01,
             260;

  Vector3f ecef2n = ecef2ned(l2ecef, lla_ref, DEGREES);
  
  Serial.println("ecef2ned");
  printVec3f(ecef2n);
  Serial.println();

  Vector3f l2ned = lla2ned(ecef2l, lla_ref, DEGREES);
  
  Serial.println("lla2ned");
  printVec3f(l2ned);
  Serial.println();

  Vector3f ned3e = ned2ecef(l2ned, lla_ref, DEGREES);
  
  Serial.println("ned2ecef");
  printVec3f(ned3e);
  Serial.println();

  Vector3f ned3l = ned2lla(l2ned, lla_ref, DEGREES);
  
  Serial.println("ned2lla");
  printVec3f(ned3l);
  Serial.println();
  
  Vector3f w;
  w << 1,
       2,
       3;

  Matrix3f wk = skew(w);
  
  Serial.println("ned2lla");
  printMat3f(wk);
  Serial.println();
}


void loop()
{
  
}

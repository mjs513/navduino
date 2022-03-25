#include "navduino.h"




void setup()
{
  Serial.begin(115200);
  while (!Serial);

  if (test_angle2dcm())
    Serial.println("angle2dcm() passed");
  else
    Serial.println("angle2dcm() failed <-----");

  
  if (test_dcm2angle())
    Serial.println("dcm2angle() passed");
  else
    Serial.println("dcm2angle() failed <-----");

  
  if (test_angle2quat())
    Serial.println("angle2quat() passed");
  else
    Serial.println("angle2quat() failed <-----");

  
  if (test_quat2angle())
    Serial.println("quat2angle() passed");
  else
    Serial.println("quat2angle() failed <-----");

  
  if (test_quat2dcm())
    Serial.println("quat2dcm() passed");
  else
    Serial.println("quat2dcm() failed <-----");

  
  if (test_dcm2quat())
    Serial.println("dcm2quat() passed");
  else
    Serial.println("dcm2quat() failed <-----");

  
  if (test_earthGeoRad())
    Serial.println("earthGeoRad() passed");
  else
    Serial.println("earthGeoRad() failed <-----");

  
  if (test_earthAzimRad())
    Serial.println("earthAzimRad() passed");
  else
    Serial.println("earthAzimRad() failed <-----");

  
  if (test_earthRate())
    Serial.println("earthRate() passed");
  else
    Serial.println("earthRate() failed <-----");

  
  if (test_llaRate())
    Serial.println("llaRate() passed");
  else
    Serial.println("llaRate() failed <-----");

  
  if (test_navRate())
    Serial.println("navRate() passed");
  else
    Serial.println("navRate() failed <-----");

  
  if (test_lla2ecef())
    Serial.println("lla2ecef() passed");
  else
    Serial.println("lla2ecef() failed <-----");

  
  if (test_ecef2lla())
    Serial.println("ecef2lla() passed");
  else
    Serial.println("ecef2lla() failed <-----");
}




void loop()
{
  
}




bool test_angle2dcm()
{
  Vector3f euler;
  euler << 45, // x - roll  - degrees
           10, // y - pitch - degrees
          -60; // z - yaw   - degrees

  Matrix3f dcm = angle2dcm(euler, DEGREES, BODY_TO_NED, 321);

  Matrix3f truth(3, 3);
  truth << 0.4924039, 0.6737663, -0.5509785,
          -0.8528686, 0.2472160, -0.4598908,
          -0.1736482, 0.6963642,  0.6963642;

  return dcm.isApprox(truth);
}




bool test_dcm2angle()
{
  Matrix3f dcm(3, 3);
  dcm << 0.4924039, 0.6737663, -0.5509785,
        -0.8528686, 0.2472160, -0.4598908,
        -0.1736482, 0.6963642,  0.6963642;

  Vector3f euler = dcm2angle(dcm, DEGREES, BODY_TO_NED, 321);

  Vector3f truth;
  truth << 45, // x - roll  - degrees
           10, // y - pitch - degrees
          -60; // z - yaw   - degrees

  return euler.isApprox(truth);
}




bool test_angle2quat()
{
  Vector3f euler;
  euler << 45, // x - roll  - degrees
           10, // y - pitch - degrees
          -60; // z - yaw   - degrees

  Quaternionf quat = angle2quat(euler, DEGREES, BODY_TO_NED, 321);
  
  Vector4f test;
  test << quat.x(),
          quat.y(),
          quat.z(),
          quat.w();

  Vector4f truth;
  truth << 0.3704131, // x
          -0.12088,   // y
          -0.4890665, // y
           0.780382;  // w

  return test.isApprox(truth);
}




bool test_quat2angle()
{
  Quaternionf quat(0.780382,   // w
                   0.3704131,  // x
                  -0.12088,    // y
                  -0.4890665); // z
  
  Vector3f euler = quat2angle(quat, DEGREES, BODY_TO_NED, 321);
  
  Vector3f truth;
  truth << 45, // x - roll  - degrees
           10, // y - pitch - degrees
          -60; // z - yaw   - degrees
  
  return euler.isApprox(truth);
}




bool test_quat2dcm()
{
  Quaternionf quat(0.780382,   // w
                   0.3704131,  // x
                  -0.12088,    // y
                  -0.4890665); // z
  
  Matrix3f dcm = quat2dcm(quat);
  
  Matrix3f truth(3, 3);
  truth << 0.4924039, 0.6737663, -0.5509785,
          -0.8528686, 0.2472160, -0.4598908,
          -0.1736482, 0.6963642,  0.6963642;
  
  return dcm.isApprox(truth);
}




bool test_dcm2quat()
{
  Matrix3f dcm(3, 3);
  dcm << 0.4924039, 0.6737663, -0.5509785,
        -0.8528686, 0.2472160, -0.4598908,
        -0.1736482, 0.6963642,  0.6963642;
  
  Quaternionf quat = dcm2quat(dcm);
  
  Vector4f test;
  test << quat.x(),
          quat.y(),
          quat.z(),
          quat.w();

  Vector4f truth;
  truth << 0.3704131, // x
          -0.12088,   // y
          -0.4890665, // y
           0.780382;  // w
  
  return test.isApprox(truth);
}




bool test_earthGeoRad()
{
  float lat = 35; // Degrees
  
  float radius = earthGeoRad(lat, DEGREES);

  float truth = 6371141.23065; // m

  if (abs(truth - radius) <= 1e-10)
    return true;
  return false;
}




bool test_earthRad()
{
  float lat = 35; // Degrees
  
  Vector2f erad = earthRad(lat, DEGREES);

  Vector2f truth;
  truth << 6.3851722e6, // m
           6.3564267e6; // m

  return erad.isApprox(truth);
}




bool test_earthAzimRad()
{
  float lat     = 35; // Degrees
  float azimuth = 50; // Degrees off North
  
  float earad = earthAzimRad(lat, azimuth, DEGREES);

  float truth = 6.373264e6;
  
  if (abs(truth - earad) <= 1e-10)
    return true;
  return false;
}




bool test_earthRate()
{
  float lat = 35; // Degrees
  
  Vector3f erate = earthRate(lat, DEGREES);

  Vector3f truth;
  truth << 0.00005973351, // North - rad/s
           0,             // East  - rad/s
          -0.00004182585; // Down  - rad/s

  return erate.isApprox(truth);
}




bool test_llaRate()
{
  Vector3f vned;
  vned << 10, // North - m/s
          20, // East  - m/s
         -10; // Down  - m/s
  
  Vector3f lla;
  lla << 35, // lat - degrees
        -10, // lon - degrees
        250; // alt - m
  
  Vector3f lrate = llaRate(vned, lla, DEGREES);

  Vector3f truth;
  truth << 0.000090134801905, // lat - degrees/s
           0.00021907786142,  // lon - degrees/s
           10;                // alt - m/s
  
  return lrate.isApprox(truth);
}




bool test_navRate()
{
  Vector3f vned;
  vned << 10, // North - m/s
          20, // East  - m/s
         -10; // Down  - m/s
  
  Vector3f lla;
  lla << 35, // lat - degrees
        -10, // lon - degrees
        250; // alt - m
  
  Vector3f nrate = navRate(vned, lla, DEGREES);

  Vector3f truth;
  truth << 0.00017945808,  // ECEF x - degrees/s
          -0.000090134802, // ECEF y - degrees/s
          -0.0001256579;   // ECEF z - degrees/s
  
  return nrate.isApprox(truth);
}




bool test_lla2ecef()
{
  Vector3f lla;
  lla << 35, // lat - degrees
        -10, // lon - degrees
        250; // alt - m
  
  Vector3f ecef = lla2ecef(lla, DEGREES);

  Vector3f truth;
  truth << 5151166.58, // ECEF x
          -908289.65,  // ECEF y
           3638010.30; // ECEF z
  
  return ecef.isApprox(truth);
}




bool test_ecef2lla()
{
  Vector3f ecef;
  ecef << 5151166.58, // ECEF x
         -908289.65,  // ECEF y
          3638010.30; // ECEF z
  
  Vector3f lla = ecef2lla(ecef, DEGREES);

  Vector3f truth;
  truth << 35, // lat - degrees
          -10, // lon - degrees
          250; // alt - m

  Serial.println();
  Serial.println("LLA Computed:");
  printVec3f(lla);
  Serial.println();
  Serial.println("LLA Truth:");
  printVec3f(truth);
  Serial.println();
  
  return lla.isApprox(truth);
}

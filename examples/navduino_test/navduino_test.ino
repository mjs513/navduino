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

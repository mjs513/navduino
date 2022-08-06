#include "navduino.h"




const float THRESH = 1e-10;




vehicle_pose plane(0);
payload_pose gimbal(0, 0);
sensor_pose fpvCam(0, 0, 0);




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


  if (test_ecef2ned_dcm())
    Serial.println("ecef2ned_dcm() passed");
  else
    Serial.println("ecef2ned_dcm() failed <-----");

  
  if (test_poseMat())
    Serial.println("poseMat() passed");
  else
    Serial.println("poseMat() failed <-----");

  
  if (test_pose2dcm())
    Serial.println("pose2dcm() passed");
  else
    Serial.println("pose2dcm() failed <-----");

  
  if (test_pose2t())
    Serial.println("pose2t() passed");
  else
    Serial.println("pose2t() failed <-----");

  
  if (test_reversePoseMat())
    Serial.println("reversePoseMat() passed");
  else
    Serial.println("reversePoseMatt() failed <-----");

  
  if (test_transformPt_dcm_t())
    Serial.println("transformPt(dcm, t) passed");
  else
    Serial.println("transformPt(dcm, t) failed <-----");

  
  if (test_transformPt_pose())
    Serial.println("transformPt(pose) passed");
  else
    Serial.println("transformPt(pose) failed <-----");

  
  if (test_skew())
    Serial.println("skew passed");
  else
    Serial.println("skew failed <-----");

  
  if (test_bearingLla())
    Serial.println("bearingLla() passed");
  else
    Serial.println("bearingLla() failed <-----");

  
  if (test_distanceLla())
    Serial.println("distanceLla() passed");
  else
    Serial.println("distanceLla() failed <-----");

  
  if (test_LDAE2lla())
    Serial.println("LDAE2lla() passed");
  else
    Serial.println("LDAE2lla() failed <-----");

  examplePoseFunc();
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

  if (abs(truth - radius) <= THRESH)
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
  
  if (abs(truth - earad) <= THRESH)
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
  
  return lla.isApprox(truth, 1e-1);
}




bool test_ecef2ned_dcm()
{
  Vector3f lla;
  lla << 35, // lat - degrees
        -10, // lon - degrees
        250; // alt - m

  Matrix3f dcm = ecef2ned_dcm(lla, DEGREES);

  Matrix3f truth;
  truth << -0.56486252, 0.0996005,   0.81915204,
            0.17364818, 0.98480775,  0,
           -0.80670728, 0.14224426, -0.57357644;
  
  return dcm.isApprox(truth);
}




bool test_poseMat()
{
  Matrix3f dcm(3, 3);
  dcm << 0.4924039, 0.6737663, -0.5509785,
        -0.8528686, 0.2472160, -0.4598908,
        -0.1736482, 0.6963642,  0.6963642;

  Vector3f t;
  t << 1,
      10,
      -5;

  Matrix4f pose = poseMat(dcm, t);

  Matrix4f truth;
  truth << 0.4924039, 0.6737663, -0.5509785,  1,
          -0.8528686, 0.2472160, -0.4598908, 10,
          -0.1736482, 0.6963642,  0.6963642, -5,
           0.0      , 0.0      ,  0.0      ,  1;

  return pose.isApprox(truth);
}




bool test_pose2dcm()
{
  Matrix3f dcm(3, 3);
  dcm << 0.4924039, 0.6737663, -0.5509785,
        -0.8528686, 0.2472160, -0.4598908,
        -0.1736482, 0.6963642,  0.6963642;

  Vector3f t;
  t << 1,
      10,
      -5;

  Matrix3f testDcm = pose2dcm(poseMat(dcm, t));
  
  Matrix3f truth;
  truth << dcm;

  return testDcm.isApprox(truth);
}




bool test_pose2t()
{
  Matrix3f dcm(3, 3);
  dcm << 0.4924039, 0.6737663, -0.5509785,
        -0.8528686, 0.2472160, -0.4598908,
        -0.1736482, 0.6963642,  0.6963642;

  Vector3f t;
  t << 1,
      10,
      -5;

  Vector3f testTranslation = pose2t(poseMat(dcm, t));
  
  Vector3f truth;
  truth << t;

  return testTranslation.isApprox(truth);
}




bool test_reversePoseMat()
{
  Matrix3f dcm(3, 3);
  dcm << 0.4924039, 0.6737663, -0.5509785,
        -0.8528686, 0.2472160, -0.4598908,
        -0.1736482, 0.6963642,  0.6963642;

  Vector3f t;
  t << 1,
      10,
      -5;

  Matrix4f reversed = reversePoseMat(poseMat(dcm, t));
  
  Matrix4f truth;
  truth << 0.4924039, -0.8528686, -0.1736482, 7.1680411,
           0.6737663,  0.2472160,  0.6963642, 0.3358947,
          -0.5509785, -0.4598908,  0.6963642, 8.6317075,
           0.0,        0.0,        0.0,       1.0;

  return reversed.isApprox(truth);
}




bool test_transformPt_dcm_t()
{
  Matrix3f dcm(3, 3);
  dcm << 0.4924039, 0.6737663, -0.5509785,
        -0.8528686, 0.2472160, -0.4598908,
        -0.1736482, 0.6963642,  0.6963642;

  Vector3f t;
  t << 1,
      10,
      -5;
  
  Vector3f x;
  x << 5,
      -1,
       7;

  Vector3f new_x = transformPt(dcm, t, x);

  Vector3f truth;
  truth << -1.0685963,
            2.2692054,
           -1.6900558;

  return new_x.isApprox(truth);
}




bool test_transformPt_pose()
{
  Matrix4f pose;
  pose << 0.4924039, 0.6737663, -0.5509785,  1,
         -0.8528686, 0.2472160, -0.4598908, 10,
         -0.1736482, 0.6963642,  0.6963642, -5;
  
  Vector3f x;
  x << 5,
      -1,
       7;

  Vector3f new_x = transformPt(pose, x);

  Vector3f truth;
  truth << -1.0685963,
            2.2692054,
           -1.6900558;

  return new_x.isApprox(truth);
}




bool test_skew()
{
  Vector3f x;
  x << 5,
      -1,
       7;

  Matrix3f ssm = skew(x);

  Matrix3f truth;
  truth << 0, -7, -1,
           7,  0, -5,
           1,  5,  0;
  
  return ssm.isApprox(truth);
}




bool test_bearingLla()
{
  Vector3f lla_1;
  lla_1 << 39, // lat - degrees
           27, // lon - degrees
            0; // alt - m
  
  Vector3f lla_2;
  lla_2 << 39.001, // lat - degrees
           27.002, // lon - degrees
            0;     // alt - m
  
  float bearing = bearingLla(lla_1, lla_2, DEGREES);

  float truth = 57.2442;
  
  if (abs(truth - bearing) <= THRESH)
    return true;
  return false;
}




bool test_distanceLla()
{
  Vector3f lla_1;
  lla_1 << 39, // lat - degrees
           27, // lon - degrees
            0; // alt - m
  
  Vector3f lla_2;
  lla_2 << 39.001, // lat - degrees
           27.002, // lon - degrees
            0;     // alt - m
  
  float dist = distanceLla(lla_1, lla_2, DEGREES);

  float truth = 205.5;
  
  if (abs(truth - dist) <= THRESH)
    return true;
  return false;
}




bool test_LDAE2lla()
{
  Vector3f lla;
  lla << 39, // lat - degrees
         27, // lon - degrees
          0; // alt - m

  float dist      = 1000; // m
  float azimuth   = 45;   // degrees
  float elevation = 0;    // degrees
  
  Vector3f dest = LDAE2lla(lla, dist, azimuth, elevation, DEGREES);
  Serial.println();
  Serial.println("LLA Computed:");
  printVec3f(dest);

  Vector3f truth;
  truth << 39.00638889, // lat - degrees
           29.00805556, // lon - degrees
            0;          // alt - m
  
  return dest.isApprox(truth);
}




void examplePoseFunc()
{
  Vector3f LatLonAlt;
  LatLonAlt << 48.858370,
                2.294481,
               34.000000;
               
  Vector3f planeAngles;
  planeAngles << 10.0,
                 -5.5,
                  2.0;
                  
  Matrix3f planeDCM = angle2dcm(planeAngles);
  
  plane.update_loc_lla(LatLonAlt);
  plane.update_dcm(planeDCM);

  
  Vector3f gimbalTranslation;
  gimbalTranslation << 0.5,
                       1.5,
                      -0.2;

  Vector3f gimbalAngles;
  gimbalAngles << 45.0,
                 -14.2,
                   0.1;
                    
  Matrix3f gimbalDCM = angle2dcm(gimbalAngles);

  gimbal.update_v_t_v_p(gimbalTranslation);
  gimbal.update_p_R_v(gimbalDCM);


  Vector3f fpvCamTranslation;
  fpvCamTranslation << 0.01,
                      -0.04,
                      -0.20;

  Vector3f fpvCamAngles;
  fpvCamAngles << -1.2,
                   2.0,
                   0.1;
                    
  Matrix3f fpvCamDCM = angle2dcm(fpvCamAngles);

  fpvCam.update_p_t_p_s(fpvCamTranslation);
  fpvCam.update_s_R_p(fpvCamDCM);


  Vector3f sensor_x;
  sensor_x << 0.3,
             20.0,
             -3.5;

  Vector3f plane_x = sensor2vehicle(gimbal, fpvCam, sensor_x);


  Serial.println();
  Serial.println();
  Serial.println("Point sensed in the FPV camera's xyz body frame:");
  printVec3f(sensor_x);
  Serial.println();
  Serial.println("The same point transformed into the airplane's xyz body frame:");
  printVec3f(plane_x);
  Serial.println();
  Serial.println();
}

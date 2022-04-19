#pragma once
#include "Arduino.h"
#include "eigen.h"
#include <Eigen/LU>
#include <Eigen/Geometry>




#define deg2rad(angle) (angle * M_PI / 180.0)
#define rad2deg(angle) (angle * 180.0 / M_PI)
#define wrapToPi(e)    (fmod(e + M_PI, 2 * M_PI) - M_PI)




const bool RADIANS = true;
const bool DEGREES = false;

const bool NED_TO_BODY = true;
const bool BODY_TO_NED = false;

const int dRx = 0; // w.r.t. rotation around the x axis
const int dRy = 1; // w.r.t. rotation around the y axis
const int dRz = 2; // w.r.t. rotation around the z axis
const int dtx = 3; // w.r.t. translation around the x axis
const int dty = 4; // w.r.t. translation around the y axis
const int dtz = 5; // w.r.t. translation around the z axis




using namespace Eigen;




// Rotation conversions
Matrix3f angle2dcm(const Vector3f& angles, const bool& angle_unit = DEGREES, const bool& NED_to_body = true, const int& rotation_sequence = 321);
Vector3f dcm2angle(const Matrix3f& dcm, const bool& angle_unit = DEGREES, const bool& NED_to_body = true, const int& rotation_sequence = 321);
Quaternionf angle2quat(const Vector3f& angles, const bool& angle_unit = DEGREES, const bool& NED_to_body = true, const int& rotation_sequence = 321);
Vector3f quat2angle(const Quaternionf& quat, const bool& angle_unit = DEGREES, const bool& NED_to_body = true, const int& rotation_sequence = 321);
Matrix3f quat2dcm(const Quaternionf& quat);
Quaternionf dcm2quat(const Matrix3f& C);

// Earth radius calculations
float earthGeoRad(const float& _lat, const bool& angle_unit = DEGREES);
Vector2f earthRad(const float& _lat, const bool& angle_unit = DEGREES);
float earthAzimRad(const float& lat, const float& _azimuth, const bool& angle_unit = DEGREES);

// Frame angular rate calculations
Vector3f earthRate(const float& _lat, const bool& angle_unit = DEGREES);
Vector3f llaRate(const Vector3f& vned, const Vector3f& lla, const bool& angle_unit = DEGREES);
Vector3f navRate(const Vector3f& vned, const Vector3f& lla, const bool& angle_unit = DEGREES);

// Frame conversions
Vector3f lla2ecef(const Vector3f& lla, const bool& angle_unit = DEGREES);
Vector3f ecef2lla(const Vector3f& ecef, const bool& angle_unit = DEGREES);
Matrix3f ecef2ned_dcm(const Vector3f& lla, const bool& angle_unit = DEGREES);
Vector3f ecef2ned(const Vector3f& ecef, const Vector3f& lla_ref, const bool& angle_unit = DEGREES);
Vector3f lla2ned(const Vector3f& lla, const Vector3f& lla_ref, const bool& angle_unit = DEGREES);
Vector3f ned2ecef(const Vector3f& ned, const Vector3f& lla_ref, const bool& angle_unit = DEGREES);
Vector3f ned2lla(const Vector3f& ned, const Vector3f& lla_ref, const bool& angle_unit = DEGREES);

// Pose functions
Matrix4f poseMat(const Matrix3f& dcm, const Vector3f& t);
Matrix3f pose2dcm(const Matrix4f& poseMatrix);
Vector3f pose2t(const Matrix4f& poseMatrix);
Matrix4f reversePoseMat(const Matrix4f& poseMatrix);
Matrix4f poseMatDeriv(const Matrix4f& poseMatrix, const int& derivType);
Vector3f transformPt(const Matrix3f& dcm, const Vector3f& t, const Vector3f& x);
Vector3f transformPt(const Matrix4f& poseMatrix, const Vector3f& x);

// Linear algebra
Matrix3f skew(const Vector3f& w);

// Bearing calculations
float bearingLla(const Vector3f& lla_1, const Vector3f& lla_2, const bool& angle_unit = DEGREES);
float bearingNed(const Vector3f& ned_1, const Vector3f& ned_2);

// Distance calculations
double distanceLla(const Vector3f& lla_1, const Vector3f& lla_2, const bool& angle_unit = DEGREES);
float distanceNed(const Vector3f& ned_1, const Vector3f& ned_2);
float distanceNedHoriz(const Vector3f& ned_1, const Vector3f& ned_2);
float distanceNedVert(const Vector3f& ned_1, const Vector3f& ned_2);
float distanceEcef(const Vector3f& ecef_1, const Vector3f& ecef_2);

// Elevation calculations
float elevationLla(const Vector3f& lla_1, const Vector3f& lla_2, const bool& angle_unit = DEGREES);
float elevationNed(const Vector3f& ned_1, const Vector3f& ned_2);

// Relative coordinate calculations
Vector3f LDAE2lla(const Vector3f& lla, const float& dist, const float& _azimuth, const float& _elevation = 0, const bool& angle_unit = DEGREES);
Vector3f NDAE2ned(const Vector3f& ned, const float& dist, const float& _azimuth, const float& _elevation = 0, const bool& angle_unit = DEGREES);

// Eigen object printing functions
void printVec2f(const Vector2f& vec, const int& p = 5, Stream& stream = Serial);
void printVec3f(const Vector3f& vec, const int& p = 5, Stream& stream = Serial);
void printVec4f(const Vector4f& vec, const int& p = 5, Stream& stream = Serial);
void printQuatf(const Quaternionf& quat, const int& p = 5, Stream& stream = Serial);
void printMat3f(const Matrix3f& mat, const int& p = 5, Stream& stream = Serial);
void printMat4f(const Matrix4f& mat, const int& p = 5, Stream& stream = Serial);

// Constrain functions
float float_constrain(const float& input, const float& min, const float& max);
double double_constrain(const double& input, const double& min, const double& max);

// Map functions
float float_map(const float& x, const float& in_min, const float& in_max, const float& out_min, const float& out_max);
double double_map(const double& x, const double& in_min, const double& in_max, const double& out_min, const double& out_max);

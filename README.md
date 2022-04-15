# navduino
[![GitHub version](https://badge.fury.io/gh/PowerBroker2%2Fnavduino.svg)](https://badge.fury.io/gh/PowerBroker2%2Fnavduino) [![arduino-library-badge](https://www.ardu-badge.com/badge/navduino.svg?)](https://www.ardu-badge.com/navduino)<br /><br />
Arduino library for basic aerial navigation functions used for

* [Euler angles](https://en.wikipedia.org/wiki/Euler_angles)
* [Direction cosine matrices](https://en.wikipedia.org/wiki/Rotation_matrix)
* [Quaternions](https://eater.net/quaternions)
* [Earth radii calculations](https://en.wikipedia.org/wiki/Earth_radius)
* Earth rotation rate calculations
* Frame conversions
  *  [Latitude-Longitude-Altitude (LLA)](https://en.wikipedia.org/wiki/Geographic_coordinate_system)
  *  [North-East-Down (NED)](https://en.wikipedia.org/wiki/Local_tangent_plane_coordinates)
  *  [Earth Centered Earth Fixed (ECEF)](https://en.wikipedia.org/wiki/Earth-centered,_Earth-fixed_coordinate_system)
  *  [Affine/Pose transforms (conversions between two non-colocated cartesian coordinate frames)](https://en.wikipedia.org/wiki/Affine_transformation)
* [Distance and bearing calculations between 2 coordinates](http://www.movable-type.co.uk/scripts/latlong.html)
* [Calculating a new coordinate based on a reference coordinate (i.e. given a LLA coordinate, great circle distance, azimuth, and elevation angle, find the resulting LLA coordinate)](http://www.movable-type.co.uk/scripts/latlong.html)

# API:
```C++
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
Matrix4f reversePoseMat(const Matrix4f& poseMatrix);
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

// Constrain functions
float float_constrain(const float& input, const float& min, const float& max);
double double_constrain(const double& input, const double& min, const double& max);

// Map functions
float float_map(const float& x, const float& in_min, const float& in_max, const float& out_min, const float& out_max);
double double_map(const double& x, const double& in_min, const double& in_max, const double& out_min, const double& out_max);
```

# Credit:
Navduino is based on the Python library [NavPy](https://github.com/NavPy/NavPy).

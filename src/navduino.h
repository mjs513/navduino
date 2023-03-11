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
Quaternionf dcm2quat(const Matrix3f& dcm);
Matrix3f vec2dcm(const Vector3f& vec);
Vector3f dcm2vec(const Matrix3f& dcm);
Quaternionf vec2quat(const Vector3f& vec);
Vector3f quat2vec(const Quaternionf& quat);
Vector3f vec2angle(const Vector3f& vec, const bool& angle_unit = DEGREES, const bool& NED_to_body = true, const int& rotation_sequence = 321);
Vector3f angle2vec(const Vector3f& angles, const bool& angle_unit = DEGREES, const bool& NED_to_body = true, const int& rotation_sequence = 321);

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




class vehicle_pose
{
public:
	vehicle_pose(const byte& id) { _vehicleID = id; };

	byte vehicleID() { return _vehicleID; };

	Vector3f e_t_e_v() { return _e_t_e_v; };
	Vector3f v_t_v_e() { return -_e_t_e_v; };
	Matrix3f n_R_e() { return _n_R_e; };
	Matrix3f e_R_n() { return _n_R_e.transpose(); };
	Matrix3f v_R_n() { return _v_R_n; };
	Matrix3f n_R_v() { return _v_R_n.transpose(); };
	Matrix4f n_P_e() { return _n_P_e; };
	Matrix4f e_P_n() { return reversePoseMat(_n_P_e); };
	Matrix4f v_P_e() { return _v_P_e; };
	Matrix4f e_P_v() { return reversePoseMat(_v_P_e); };

	Vector3f lla() { return _lla; };
	Vector3f ecef() { return lla2ecef(_lla, DEGREES); };
	Vector3f euler() { return dcm2angle(_v_R_n, DEGREES, NED_TO_BODY, 321); };
	Quaternionf quat() { return dcm2quat(_v_R_n); };

	Vector3f body2ned(const Vector3f& x) { return _v_R_n.transpose() * x; };
	Vector3f ned2body(const Vector3f& x) { return _v_R_n * x; };
	Vector3f body2ecef(const Vector3f& x) { return transformPt(e_P_v(), x); };
	Vector3f ecef2body(const Vector3f& x) { return transformPt(_v_P_e, x); };
	Vector3f body2lla(const Vector3f& x) { return ned2lla(_v_R_n.transpose() * x, _lla, DEGREES); };
	Vector3f lla2body(const Vector3f& x) { return _v_R_n * lla2ned(x, _lla, DEGREES); };

	void update_dcm(const Matrix3f& _v_R_n_);
	void update_loc_lla(const Vector3f& _lla_, const bool& angle_unit = DEGREES);
	void update_loc_ecef(const Vector3f& _ecef_);




private:
	byte _vehicleID;

	Vector3f _lla; // dd, dd, m

	Vector3f _e_t_e_v; // translation vector (in m) from ECEF frame to vehicle's local level/NED frame in the ECEF frame
	Matrix3f _n_R_e;   // dcm from ECEF frame to vehicle's local level/NED frame
	Matrix3f _v_R_n;   // dcm from vehicle's local level/NED frame to vehicle's body frame
	Matrix4f _n_P_e;   // pose matrix that maps points from the ECEF frame to the vehicle's local level/NED frame https://en.wikipedia.org/wiki/Affine_transformation
	Matrix4f _v_P_e;   // pose matrix that maps points from the ECEF frame to the vehicle's body frame https://en.wikipedia.org/wiki/Affine_transformation
};




class payload_pose
{
public:
	payload_pose(const byte& vid, const byte& pid) { _vehicleID = vid; _payloadID = pid; };

	byte vehicleID() { return _vehicleID; };
	byte payloadID() { return _payloadID; };

	Vector3f v_t_v_p() { return _v_t_v_p; };
	Vector3f p_t_p_v() { return -_v_t_v_p; };
	Matrix3f p_R_v() { return _p_R_v; };
	Matrix3f v_R_p() { return _p_R_v.transpose(); };
	Matrix4f p_P_v() { return _p_P_v; };
	Matrix4f v_P_p() { return reversePoseMat(_p_P_v); };

	void update_v_t_v_p(const Vector3f& _v_t_v_p_);
	void update_p_t_p_v(const Vector3f& _p_t_p_v_);
	void update_p_R_v(const Matrix3f& _p_R_v_);
	void update_v_R_p(const Matrix3f& _v_R_p_);
	void update_p_P_v(const Matrix4f& _p_P_v_);
	void update_v_P_p(const Matrix4f& _v_P_p_);




private:
	byte _vehicleID;
	byte _payloadID;

	Vector3f _v_t_v_p; // translation vector (in m) from vehicle CG to payload in the vehicle's body frame
	Matrix3f _p_R_v;   // dcm from vehicle's body frame to payload's body frame
	Matrix4f _p_P_v;   // pose matrix that maps points from the vehicle's body frame to the payload's body frame https://en.wikipedia.org/wiki/Affine_transformation
};




class sensor_pose
{
public:
	sensor_pose(const byte& vid, const byte& pid, const byte& sid) { _vehicleID = vid; _payloadID = pid; _sensorID = sid; };

	byte sensorID() { return _sensorID; };

	Vector3f p_t_p_s() { return _p_t_p_s; };
	Vector3f s_t_s_p() { return -_p_t_p_s; };
	Matrix3f s_R_p() { return _s_R_p; };
	Matrix3f p_R_s() { return _s_R_p.transpose(); };
	Matrix4f s_P_p() { return _s_P_p; };
	Matrix4f p_P_s() { return reversePoseMat(_s_P_p); };

	void update_p_t_p_s(const Vector3f& _p_t_p_s_);
	void update_s_t_s_p(const Vector3f& _s_t_s_p_);
	void update_s_R_p(const Matrix3f& _s_R_p_);
	void update_p_R_s(const Matrix3f& _p_R_s_);
	void update_s_P_p(const Matrix4f& _s_P_p_);
	void update_p_P_s(const Matrix4f& _p_P_s_);




private:
	byte _vehicleID;
	byte _payloadID;
	byte _sensorID;

	Vector3f _p_t_p_s; // translation vector (in m) from payload to sensor in the payload's body frame
	Matrix3f _s_R_p;   // dcm from payload's body frame to sensor's body frame
	Matrix4f _s_P_p;   // pose matrix that maps points from the payload's body frame to the sensor's body frame https://en.wikipedia.org/wiki/Affine_transformation
};




Vector3f payload2vehicle(payload_pose& pay, const Vector3f& x);
Vector3f vehicle2payload(payload_pose& pay, const Vector3f& x);
Vector3f sensor2vehicle(payload_pose& pay, sensor_pose& sen, const Vector3f& x);
Vector3f vehicle2sensor(payload_pose& pay, sensor_pose& sen, const Vector3f& x);
Vector3f sensor2payload(payload_pose& pay, sensor_pose& sen, const Vector3f& x);
Vector3f payload2sensor(payload_pose& pay, sensor_pose& sen, const Vector3f& x);

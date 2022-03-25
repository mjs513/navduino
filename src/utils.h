#pragma once
#include "Arduino.h"
#include "math.h"
#include "wgs84.h"
#include "navduino.h"
#include "eigen.h"
#include <Eigen/Geometry>




using namespace Eigen;




#define deg2rad(angle) (angle * M_PI / 180.0)
#define rad2deg(angle) (angle * 180.0 / M_PI)
#define wrapToPi(e)    (fmod(e + M_PI, 2 * M_PI) - M_PI)




const bool RADIANS = true;
const bool DEGREES = false;

const bool NED_TO_BODY = true;
const bool BODY_TO_NED = false;




void printVec2f(const Vector2f& vec, const int& p = 5)
{
	Serial.println(vec(0), p);
	Serial.println(vec(1), p);
}




void printVec3f(const Vector3f& vec, const int& p = 5)
{
	Serial.println(vec(0), p);
	Serial.println(vec(1), p);
	Serial.println(vec(2), p);
}




void printVec4f(const Vector4f& vec, const int& p = 5)
{
	Serial.println(vec(0), p);
	Serial.println(vec(1), p);
	Serial.println(vec(2), p);
	Serial.println(vec(3), p);
}




void printQuatf(const Quaternionf& quat, const int& p = 5)
{
	Serial.println(quat.x(), p);
	Serial.println(quat.y(), p);
	Serial.println(quat.z(), p);
	Serial.println(quat.w(), p);
}




void printMat3f(const Matrix3f& mat, const int& p = 5)
{
	Serial.print(mat(0, 0), p); Serial.print(", "); Serial.print(mat(0, 1), p); Serial.print(", "); Serial.println(mat(0, 2), p);
	Serial.print(mat(1, 0), p); Serial.print(", "); Serial.print(mat(1, 1), p); Serial.print(", "); Serial.println(mat(1, 2), p);
	Serial.print(mat(2, 0), p); Serial.print(", "); Serial.print(mat(2, 1), p); Serial.print(", "); Serial.println(mat(2, 2), p);
}




float float_constrain(const float& input, const float& min, const float& max)
{
	if (input > max)
		return max;
	else if (input < min)
		return min;
	else
		return input;
}




double double_constrain(const double& input, const double& min, const double& max)
{
	if (input > max)
		return max;
	else if (input < min)
		return min;
	else
		return input;
}




float float_map(const float& x, const float& in_min, const float& in_max, const float& out_min, const float& out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}




double double_map(const double& x, const double& in_min, const double& in_max, const double& out_min, const double& out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
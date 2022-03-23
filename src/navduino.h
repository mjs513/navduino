#pragma once
#include "Arduino.h"
#include "math.h"
#include "utils.h"
#include "wgs84.h"
#include "eigen.h"
#include <Eigen/LU>
#include <Eigen/Geometry>


using namespace Eigen;


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


/*
This function converts Euler Angle into Direction Cosine Matrix(DCM).
*/
Matrix3f angle2dcm(const Vector3f& angles, const bool& unit_rad = true, const int& rotation_sequence = 321, const bool& NED_to_body = true)
{
    Matrix3f output(3, 3);

    Matrix3f R1(3, 3);
    Matrix3f R2(3, 3);
    Matrix3f R3(3, 3);

    float roll;
    float pitch;
    float yaw;

    if (!unit_rad)
    {
        roll  = deg2rad(angles(0));
        pitch = deg2rad(angles(1));
        yaw   = deg2rad(angles(2));
    }
    else
    {
        roll  = angles(0);
        pitch = angles(1);
        yaw   = angles(2);
    }

    R1 << 1,          0,         0,
          0,  cos(roll), sin(roll),
          0, -sin(roll), cos(roll);

    R2 << cos(pitch), 0, -sin(pitch),
                   0, 1,           0,
          sin(pitch), 0,  cos(pitch);

    R3 << cos(yaw), sin(yaw), 0,
         -sin(yaw), cos(yaw), 0,
                 0,        0, 1;

    // Note that multiplication of the matrices are opposite to the
    // rotation sequence due to how matrix multiplication works
    if (rotation_sequence == 321)
        output = R1 * R2 * R3;
    else if (rotation_sequence == 312)
        output = R2 * R1 * R3;
    else if (rotation_sequence == 231)
        output = R1 * R3 * R2;
    else if (rotation_sequence == 213)
        output = R3 * R1 * R2;
    else if (rotation_sequence == 132)
        output = R2 * R3 * R1;
    else if (rotation_sequence == 123)
        output = R3 * R2 * R1;
    else
        output = R1 * R2 * R3;

    if (!NED_to_body)
        return output.transpose();
    
    return output;
}


/*
This function converts a Direction Cosine Matrix(DCM) into the three
rotation angles.
*/
Vector3f dcm2angle(const Matrix3f& dcm, const bool& unit_rad = true, const int& rotation_sequence = 321, const bool& NED_to_body = true)
{
    Vector3f angles;

    if (rotation_sequence == 321)
    {
        if (NED_to_body)
        {
            angles << atan2(dcm(1, 2), dcm(2, 2)), // Roll
                     -asin(dcm(0, 2)),             // Pitch
                      atan2(dcm(0, 1), dcm(0, 0)); // Yaw
        }
        else
        {
            angles << atan2(dcm(2, 1), dcm(2, 2)), // Roll
                     -asin(dcm(2, 0)),             // Pitch
                      atan2(dcm(1, 0), dcm(0, 0)); // Yaw
        }
    }

    if (!unit_rad)
    {
        angles(0) = rad2deg(angles(0));
        angles(1) = rad2deg(angles(1));
        angles(2) = rad2deg(angles(2));
    }

    return angles;
}


/*
Convert a sequence of rotation angles to an equivalent unit quaternion
*/
Quaternionf angle2quat(const Vector3f& angles, const bool& unit_rad = true, const int& rotation_sequence = 321, const bool& NED_to_body = true)
{
    Quaternionf quat(angle2dcm(angles, unit_rad, rotation_sequence, NED_to_body));
    return quat;
}


/*
Convert a unit quaternion to the equivalent sequence of angles of rotation
about the rotation_sequence axes.
*/
Vector3f quat2angle(const Quaternionf& quat, const bool& unit_rad = true, const int& rotation_sequence = 321, const bool& NED_to_body = true)
{
    return dcm2angle(quat.toRotationMatrix(), unit_rad, rotation_sequence, NED_to_body);
}


/*
Convert a single unit quaternion to one DCM
*/
Matrix3f quat2dcm(const Quaternionf& quat)
{
    return quat.toRotationMatrix();
}


/*
Convert a DCM to a unit quaternion
*/
Quaternionf dcm2quat(const Matrix3f& C)
{
    Quaternionf quat(C);
    return quat;
}


/*
Calculate radius of curvature in the prime vertical(East - West) and
meridian(North - South) at a given latitude.

https://en.wikipedia.org/wiki/Earth_radius
*/
Vector2f earthrad(const float& lat, const bool& unit_rad = false)
{
    float _lat = lat;

    if (!unit_rad)
        _lat = deg2rad(_lat);

    float R_N = a / sqrt(1 - (ecc_sqrd * pow(sin(_lat), 2)));
    float R_M = (a * (1 - ecc_sqrd)) / pow(1 - (ecc_sqrd * pow(sin(_lat), 2)), 1.5);

    Vector2f out;
    out << R_N, // Earth's prime-vertical radius of curvature
           R_M; // Earth's meridional radius of curvature

    return out;
}


/*
Calculate the earth rotation rate resolved on NED axes given lat.
*/
Vector3f earthrate(const float& lat, const bool& unit_rad = false)
{
    float _lat = lat;

    if (!unit_rad)
        _lat = deg2rad(_lat);

    Vector3f e;
    e << omega_E * cos(_lat),
        -omega_E * sin(_lat),
        0;

    return e;
}


/*
Calculate Latitude, Longitude, Altitude Rate given locally tangent velocity
*/
Vector3f llarate(const Vector3f& vned, const Vector3f& lla, const bool& unit_rad = false)
{
    float VN = vned(0);
    float VE = vned(1);
    float VD = vned(2);

    float lat = lla(0);
    float alt = lla(2);

    Vector2f eradvec = earthrad(lat, unit_rad);
    float Rew = eradvec(0);
    float Rns = eradvec(1);

    Vector3f lla_dot;

    if (unit_rad)
    {
        lla_dot << VN / (Rns + alt),
            VE / (Rew + alt) / cos(deg2rad(lat)),
            -VD;
    }
    else
    {
        lla_dot << rad2deg(VN / (Rns + alt)),
            rad2deg(VE / (Rew + alt) / cos(deg2rad(lat))),
            -VD;
    }

    return lla_dot;
}


/*
Calculate navigation / transport rate given VN, VE, VD, lat, and alt.
Navigation / transport rate is the angular velocity of the NED frame relative
to the earth ECEF frame.
*/
Vector3f navrate(const Vector3f& vned, const Vector3f& lla, const bool& unit_rad = false)
{
    float VN = vned(0);
    float VE = vned(1);

    float lat = lla(0);
    float alt = lla(2);

    if (!unit_rad)
        lat = deg2rad(lat);

    Vector2f eradvec = earthrad(lat, unit_rad);
    float Rew = eradvec(0);
    float Rns = eradvec(1);

    Vector3f rho;

    rho << VE / (Rew + alt),
          -VN / (Rns + alt),
          -VE * tan(lat) / (Rew + alt);

    return rho;
}


/*
Convert Latitude, Longitude, Altitude, to ECEF position
*/
Vector3f lla2ecef(const Vector3f& lla, const bool& unit_rad = false)
{
    float lat = lla(0);
    float lon = lla(1);
    float alt = lla(2);

    Vector2f eradvec = earthrad(lat, unit_rad);
    float Rew = eradvec(0);

    if (!unit_rad)
    {
        lat = deg2rad(lat);
        lon = deg2rad(lon);
    }

    Vector3f ecef;

    ecef << (Rew + alt) * cos(lat) * cos(lon),
            (Rew + alt) * cos(lat) * sin(lon),
            ((1 - ecc_sqrd) * Rew + alt) * sin(lat);

    return ecef;
}


/*
Calculate the Latitude, Longitudeand Altitude of a point located on earth
given the ECEF Coordinates.

https://en.wikipedia.org/wiki/Geographic_coordinate_conversion
*/
Vector3f ecef2lla(const Vector3f& ecef, const bool& unit_rad = false)
{
    float x = ecef(0);
    float y = ecef(1);
    float z = ecef(2);

    float x_sqrd = pow(x, 2);
    float y_sqrd = pow(y, 2);
    float z_sqrd = pow(z, 2);

    float lon = atan2(y, x);

    float p      = sqrt(x_sqrd + y_sqrd);
    float p_sqrd = pow(p, 2);
    float F      = 54 * b_sqrd * z_sqrd;
    float G      = p_sqrd + ((1 - ecc_sqrd) * z_sqrd) - (ecc_sqrd * (a_sqrd - b_sqrd));
    float G_sqrd = pow(G, 2);
    float c      = (pow(ecc, 4) * F * p_sqrd) / pow(G, 3);
    float c_sqrd = pow(c, 2);
    float s      = pow(1 + c + sqrt(c_sqrd + (2 * c)), 1.5);
    float k      = s + 1 + (1 / s);
    float k_sqrd = pow(k, 2);
    float P      = F / (3 * k_sqrd * G_sqrd);
    float Q      = sqrt(1 + (2 * pow(ecc, 4) * P));
    float r0     = ((-P * ecc_sqrd * p) / (1 + Q)) + sqrt((0.5 * a_sqrd * (1 + (1 / Q))) - ((P * (1 - ecc_sqrd) * z_sqrd) / (Q * (1 + Q))) - (0.5 * P * p_sqrd));
    float U      = sqrt(pow(p - (ecc_sqrd * r0), 2) + z_sqrd);
    float V      = sqrt(pow(p - (ecc_sqrd * r0), 2) + ((1 - ecc_sqrd) * z_sqrd));
    float z0     = (b_sqrd * z) / (a * V);

    float h   = U * (1 - (b_sqrd / (a * V)));
    float lat = atan2(z + (ecc_prime_sqrd * z0), p);

    if (!unit_rad)
    {
        lat = rad2deg(lat);
        lon = rad2deg(lon);
    }

    Vector3f lla;

    lla << lat,
           lon,
           h;

    return lla;
}


/*
Transform a vector resolved in ECEF coordinate to its resolution in the NED
coordinate.The center of the NED coordiante is given by lat_ref, lon_ref,
and alt_ref.

https://en.wikipedia.org/wiki/Local_tangent_plane_coordinates
*/
Vector3f ecef2ned(const Vector3f& ecef, const Vector3f& lla_ref, const bool& unit_rad = false)
{
    float lat_ref = lla_ref(0);
    float lon_ref = lla_ref(1);

    if (!unit_rad)
    {
        lat_ref = deg2rad(lat_ref);
        lon_ref = deg2rad(lon_ref);
    }

    Matrix3f C(3, 3);

    C(0, 0) = -sin(lat_ref) * cos(lon_ref);
    C(0, 1) = -sin(lat_ref) * sin(lon_ref);
    C(0, 2) =  cos(lat_ref);

    C(1, 0) = -sin(lon_ref);
    C(1, 1) =  cos(lon_ref);
    C(1, 2) =  0;

    C(2, 0) = -cos(lat_ref) * cos(lon_ref);
    C(2, 1) = -cos(lat_ref) * sin(lon_ref);
    C(2, 2) = -sin(lat_ref);

    Vector3f ecef_ref = lla2ecef(lla_ref, unit_rad);
    
    Vector3f ned;
    ned = C * (ecef - ecef_ref);

    return ned;
}


/*
Convert Latitude, Longitude, Altitude to its resolution in the NED
coordinate.The center of the NED coordiante is given by lat_ref, lon_ref,
and alt_ref.

For example, this can be used to convert GPS data to a local NED frame.
*/
Vector3f lla2ned(const Vector3f& lla, const Vector3f& lla_ref, const bool& unit_rad = false)
{
    Vector3f ecef = lla2ecef(lla, unit_rad);
    Vector3f ned  = ecef2ned(ecef, lla_ref, unit_rad);

    return ned;
}


/*
Transform a vector resolved in NED(origin given by lat_ref, lon_ref, and alt_ref)
coordinates to its ECEF representation.
*/
Vector3f ned2ecef(const Vector3f& ned, const Vector3f& lla_ref, const bool& unit_rad = false)
{
    float lat_ref = lla_ref(0);
    float lon_ref = lla_ref(1);

    if (!unit_rad)
    {
        lat_ref = deg2rad(lat_ref);
        lon_ref = deg2rad(lon_ref);
    }

    Matrix3f C(3, 3);

    C(0, 0) = -sin(lat_ref) * cos(lon_ref);
    C(0, 1) = -sin(lat_ref) * sin(lon_ref);
    C(0, 2) = cos(lat_ref);

    C(1, 0) = -sin(lon_ref);
    C(1, 1) = cos(lon_ref);
    C(1, 2) = 0;

    C(2, 0) = -cos(lat_ref) * cos(lon_ref);
    C(2, 1) = -cos(lat_ref) * sin(lon_ref);
    C(2, 2) = -sin(lat_ref);

    Vector3f ecef;
    ecef = C.transpose() * ned;

    return ecef;
}


/*
Calculate the Latitude, Longitudeand Altitude of points given by NED coordinates
where NED origin given by lat_ref, lon_ref, and alt_ref.
*/
Vector3f ned2lla(const Vector3f& ned, const Vector3f& lla_ref, const bool& unit_rad = false)
{
    Vector3f ecef     = ned2ecef(ned, lla_ref, unit_rad);
    Vector3f ecef_ref = lla2ecef(lla_ref, unit_rad);
    ecef += ecef_ref;

    Vector3f lla = ecef2lla(ecef, unit_rad);

    return lla;
}


/*
Make a skew symmetric 2 - D array
*/
Matrix3f skew(const Vector3f& w)
{
    Matrix3f C(3, 3);

    C << 0.0, -w(2),  w(1),
        w(2),   0.0, -w(0),
       -w(1),  w(0),   0.0;

    return C;
}

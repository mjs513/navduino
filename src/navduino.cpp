#include "Arduino.h"
#include "math.h"
#include "navduino.h"
#include "wgs84.h"
#include "eigen.h"
#include <Eigen/LU>
#include <Eigen/Geometry>




using namespace Eigen;




/*
  Description:
  ------------
  This function converts a vector of euler angles into a Direction Cosine
  Matrix (DCM) given a rotation and frame sequence. In the case of this
  function, the returned DCM maps vectors from one coordinate frame (either
  North-East-Down (NED) or body frame) into the other. This is done by
  multiplying the DCM with the vector to be rotated: v_body = DCM * v_NED.

  https://en.wikipedia.org/wiki/Rotation_matrix
  https://en.wikipedia.org/wiki/Axes_conventions
  https://en.wikipedia.org/wiki/Euler_angles
  
  Arguments:
  ----------
  * const Vector3f& angles       - Vector of euler angles to describe the rotation
  * const bool& angle_unit       - Unit of the euler angles (rad or degrees)
  * const bool& NED_to_body      - Rotate either to or from the NED frame
  * const int& rotation_sequence - The order in which the euler angles are applied
                                   to the rotation. 321 is the standard rotation
                                   sequence for aerial navigation
  
  Returns:
  --------
  * Matrix3f dcm - Rotates vectors from one coordinate frame to the other
*/
Matrix3f angle2dcm(const Vector3f& angles, const bool& angle_unit, const bool& NED_to_body, const int& rotation_sequence)
{
    float roll;
    float pitch;
    float yaw;

    if (angle_unit == DEGREES)
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

    Matrix3f R1(3, 3);
    Matrix3f R2(3, 3);
    Matrix3f R3(3, 3);

    R1 << 1,          0,         0,
          0,  cos(roll), sin(roll),
          0, -sin(roll), cos(roll);

    R2 << cos(pitch), 0, -sin(pitch),
                   0, 1,           0,
          sin(pitch), 0,  cos(pitch);

    R3 << cos(yaw), sin(yaw), 0,
         -sin(yaw), cos(yaw), 0,
                 0,        0, 1;

    Matrix3f dcm(3, 3);

    // Note that multiplication of the matrices are opposite to the
    // rotation sequence due to how matrix multiplication works
    if (rotation_sequence == 321)
        dcm = R1 * R2 * R3;
    else if (rotation_sequence == 312)
        dcm = R2 * R1 * R3;
    else if (rotation_sequence == 231)
        dcm = R1 * R3 * R2;
    else if (rotation_sequence == 213)
        dcm = R3 * R1 * R2;
    else if (rotation_sequence == 132)
        dcm = R2 * R3 * R1;
    else if (rotation_sequence == 123)
        dcm = R3 * R2 * R1;
    else
        dcm = R1 * R2 * R3;

    if (!NED_to_body)
        return dcm.transpose();

    return dcm;
}




/*
  Description:
  ------------
  This function converts a Direction Cosine Matrix (DCM) into the 
  corresponding euler angles.

  https://en.wikipedia.org/wiki/Rotation_matrix
  https://en.wikipedia.org/wiki/Axes_conventions
  https://en.wikipedia.org/wiki/Euler_angles

  Arguments:
  ----------
  * const Matrix3f& dcm          - Rotates vectors from one coordinate frame to the other
  * const bool& angle_unit       - Unit of the euler angles (rad or degrees)
  * const bool& NED_to_body      - Rotate either to or from the NED frame
  * const int& rotation_sequence - The order in which the euler angles are applied
                                   to the rotation. 321 is the standard rotation
                                   sequence for aerial navigation

  Returns:
  --------
  * Vector3f angles - Vector of euler angles to describe the rotation
*/
Vector3f dcm2angle(const Matrix3f& dcm, const bool& angle_unit, const bool& NED_to_body, const int& rotation_sequence)
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

    if (angle_unit == DEGREES)
    {
        angles(0) = rad2deg(angles(0));
        angles(1) = rad2deg(angles(1));
        angles(2) = rad2deg(angles(2));
    }

    return angles;
}




/*
  Description:
  ------------
  Convert a sequence of euler angles to an equivalent unit quaternion.

  https://eater.net/quaternions
  https://en.wikipedia.org/wiki/Axes_conventions
  https://en.wikipedia.org/wiki/Euler_angles

  Arguments:
  ----------
  * const Vector3f& angles       - Vector of euler angles to describe the rotation
  * const bool& angle_unit       - Unit of the euler angles (rad or degrees)
  * const bool& NED_to_body      - Rotate either to or from the NED frame
  * const int& rotation_sequence - The order in which the euler angles are applied
                                   to the rotation. 321 is the standard rotation
                                   sequence for aerial navigation

  Returns:
  --------
  * Quaternionf quat - Quaternion that describes the rotation
*/
Quaternionf angle2quat(const Vector3f& angles, const bool& angle_unit, const bool& NED_to_body, const int& rotation_sequence)
{
    Quaternionf quat(angle2dcm(angles, angle_unit, NED_to_body, rotation_sequence));
    return quat;
}




/*
  Description:
  ------------
  Convert a unit quaternion to the equivalent sequence of euler angles
  about the rotation_sequence axes.

  https://eater.net/quaternions
  https://en.wikipedia.org/wiki/Axes_conventions
  https://en.wikipedia.org/wiki/Euler_angles

  Arguments:
  ----------
  * const Quaternionf quat       - Quaternion that describes the rotation
  * const bool& angle_unit       - Unit of the euler angles (rad or degrees)
  * const bool& NED_to_body      - Rotate either to or from the NED frame
  * const int& rotation_sequence - The order in which the euler angles are applied
                                   to the rotation. 321 is the standard rotation
                                   sequence for aerial navigation

  Returns:
  --------
  * Vector3f& angles - Vector of euler angles to describe the rotation
*/
Vector3f quat2angle(const Quaternionf& quat, const bool& angle_unit, const bool& NED_to_body, const int& rotation_sequence)
{
    return dcm2angle(quat.toRotationMatrix(), angle_unit, NED_to_body, rotation_sequence);
}




/*
  Description:
  ------------
  Convert a single unit quaternion to one DCM.

  https://eater.net/quaternions
  https://en.wikipedia.org/wiki/Rotation_matrix
  https://en.wikipedia.org/wiki/Axes_conventions

  Arguments:
  ----------
  * const Quaternionf quat       - Quaternion that describes the rotation
  * const bool& angle_unit       - Unit of the euler angles (rad or degrees)
  * const bool& NED_to_body      - Rotate either to or from the NED frame
  * const int& rotation_sequence - The order in which the euler angles are applied
                                   to the rotation. 321 is the standard rotation
                                   sequence for aerial navigation

  Returns:
  --------
  * Matrix3f& dcm - Rotates vectors from one coordinate frame to the other
*/
Matrix3f quat2dcm(const Quaternionf& quat)
{
    return quat.toRotationMatrix();
}




/*
  Description:
  ------------
  Convert a DCM to a unit quaternion.

  https://eater.net/quaternions
  https://en.wikipedia.org/wiki/Rotation_matrix
  https://en.wikipedia.org/wiki/Axes_conventions

  Arguments:
  ----------
  * const Matrix3f& dcm          - Rotates vectors from one coordinate frame to the other
  * const bool& angle_unit       - Unit of the euler angles (rad or degrees)
  * const bool& NED_to_body      - Rotate either to or from the NED frame
  * const int& rotation_sequence - The order in which the euler angles are applied
                                   to the rotation. 321 is the standard rotation
                                   sequence for aerial navigation

  Returns:
  --------
  * Quaternionf quat - Quaternion that describes the rotation
*/
Quaternionf dcm2quat(const Matrix3f& C)
{
    Quaternionf quat(C);
    return quat;
}




/*
  Description:
  ------------
  Calculate Earth's geocentric radius at a given geodetic latitude in
  meters. "The geocentric radius is the distance from the Earth's
  center to a point on the spheroid surface at geodetic latitude".

  https://en.wikipedia.org/wiki/Earth_radius
  https://en.wikipedia.org/wiki/Geographic_coordinate_system

  Arguments:
  ----------
  * const float& _lat      - Angle of latitude
  * const bool& angle_unit - Unit of the latitude angle (rad or degrees)
  
  Returns:
  --------
  * float radius - Earth's geocentric radius at a given latitude in meters
*/
float earthGeoRad(const float& _lat, const bool& angle_unit)
{
    float lat = _lat;

    if (angle_unit == DEGREES)
        lat = deg2rad(lat);

    float num = pow(a_sqrd * cos(lat), 2) + pow(b_sqrd * sin(lat), 2);
    float den = pow(a * cos(lat), 2) + pow(b * sin(lat), 2);

    return sqrt(num / den);
}




/*
  Description:
  ------------
  Calculate Earth's radius of curvature in the prime vertical (East -
  West) - denoted as N - and meridian (North - South) - denoted as
  M - at a given latitude.

  https://en.wikipedia.org/wiki/Radius_of_curvature
  https://en.wikipedia.org/wiki/Earth_radius
  https://en.wikipedia.org/wiki/Geographic_coordinate_system

  Arguments:
  ----------
  * const float& _lat      - Angle of latitude
  * const bool& angle_unit - Unit of the latitude angle (rad or degrees)

  Returns:
  --------
  * Vector2f radii - Earth's radii of curvature { N, M } at a given latitude
*/
Vector2f earthRad(const float& _lat, const bool& angle_unit)
{
    float lat = _lat;

    if (angle_unit == DEGREES)
        lat = deg2rad(lat);

    float R_N = a / sqrt(1 - (ecc_sqrd * pow(sin(lat), 2)));
    float R_M = (a * (1 - ecc_sqrd)) / pow(1 - (ecc_sqrd * pow(sin(lat), 2)), 1.5);

    Vector2f radii;
    radii << R_N, // Earth's prime-vertical radius of curvature
             R_M; // Earth's meridional radius of curvature

    return radii;
}




/*
  Description:
  ------------
  Calculate azimuthal radius at a given latitude. This is the
  Earth's radius of cuvature at a given latitude in the
  direction of a given asimuth.

  https://en.wikipedia.org/wiki/Radius_of_curvature
  https://en.wikipedia.org/wiki/Earth_radius
  https://en.wikipedia.org/wiki/Geographic_coordinate_system

  Arguments:
  ----------
  * const float& _lat      - Angle of latitude
  * const bool& angle_unit - Unit of the latitude angle (rad or degrees)

  Returns:
  --------
  * float radius - Earth's azimuthal radius at a given latitude and azimuth
*/
float earthAzimRad(const float& lat, const float& _azimuth, const bool& angle_unit)
{
    float azimuth = _azimuth;

    if (angle_unit == DEGREES)
        azimuth = deg2rad(azimuth);

    Vector2f eradvec = earthRad(lat, angle_unit);
    float N = eradvec(0);
    float M = eradvec(1);

    return 1 / ((pow(cos(azimuth), 2) / M) + (pow(sin(azimuth), 2) / N));
}




/*
  Description:
  ------------
  Calculate Earth's angular velocity in m/s resolved in the NED frame at a given
  latitude.

  Arguments:
  ----------
  * const float& _lat      - Angle of latitude
  * const bool& angle_unit - Unit of the latitude angle (rad or degrees)

  Returns:
  --------
  * Vector3f e - Earth's angular velocity in rad/s resolved in the NED frame
                 at a given latitude
*/
Vector3f earthRate(const float& _lat, const bool& angle_unit)
{
    float lat = _lat;

    if (angle_unit == DEGREES)
        lat = deg2rad(lat);

    Vector3f e;
    e << omega_E * cos(lat),
         0,
        -omega_E * sin(lat);

    return e;
}




/*
Calculate Latitude, Longitude, Altitude Rate given locally tangent velocity
*/
Vector3f llaRate(const Vector3f& vned, const Vector3f& lla, const bool& angle_unit)
{
    float VN = vned(0);
    float VE = vned(1);
    float VD = vned(2);

    float lat = lla(0);
    float alt = lla(2);

    Vector2f eradvec = earthRad(lat, angle_unit);
    float Rew = eradvec(0);
    float Rns = eradvec(1);

    if (angle_unit == DEGREES)
        lat = deg2rad(lat);

    Vector3f lla_dot;

    if (angle_unit == RADIANS)
    {
        lla_dot << VN / (Rns + alt),
                   VE / (Rew + alt) / cos(lat),
                  -VD;
    }
    else
    {
        lla_dot << rad2deg(VN / (Rns + alt)),
                   rad2deg(VE / (Rew + alt) / cos(lat)),
                  -VD;
    }

    return lla_dot;
}




/*
Calculate navigation / transport rate given VN, VE, VD, lat, and alt.
Navigation / transport rate is the angular velocity of the NED frame relative
to the earth ECEF frame.
*/
Vector3f navRate(const Vector3f& vned, const Vector3f& lla, const bool& angle_unit)
{
    float VN = vned(0);
    float VE = vned(1);

    float lat = lla(0);
    float alt = lla(2);

    Vector2f eradvec = earthRad(lat, angle_unit);
    float Rew = eradvec(0);
    float Rns = eradvec(1);

    if (angle_unit == DEGREES)
        lat = deg2rad(lat);

    Vector3f rho;

    if (angle_unit == RADIANS)
    {
        rho << VE / (Rew + alt),
              -VN / (Rns + alt),
              -VE * tan(lat) / (Rew + alt);
    }
    else
    {
        rho << rad2deg(VE / (Rew + alt)),
               rad2deg(-VN / (Rns + alt)),
               rad2deg(-VE * tan(lat) / (Rew + alt));
    }

    return rho;
}




/*
Convert Latitude, Longitude, Altitude, to ECEF position
*/
Vector3f lla2ecef(const Vector3f& lla, const bool& angle_unit)
{
    float lat = lla(0);
    float lon = lla(1);
    float alt = lla(2);

    Vector2f eradvec = earthRad(lat, angle_unit);
    float Rew = eradvec(0);

    if (angle_unit == DEGREES)
    {
        lat = deg2rad(lat);
        lon = deg2rad(lon);
    }

    Vector3f ecef;

    ecef << (Rew + alt) * cos(lat) * cos(lon),
            (Rew + alt)* cos(lat)* sin(lon),
            ((1 - ecc_sqrd) * Rew + alt)* sin(lat);

    return ecef;
}




/*
Calculate the Latitude, Longitudeand Altitude of a point located on earth
given the ECEF Coordinates.

https://en.wikipedia.org/wiki/Geographic_coordinate_conversion
*/
Vector3f ecef2lla(const Vector3f& ecef, const bool& angle_unit)
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

    if (angle_unit == DEGREES)
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
Vector3f ecef2ned(const Vector3f& ecef, const Vector3f& lla_ref, const bool& angle_unit)
{
    float lat_ref = lla_ref(0);
    float lon_ref = lla_ref(1);

    if (angle_unit == DEGREES)
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

    Vector3f ecef_ref = lla2ecef(lla_ref, angle_unit);

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
Vector3f lla2ned(const Vector3f& lla, const Vector3f& lla_ref, const bool& angle_unit)
{
    Vector3f ecef = lla2ecef(lla, angle_unit);
    Vector3f ned  = ecef2ned(ecef, lla_ref, angle_unit);

    return ned;
}




/*
Transform a vector resolved in NED(origin given by lat_ref, lon_ref, and alt_ref)
coordinates to its ECEF representation.
*/
Vector3f ned2ecef(const Vector3f& ned, const Vector3f& lla_ref, const bool& angle_unit)
{
    float lat_ref = lla_ref(0);
    float lon_ref = lla_ref(1);

    if (angle_unit == DEGREES)
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

    Vector3f ecef;
    ecef = C.transpose() * ned;

    return ecef;
}




/*
Calculate the Latitude, Longitudeand Altitude of points given by NED coordinates
where NED origin given by lat_ref, lon_ref, and alt_ref.
*/
Vector3f ned2lla(const Vector3f& ned, const Vector3f& lla_ref, const bool& angle_unit)
{
    Vector3f ecef     = ned2ecef(ned, lla_ref, angle_unit);
    Vector3f ecef_ref = lla2ecef(lla_ref, angle_unit);
    ecef += ecef_ref;

    Vector3f lla = ecef2lla(ecef, angle_unit);

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




float bearingLla(const Vector3f& lla_1, const Vector3f& lla_2, const bool& angle_unit)
{
    float lat_1 = lla_1(0);
    float lon_1 = lla_1(1);

    float lat_2 = lla_2(0);
    float lon_2 = lla_2(1);

    if (angle_unit == DEGREES)
    {
        lat_1 = deg2rad(lat_1);
        lon_1 = deg2rad(lon_1);

        lat_2 = deg2rad(lat_2);
        lon_2 = deg2rad(lon_2);
    }

    float deltaLon = lon_2 - lon_1;

    float x = cos(lat_2) * sin(deltaLon);
    float y = cos(lat_1) * sin(lat_2) - sin(lat_1) * cos(lat_2) * cos(deltaLon);

    return fmod(rad2deg(atan2(x, y)) + 360, 360);
}




float bearingNed(const Vector3f& ned_1, const Vector3f& ned_2)
{
    float n_1 = ned_1(0);
    float e_1 = ned_1(1);

    float n_2 = ned_2(0);
    float e_2 = ned_2(1);

    float x = e_2 - e_1;
    float y = n_2 - n_1;

    return fmod(rad2deg(atan2(x, y)) + 360, 360);
}




double distanceLla(const Vector3f& lla_1, const Vector3f& lla_2, const bool& angle_unit)
{
    float lat_1 = lla_1(0);
    float lon_1 = lla_1(1);

    float lat_2 = lla_2(0);
    float lon_2 = lla_2(1);

    if (angle_unit == DEGREES)
    {
        lat_1 = deg2rad(lat_1);
        lon_1 = deg2rad(lon_1);

        lat_2 = deg2rad(lat_2);
        lon_2 = deg2rad(lon_2);
    }

    double deltaLat = lat_2 - lat_1;
    double deltaLon = lon_2 - lon_1;

    double _a = (sin(deltaLat / 2) * sin(deltaLat / 2)) + cos(lat_1) * cos(lat_2) * (sin(deltaLon / 2)) * (sin(deltaLon / 2));

    float azimuth = bearingLla(lla_1, lla_2, angle_unit);
    float radius  = earthAzimRad(lla_1(0), azimuth, angle_unit);

    return 2 * radius * atan2(sqrt(_a), sqrt(1 - _a));
}




float distanceNed(const Vector3f& ned_1, const Vector3f& ned_2)
{
    Vector3f out = ned_2 - ned_1;
    return out.norm();
}




float distanceNedHoriz(const Vector3f& ned_1, const Vector3f& ned_2)
{
    Vector2f out;

    out << ned_2(0) - ned_1(0),
           ned_2(1) - ned_1(1);

    return out.norm();
}




float distanceNedVert(const Vector3f& ned_1, const Vector3f& ned_2)
{
    return ned_2(2) - ned_1(2);
}




float distanceEcef(const Vector3f& ecef_1, const Vector3f& ecef_2)
{
    Vector3f out = ecef_2 - ecef_1;
    return out.norm();
}




float elevationLla(const Vector3f& lla_1, const Vector3f& lla_2, const bool& angle_unit)
{
    float lat_1 = lla_1(0);
    float lon_1 = lla_1(1);
    float alt_1 = lla_1(2);

    float lat_2 = lla_2(0);
    float lon_2 = lla_2(1);
    float alt_2 = lla_2(2);

    if (angle_unit == DEGREES)
    {
        lat_1 = deg2rad(lat_1);
        lon_1 = deg2rad(lon_1);

        lat_2 = deg2rad(lat_2);
        lon_2 = deg2rad(lon_2);
    }

    float dist   = distanceLla(lla_1, lla_2, angle_unit);
    float height = alt_2 - alt_1;

    return rad2deg(atan2(height, dist));
}




float elevationNed(const Vector3f& ned_1, const Vector3f& ned_2)
{
    float d_1 = -ned_1(2);
    float d_2 = -ned_2(2);

    float dist   = distanceNed(ned_1, ned_2);
    float height = d_2 - d_1;

    return rad2deg(atan2(height, dist));
}




Vector3f LDAE2lla(const Vector3f& lla, const float& dist, const float& _azimuth, const float& _elevation, const bool& angle_unit)
{
    float lat = lla(0);
    float lon = lla(1);
    float alt = lla(2);

    float azimuth   = _azimuth;
    float elevation = _elevation;

    if (angle_unit == DEGREES)
    {
        lat = deg2rad(lat);
        lon = deg2rad(lon);

        azimuth   = deg2rad(azimuth);
        elevation = deg2rad(elevation);
    }

    float radius   = earthAzimRad(lla(0), _azimuth, angle_unit);
    float adj_dist = dist / radius;

    float lat_2 = asin(sin(lat) * cos(adj_dist) + cos(lat) * sin(adj_dist) * cos(azimuth));
    float lon_2 = lon + atan2(sin(azimuth) * sin(adj_dist) * cos(lat), cos(adj_dist) - sin(lat) * sin(lat_2));

    Vector3f out;

    out << rad2deg(lat_2),
           rad2deg(lon_2),
           alt + (dist * tan(elevation));

    return out;
}




Vector3f NDAE2ned(const Vector3f& ned, const float& dist, const float& _azimuth, const float& _elevation, const bool& angle_unit)
{
    float n = ned(0);
    float e = ned(1);
    float d = ned(2);

    float azimuth   = _azimuth;
    float elevation = _elevation;

    if (angle_unit == DEGREES)
    {
        azimuth   = deg2rad(azimuth);
        elevation = deg2rad(elevation);
    }

    Vector3f out;

    out << n + dist * cos(azimuth),
           e + dist * sin(azimuth),
           d + (dist * tan(elevation));

    return out;
}

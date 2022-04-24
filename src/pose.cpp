#include "Arduino.h"
#include "navduino.h"




void vehicle_pose::update_dcm(const Matrix3f& _v_R_n_)
{
	_v_R_n = _v_R_n_;
	_v_P_e = poseMat(_v_R_n * _n_R_e, _e_t_e_v);
};




void vehicle_pose::update_loc_lla(const Vector3f& _lla_, const bool& angle_unit)
{
	if (angle_unit == RADIANS)
	{
		_lla << rad2deg(_lla_(0)),
			    rad2deg(_lla_(1)),
			    _lla_(2);
	}
	else
		_lla = _lla_;

	_e_t_e_v = lla2ecef(_lla, DEGREES);
	_n_R_e   = ecef2ned_dcm(_lla, DEGREES);
	_n_P_e   = poseMat(_n_R_e, _e_t_e_v);
	_v_P_e   = poseMat(_v_R_n * _n_R_e, _e_t_e_v);
};




void vehicle_pose::update_loc_ecef(const Vector3f& _ecef_)
{
	_lla     = ecef2lla(_ecef_, DEGREES);
	_e_t_e_v = _ecef_;
	_n_R_e   = ecef2ned_dcm(_lla, DEGREES);
	_n_P_e   = poseMat(_n_R_e, _e_t_e_v);
	_v_P_e   = poseMat(_v_R_n * _n_R_e, _e_t_e_v);
};




void payload_pose::update_v_t_v_p(const Vector3f& _v_t_v_p_)
{
	_v_t_v_p = _v_t_v_p_;
	_p_P_v   = poseMat(_p_R_v, _v_t_v_p);
};




void payload_pose::update_p_t_p_v(const Vector3f& _p_t_p_v_)
{
	_v_t_v_p = -_p_t_p_v_;
	_p_P_v   = poseMat(_p_R_v, _v_t_v_p);
}




void payload_pose::update_p_R_v(const Matrix3f& _p_R_v_)
{
	_p_R_v = _p_R_v_;
	_p_P_v = poseMat(_p_R_v, _v_t_v_p);
};




void payload_pose::update_v_R_p(const Matrix3f& _v_R_p_)
{
	_p_R_v = _v_R_p_.transpose();
	_p_P_v = poseMat(_p_R_v, _v_t_v_p);
};




void payload_pose::update_p_P_v(const Matrix4f& _p_P_v_)
{
	_p_P_v   = _p_P_v_;
	_p_R_v   = pose2dcm(_p_P_v);
	_v_t_v_p = pose2t(_p_P_v);
}




void payload_pose::update_v_P_p(const Matrix4f& _v_P_p_)
{
	_p_P_v   = reversePoseMat(_v_P_p_);
	_p_R_v   = pose2dcm(_p_P_v);
	_v_t_v_p = pose2t(_p_P_v);
}




void sensor_pose::update_p_t_p_s(const Vector3f& _p_t_p_s_)
{
	_p_t_p_s = _p_t_p_s_;
	_s_P_p   = poseMat(_s_R_p, _p_t_p_s);
}




void sensor_pose::update_s_t_s_p(const Vector3f& _s_t_s_p_)
{
	_p_t_p_s = -_s_t_s_p_;
	_s_P_p   = poseMat(_s_R_p, _p_t_p_s);
}




void sensor_pose::update_s_R_p(const Matrix3f& _s_R_p_)
{
	_s_R_p = _s_R_p_;
	_s_P_p = poseMat(_s_R_p, _p_t_p_s);
}




void sensor_pose::update_p_R_s(const Matrix3f& _p_R_s_)
{
	_s_R_p = _p_R_s_.transpose();
	_s_P_p = poseMat(_s_R_p, _p_t_p_s);
}




void sensor_pose::update_s_P_p(const Matrix4f& _s_P_p_)
{
	_s_P_p   = _s_P_p_;
	_s_R_p   = pose2dcm(_s_P_p);
	_p_t_p_s = pose2t(_s_P_p);
}




void sensor_pose::update_p_P_s(const Matrix4f& _p_P_s_)
{
	_s_P_p   = reversePoseMat(_p_P_s_);
	_s_R_p   = pose2dcm(_s_P_p);
	_p_t_p_s = pose2t(_s_P_p);
}




Vector3f payload2vehicle(payload_pose& pay, const Vector3f& x)
{
	return transformPt(pay.v_P_p(), x);
}




Vector3f vehicle2payload(payload_pose& pay, const Vector3f& x)
{
	return transformPt(pay.p_P_v(), x);
}




Vector3f sensor2vehicle(payload_pose& pay, sensor_pose& sen, const Vector3f& x)
{
	return transformPt(pay.v_P_p() * sen.p_P_s(), x);
}




Vector3f vehicle2sensor(payload_pose& pay, sensor_pose& sen, const Vector3f& x)
{
	return transformPt(sen.s_P_p() * pay.p_P_v(), x);
}




Vector3f sensor2payload(payload_pose& pay, sensor_pose& sen, const Vector3f& x)
{
	return transformPt(sen.p_P_s(), x);
}




Vector3f payload2sensor(payload_pose& pay, sensor_pose& sen, const Vector3f& x)
{
	return transformPt(sen.s_P_p(), x);
}

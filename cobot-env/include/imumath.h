#ifndef IMUMATH_H
#define IMUMATH_H

#include <cmath>
#define RAD2DEG 57.2957795131
#define DEG2RAD 0.01745329251

struct Quaternion{
	double x, y, z, w;
};

struct Cardan{
	double roll, pitch, yaw;
};

Cardan get_cardan_angles(Quaternion q){
	Cardan angles;
	
	double sqw = q.w*q.w;
    double sqx = q.x*q.x;
    double sqy = q.y*q.y;
    double sqz = q.z*q.z;
    
    angles.roll  = atan2(2.0*(q.y*q.z+q.x*q.w),(-sqx-sqy+sqz+sqw)) * RAD2DEG;
	angles.pitch = asin(2.0*(q.y*q.w-q.x*q.z)/(sqx+sqy+sqz+sqw)) * RAD2DEG;
	angles.yaw   = atan2(2.0*(q.x*q.y+q.z*q.w),(sqx-sqy-sqz+sqw)) * RAD2DEG;
	
	return angles;
}

Cardan get_cardan_angles(double x, double y, double z, double w){
	Cardan angles;
	
	double sqw = w*w;
    double sqx = x*x;
    double sqy = y*y;
    double sqz = z*z;
    
    angles.roll  = atan2(2.0*(y*z+x*w),(-sqx-sqy+sqz+sqw)) * RAD2DEG;
	angles.pitch = asin(2.0*(y*w-x*z)/(sqx+sqy+sqz+sqw)) * RAD2DEG;
	angles.yaw   = atan2(2.0*(x*y+z*w),(sqx-sqy-sqz+sqw)) * RAD2DEG;
	
	return angles;
}

Quaternion get_quaternion(Cardan angles){
	Quaternion q;
	
	double cy = cos(angles.yaw * DEG2RAD * 0.5);
	double sy = sin(angles.yaw * DEG2RAD * 0.5);
	double cp = cos(angles.pitch * DEG2RAD * 0.5);
	double sp = sin(angles.pitch * DEG2RAD * 0.5);
	double cr = cos(angles.roll * DEG2RAD * 0.5);
	double sr = sin(angles.roll * DEG2RAD * 0.5);
	
	q.w = cr * cp * cy + sr * sp * sy;
	q.x = sr * cp * cy - cr * sp * sy;
	q.y = cr * sp * cy + sr * cp * sy;
	q.z = cr * cp * sy - sr * sp * cy;

	return q;
}

Quaternion get_quaternion(double roll, double pitch, double yaw){
	Quaternion q;
	
	double cy = cos(yaw * DEG2RAD * 0.5);
	double sy = sin(yaw * DEG2RAD * 0.5);
	double cp = cos(pitch * DEG2RAD * 0.5);
	double sp = sin(pitch * DEG2RAD * 0.5);
	double cr = cos(roll * DEG2RAD * 0.5);
	double sr = sin(roll * DEG2RAD * 0.5);
	
	q.w = cr * cp * cy + sr * sp * sy;
	q.x = sr * cp * cy - cr * sp * sy;
	q.y = cr * sp * cy + sr * cp * sy;
	q.z = cr * cp * sy - sr * sp * cy;

	return q;
}

#endif

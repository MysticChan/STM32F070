/*
Author Shi Lei
date 2018.6.5
AHRS  Algrithom
parameter explan:
imu.time 是s为单位的double 量，精确到1ms，时分秒转化得到

*/
//#pragma once
#include <string.h>
//#include "ComFunc.h"
//#include "elecompass.h"

//double ary_ax_err[200], ary_ay_err[200], ary_az_err[200];// save calibration data，
//double ary_gx_bias[200], ary_gy_bias[200], ary_gz_bias[200];
//const double deg2r = 3.141592653589 / 180;
//const double rad2d = 180 / 3.141592653589;
//const double gravity = 9.7883;
//const double deltaT = 0.005;
//double gravity_static;
//const double pi = 3.141592653589f;
//double install_acc[2]= { 0.0 };
//double bias_gyro[3]= { 0.0 };
//int count_imu = 0;
//const int num_imustatic = 200;
//enum type_states {
//	uninitial,
//	init_ready
//};

#define uninitial 0
#define init_ready 1
//enum type_states states;
typedef struct  {
	double ax, ay, az;
	double gx, gy, gz;
	double mx, my, mz;
	double time;
}type_imu;

typedef struct  {
	double roll, pitch, yaw;//deg
	double tilt, azimuth;
	int cred;// 1  ok 2 未静止 3 磁干扰 
	double time;
}type_ahrs;

struct type_magcomp {
	double center[3];
	double scater[9];
};
//struct type_magcomp magcomp0 = {
//	{ 16.6325, -0.8414,  22.4191},
//    { 0.0318,   0.0001,  0.0010,
//      0.0001,   0.0322, - 0.0008,
//      0.0010, - 0.0008,   0.0297}
//};
//struct type_magcomp acccomp0 = {
//	{ 0.0119 ,0.0012, 0.0954 },
//{ 0.9987 ,- 0.0000, - 0.0027,
//- 0.0000 , 1.0010 ,   0.0009,
//- 0.0027 , 0.0009 ,   0.9955}
//};

//struct type_magcomp magcomp0, acccomp0;
extern type_imu rawimu,calimu , lastimu;
extern type_ahrs ahrs,lastahrs;
int  initAhrs(type_imu rawimu);
int  cal_installerr(type_imu rawimu);
void comp_installerr(type_imu rawimu);
void cal_rpy(type_ahrs _ahrs);
int  instatic(type_imu imu);
void ahrsupdate(type_imu imu, type_ahrs ahrs);

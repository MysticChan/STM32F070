// elecompass.cpp: 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include "ComFunc.h"
#include "elecompass.h"

/*该函数用以实现坐标系调整,加计的误差补偿和滤波*/
int initAhrs(struct type_imu &rawimu)
{
  struct type_imu imu;
  double acct[3], magt[3];
  double acc[3] = { rawimu.ax,rawimu.ay,    rawimu.az };// to  ENU 
  double gyr[3] = { rawimu.gx,rawimu.gy,    rawimu.gz };
  double mag[3] = { rawimu.my,rawimu.mx, -1*rawimu.mz };
  // acc calibrate
  Mmin(acc, acccomp0.center, 3, 1);//mag calibrate  we use x y z on(ENU coordinates)
  Mmulnm(acccomp0.scater, acc, 3, 3, 1, acct);
  Mequalm(acct, 3, 1, acc);	
  // mag caliberate
	Mmin(mag, magcomp0.center, 3, 1);//mag calibrate  we use x y z on(ENU coordinates)
	Mmulnm(magcomp0.scater, mag, 3, 3, 1, magt);
	Mequalm(magt, 3, 1, mag);

	imu.ax = acc[0];
	imu.ay = acc[1];
	imu.az = acc[2];
	imu.gx = gyr[0];
	imu.gy = gyr[1];
	imu.gz = gyr[2];
	imu.mx = mag[0];
	imu.my = mag[1];
	imu.mz = mag[2];


	//coordinate change to ENU
	/*
	accgyr = rawimu.ax;//coordinate to ENU4.20
	rawimu.ax = rawimu.ay;
	rawimu.ay = accgyr;
	rawimu.az = -1 * rawimu.az;
	accgyr = rawimu.gx;
	rawimu.gx = rawimu.gy;//coordinate to ENU4.20
	rawimu.gy = accgyr;
	rawimu.gz = -1 * rawimu.gz;
	*/


	cal_installerr(imu);
	comp_installerr(imu);//output calimu
	cal_rpy(ahrs);// use calimu
	return 0;
}
//calculate accmeter install angle
int cal_installerr(struct type_imu &rawimu)
{
	double stdax, stday, stdaz;
	double aveax, aveay, aveaz;
	double avegx, avegy, avegz;
	double roll = 0.0, pitch = 0.0;

	if (count_imu<num_imustatic&&  states == uninitial)
	{
		ary_ax_err[count_imu] = rawimu.ax;
		ary_ay_err[count_imu] = rawimu.ay;
		ary_az_err[count_imu] = rawimu.az;
		ary_gx_bias[count_imu] = rawimu.gx;
		ary_gy_bias[count_imu] = rawimu.gy;
		ary_gz_bias[count_imu] = rawimu.gz;
		count_imu += 1;
		return 0;

	}
	if  (count_imu == num_imustatic && states == uninitial)
	{
		stdax = GetAveStdRMS(ary_ax_err, num_imustatic, 1);
		stday = GetAveStdRMS(ary_ay_err, num_imustatic, 1);
		stdaz = GetAveStdRMS(ary_az_err, num_imustatic, 1);
		printf("std ax %f ay %f az %f\n", stdax, stday, stdaz);
		if (stdax<0.006&& stday<0.006&&stdaz<0.009) //2.5times std of (ax ay az) in static
		{
			aveax = GetAveStdRMS(ary_ax_err, num_imustatic, 0);
			aveay = GetAveStdRMS(ary_ay_err, num_imustatic, 0);
			aveaz = GetAveStdRMS(ary_az_err, num_imustatic, 0);
			gravity_static = sqrt(aveax*aveax + aveay * aveay + aveaz * aveaz);
			roll = atan(-1 * aveax / sqrt(pow(aveay, 2) + pow(aveaz, 2)));
			pitch = atan2(aveay, aveaz);

			install_acc[0] = roll;//rad
			install_acc[1] = pitch;

			// calculate the gyrobias
			avegx = GetAveStdRMS(ary_gx_bias, num_imustatic, 0);
			avegy = GetAveStdRMS(ary_gy_bias, num_imustatic, 0);
			avegz = GetAveStdRMS(ary_gz_bias, num_imustatic, 0);
			bias_gyro[0] = avegx;
			bias_gyro[1] = avegy;
			bias_gyro[2] = avegz;
			printf("avegx=%lf avegy=%lf avegz=%lf \r\n", avegx, avegy, avegz);
			states = init_ready;
			return 1;
		}
		else
		{
			count_imu = 0;
			printf(" car is not in static,please turn off the engine \r\n");
			return 0;
		}
	}

}
//compasate accmeter error
void comp_installerr(struct type_imu &imu)
{
	imu.gx = imu.gx - bias_gyro[0];// gyros bias compasate
	imu.gy = imu.gy - bias_gyro[1];
	imu.gz = imu.gz - bias_gyro[2];
	double roll = install_acc[0];
	double pitch = install_acc[1];
	double acc_comp[3] = { 0.0 }, gyr_comp[3] = { 0.0 }, mag_comp[3] = { 0.0 };
	double acc[3] = { imu.ax,imu.ay, imu.az };
	double gyr[3] = { imu.gx,imu.gy, imu.gz };
	double mag[3] = { imu.mx,imu.my, imu.mz };
	//calculate levering matrix
	double Clevel[] = { cos(roll),0,sin(roll),
		sin(pitch)*sin(roll),cos(pitch),-1 * cos(roll)*sin(pitch),
		-1 * sin(roll)*cos(pitch),sin(pitch),cos(roll)*cos(pitch) };

	Mmulnm(Clevel, acc, 3, 3, 1, acc_comp);
	Mmulnm(Clevel, gyr, 3, 3, 1, gyr_comp);
	Mmulnm(Clevel, mag, 3, 3, 1, mag_comp);
	//Here calimu were got
	calimu.ax = acc_comp[0];
	calimu.ay = acc_comp[1];
	calimu.az = acc_comp[2];

	calimu.gx = gyr_comp[0];
	calimu.gy = gyr_comp[1];
	calimu.gz = gyr_comp[2];
#if 0
	// mag compansation
	calimu.mx = mag_comp[0];
	calimu.my = mag_comp[1];
	calimu.mz = mag_comp[2];
	// imu compansated now
#endif
}

//roll pitch yaw calculate
void cal_rpy(struct type_ahrs &_ahrs)
{
	double roll, pitch, yaw;
	double mx_level,my_level;
	double mag_level[3];
	double mag[3] = { calimu.mx, calimu.my, calimu.mz };
	double magsum;

	magsum = sqrt(calimu.mx*calimu.mx + calimu.my*calimu.my + calimu.mz*calimu.mz);
	if (instatic(calimu))//1 is statics and  0 means moving
	{
		roll = atan(-1 * calimu.ax / sqrt(pow(calimu.ay, 2) + pow(calimu.az, 2)));
		pitch = atan2(calimu.ay, calimu.az);
		ahrs.cred = 2;
	}
	else
	{
		roll = _ahrs.roll + calimu.gy*deltaT;
		pitch = _ahrs.pitch + calimu.gx*deltaT;
		ahrs.cred = 1;
	}
#if 1  // 磁北角的计算
	// ENU imu coordinate
	 double Clevel[9] = { cos(roll),0.0 ,sin(roll),
			          sin(pitch)*sin(roll),cos(pitch),-1 * cos(roll)*sin(pitch),
				 -1 * sin(roll)*cos(pitch),sin(pitch),cos(roll)*cos(pitch) };

		Mmulnm(Clevel, mag, 3, 3, 1, mag_level);
		mx_level = mag_level[0];
		my_level = mag_level[1];
		if (fabs(magsum - 1) < 0.08) 
		{
			yaw = atan2(mx_level, my_level)*180.0 / pi;
			ahrs.cred = 1;
		}
		else
		{
			yaw = ahrs.yaw - deltaT * calimu.gz;
			ahrs.cred = 3;
		}
		
		if (yaw<0.0)
		{
			yaw = yaw + 360;
		}   // [0-2*pi] how to cooperate yaw and rtk.hesding not in plane now .
		if (yaw>360)
		{
			yaw = yaw - 360;
		}
	
#endif 
	_ahrs.roll = roll;
	_ahrs.pitch = pitch;
	_ahrs.yaw = yaw;//deg
}
int  instatic(struct type_imu &imu)
{
	double gout;
	gout = sqrt(imu.ax*imu.ax + imu.ay*imu.ay + imu.az*imu.az);
	//printf("gout=%lf\n",gout);
	if (fabs(imu.gx)<0.035&&fabs(gout - gravity_static)<0.08&&fabs(imu.gz)<0.035)//need to do
	{
		ahrs.cred = 1;
		return 1;
	}
	else
	{
		ahrs.cred = 2;
		return 0;
	}
}
/*For user using*/
void ahrsupdate(struct type_imu imu, struct type_ahrs ahrs) 
{
	initAhrs(imu);
	ahrs.tilt = acos(calimu.az / gravity_static) * 180 / pi;
	lowpassFilter(ahrs.roll, lastahrs.roll, 0.05);
	lowpassFilter(ahrs.pitch, lastahrs.pitch, 0.05);
	lowpassFilter(ahrs.yaw, lastahrs.yaw, 0.05);
	lowpassFilter(ahrs.tilt, lastahrs.tilt, 0.05);
	lastahrs = ahrs;
}

int main( )
{   
	ahrsupdate(rawimu,ahrs);

	// tilt and yaw is output
    return 0;
}




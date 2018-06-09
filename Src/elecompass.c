//#include "comfun.h"
#include "elecompass.h"
#include "math.h"
#include "usart.h"

type_imu rawimu,calimu,lastimu;
type_ahrs ahrs,lastahrs;
double ary_ax_err[200], ary_ay_err[200], ary_az_err[200];// save calibration data£¬
double ary_gx_bias[200], ary_gy_bias[200], ary_gz_bias[200];
#define deg2r  3.141592653589 / 180;
#define rad2d  180 / 3.141592653589;
#define gravity  9.7883;
#define deltaT  0.005;
//const double deg2r = 3.141592653589 / 180;
//const double rad2d = 180 / 3.141592653589;
//const double gravity = 9.7883;
//const double deltaT = 0.005;
double gravity_static;
#define pi  3.141592653589f;
//const double pi = 3.141592653589f;
double install_acc[2]= { 0.0 };
double bias_gyro[3]= { 0.0 };
int count_imu = 0;
#define num_imustatic 200
//const int num_imustatic = 200;
unsigned char states = uninitial;


void lowpassFilter(double var, double lastvar,double k)
{
	var = k * var + (1 - k)*lastvar;
	var = lastvar;
}

struct type_magcomp magcomp0 = {
	{ 212.5,119.4,225.7 },
{ 4.9174e-03 , -4.2698e-05,  -5.4855e-05,
-4.2698e-05 ,  4.9760e-03,   1.8560e-05,
-5.4855e-05 ,  1.8560e-05,   4.6916e-03 }
};
struct type_magcomp acccomp0 = {
	{ 0,0,0 },
{ 1.0006e+00 , -2.0767e-05, -1.1629e-05,
-2.0767e-05,  1.002e+00, 1.8448e-05,
-1.1629e-05,  1.8448e-05, 1.000e+00 }
};
void Mminn(double *a,double *b,double *c,int m,int n)
{
	int i,j;

	for(i=0;i<m;i++)
		for(j=0;j<n;j++)
		{
			*(c+i*n+j)=*(a+i*n+j)-*(b+i*n+j);		
		}
}

double GetAveStdRMS(const double *a, int n, int opt)
{
	int i;
	double avg=0.0,std=0.0,rms=0.0,sum=0.0;

	if(n==0) return 99999.9;

	for (i=0;i<n;i++){
		sum+= a[i];
		rms+= a[i]*a[i];
	}
	avg=sum/n;

	if (opt==0) return avg;

	sum=0.0;
	for (i=0;i<n;i++){
		sum+=(a[i]-avg)*(a[i]-avg);
	}

	std=sqrt(sum/(double)(n-1));
	rms=sqrt(rms/(double)(n));

	if (opt==1) return std;
	else if (opt==2) return rms;

	return 0.0;
}

void Mmin(double *a,double *b,int m,int n)
{
	int i,j;

	for(i=0;i<m;i++)
		for(j=0;j<n;j++)
		{
			*(a+i*n+j)=*(a+i*n+j)-*(b+i*n+j);		
		}
}

void Mmulnm(double *a,double *b,int m,int n,int k,double *c) 
{
	int i,j,l,u;
	for(i=0;i<=m-1;i++)
	for(j=0;j<=k-1;j++)
	{   u=i*k+j;
	    c[u]=0.0;
	    for(l=0;l<=n-1;l++)
		    c[u]=c[u]+a[i*n+l]*b[l*k+j]; 
	}
}

void Mequalm(double *M,int m,int n,double *N)
{
   for (int i=0;i<m;i++)
   {
	   for (int j=0;j<n;j++)
	   {
		   *(N+i*n+j) = *(M+i*n+j);
	   }
   }
}

int initAhrs(type_imu rawimu)
{
  type_imu imu;
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

  if (states == uninitial)
  {
	   cal_installerr(imu);
  }
  else{
	      comp_installerr(imu);//output calimu
        cal_rpy(ahrs);// use calimu
    
    }
	return 0;
}
//calculate accmeter install angle
int cal_installerr(type_imu rawimu)
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
//		printf("std ax %f ay %f az %f\n", stdax, stday, stdaz);
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
//			printf("avegx=%lf avegy=%lf avegz=%lf \r\n", avegx, avegy, avegz);
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
  return -1;
}
//compasate accmeter error
void comp_installerr(type_imu imu)
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
void cal_rpy(type_ahrs _ahrs)
{
	double roll, pitch, yaw;

	if (instatic(calimu))//1 is statics and  0 means moving
	{
		roll = atan(-1 * calimu.ax / sqrt(pow(calimu.ay, 2) + pow(calimu.az, 2)))*180/pi;
		pitch = atan2(calimu.ay, calimu.az)*180/pi;
		yaw = _ahrs.yaw;
	}
	else
	{
		roll = _ahrs.roll + calimu.gy*deltaT;
		pitch = _ahrs.pitch + calimu.gx*deltaT;
		yaw = _ahrs.yaw - calimu.gz*deltaT;
		if (yaw<0.0)
		{
			yaw = yaw + 360;
		}   // [0-2*pi] how to cooperate yaw and rtk.hesding not in plane now .
		if (yaw>360)
		{
			yaw = yaw - 360;
		}
	}
	_ahrs.roll = roll;
	_ahrs.pitch = pitch;
	_ahrs.yaw = yaw;//deg
//  printf("roll %.3f pitch %.3f yaw %.3f\r\n",roll,pitch,yaw );
}
int  instatic(type_imu imu)
{
	double gout;
	gout = sqrt(imu.ax*imu.ax + imu.ay*imu.ay + imu.az*imu.az);
//	printf("gout=%lf\n",gout);
	if (fabs(imu.gx)<0.035&&fabs(gout - gravity_static)<0.08&&fabs(imu.gz)<0.035)//need to do
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
/*For user using*/
void ahrsupdate(type_imu imu, type_ahrs ahrs) 
{
	initAhrs(imu);
	ahrs.tilt = acos(calimu.az / gravity_static) * 180 / pi;
	lowpassFilter(ahrs.roll, lastahrs.roll, 0.05);
	lowpassFilter(ahrs.pitch, lastahrs.pitch, 0.05);
	lowpassFilter(ahrs.yaw, lastahrs.yaw, 0.05);
	lowpassFilter(ahrs.tilt, lastahrs.tilt, 0.05);
	lastahrs = ahrs;
  printf(" %.4f %.4f %.4f %.4f\r\n",ahrs.roll,ahrs.pitch,ahrs.yaw,ahrs.tilt);
}



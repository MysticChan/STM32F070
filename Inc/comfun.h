/**********************************************************************
 * ComFunc.h : 
 * 
 * Copyright (C) 2016-???? by Hi-Target, All rights reserved.
 * 
 * Created : 2016/07/27 
 * 
 * Description :
 *
 * References :
 *     
 * History : 2016/07/27 
**************************************************************************/

#pragma once

#include <math.h>
#include <stdio.h>
#include <string>
#include <stdlib.h>
#include <stdarg.h>
#include <iostream>
#include <algorithm> 

using namespace std;

#define RE_WGS84    6378137.0           // earth semimajor axis (WGS84) (m)
#define FE_WGS84    (1.0/298.257223563) // earth flattening (WGS84)
#define PI		3.14159265358979
#define DEG (3.14159265358979/180.0)
#define MAXVAL		30 // max value for spilting

/* Common calculation function------------------------ */
#pragma region common function
//临时申请内存 要free
extern void *__ml_zero(int size);
//字符串转数组，格式化转换
extern void Str2Array(const char *str,const char *sep,double *val);
//查找第一个子字符串
extern int checkstr(const char* str,const char* substr);
extern double GetAveStdRMS(const double *a, int n, int opt);
extern char* time2str(double *ep, int n);
extern double str2num(const char *s, int i, int n); 
extern void ecef2pos(const double *r, double *pos); //xyz2blh
extern void pos2ecef(const double *pos, double *r); //blh2xyz
extern void xyz2enu(const double *pos, double *E);  //blh2 Cxyz2enu
extern void ecef2enu(const double *pos, const double *r, double *e); //xyz坐标系向量r转化到enu
extern void enu2ecef(const double *pos, const double *e, double *r); //enu坐标系向量r转化到xyz
extern void matmul(const char *tr, int n, int k, int m, double alpha,
	              const double *A, const double *B, double beta, double *C);
extern double dot (const double *a, const double *b, int n);
extern double norm(const double *a, int n);
extern void cross3(const double *a, const double *b, double *c);
extern void getHMS(double ggat,double ep[6]);
extern void getPOS_rad( double lat,double lon,double hgt,double blh[3]);
#pragma endregion
/* end --------------------------------------------------------- */

/* Matrixs calculation function, by DHF.20160510---------- */
#pragma region Matrixs
//c[m][n]=a[m][n]+b[m][n]
void Maddn(double *a,double *b,double *c,int m,int n);
//a[m][n]=a[m][n]+b[m][n]
void Madd(double *a,double *b,int m,int n);
//c[m][n]=a[m][n]-b[m][n]
void Mminn(double *a,double *b,double *c,int m,int n);
//a[m][n]=a[m][n]-b[m][n]
void Mmin(double *a,double *b,int m,int n);
//c[m][k]=a[m][n]*b[n][k]
void Mmulnm(double *a,double *b,int m,int n,int k,double *c);
//a[m][n]=a[m][k]*b
void Mmul(double *a,int m,int n,double b);
//c[m][n]=a[m][k]*b
void Mmuln(double *a,int m,int n,double b,double *c);
//b=aT
void Mtn(double *a,int m,int n,double *b); 
//a=aT
void Mt(double *a,int m,int n); 
//inv(a)
double Minv(double a[],int n);
//b=inv(a)
double Minvn(double a[],int n,double *b);
 //A* adjoint matrix  
double Mrem(double *a,int i,int j,int n); 
 //det 
double Mdet(double *a,int n);
//N[m][n]=M[m][n]
void Mequalm(double *M,int m,int n,double *N);
//M[m][n]=a
void Mequal(double *M,int m,int n,double a);
//mean of col
double Mmean(double *a,int m);
//求实对称矩阵的特征值及特征向量的雅格比法 
//利用雅格比(Jacobi)方法求实对称矩阵的全部特征值及特征向量 
//返回值小于0表示超过迭代jt次仍未达到精度要求 
//返回值大于0表示正常返回 
//a-长度为n*n的数组，存放实对称矩阵，返回时对角线存放n个特征值 
//n-矩阵的阶数 
//u-长度为n*n的数组，返回特征向量(按列存储) 
//eps-控制精度要求 
//jt-整型变量，控制最大迭代次数 
int Meejcb(double a[],int n,double v[],double eps,int jt);
//eye
void Munit(double* a,int n);
#pragma endregion
/* end --------------------------------------------------------- */

//方向余弦矩阵转四元数ned
void m2qua_ned(double m[],double q[]);
//四元数转方向余弦矩阵ned
void q2mat_ned(double qua[],double m[]);
//方向余弦矩阵转欧拉角ned
void m2att_ned(double m[],double a[]);
//欧拉角转方向余弦矩阵ned
void a2mat_ned(double att[],double m[]);
/* end --------------------------------------------------------- */

/*-----------------log define by DHF ----------------------*/
extern void fopendhf(const char *file);
extern void fclosedhf(void);
extern void outdhf(const char *format, ...);
/* end --------------------------------------------------------- */

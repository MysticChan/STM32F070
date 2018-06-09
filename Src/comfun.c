#include "ComFunc.h"

#pragma region common function
extern void *__ml_zero(int size)
{
	void *p = malloc(size);
	if (p == NULL) {
		printf("not malloc for val\n");
	}
	memset(p, 0, size);
	return p;
}
extern void Str2Array(const char *str,const char *sep,double *val)
{
	char *p,_str[1024];
	double d[MAXVAL]={0.0};
	int i,j;

	strcpy_s(_str,str);
	for (i=0,p=strtok(_str,sep);p&&i<MAXVAL;p=strtok(NULL,sep),i++) { 
		d[i]=atof(p);
	}

	for(j=0;j<i;j++) val[j]=d[j];
}
extern int checkstr(const char* str,const char* substr)
{
	int i, j, check ,count = 0;
    int len = strlen(str);        /*取得字符串长度，不包括'\0'*/
    int sublen = strlen(substr);
	if(len<sublen)
	{return 0;}
    for(i=0;i<len;i++)
    {
        check = 1;                /*检测标记*/
        for(j=0;j+i<len&&j<sublen;j++)        /*逐个字符进行检测，在sublen长度内，一旦出现不同字符便将check置为0*/
        {
            if( str[i+j] != substr[j] )
            {
                check = 0;
                break;
            }
        }
        if(check == 1)           /*在sublen长度内的字符都相等*/
        {
			break;
			//count++;
			//i=i+sublen;           /*调整检测起始位置*/
        }
    }
    return i;
}
extern double GetAveStdRMS(const double *a, int n, int opt)
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

	std=sqrt(sum/double(n-1));
	rms=sqrt(rms/double(n));

	if (opt==1) return std;
	else if (opt==2) return rms;

	return 0.0;
}
extern char* time2str(double *ep, int n)
{
	static char buff[64];

	if (n<0) n=0;

	sprintf_s(buff,"%04.0f/%02.0f/%02.0f %02.0f:%02.0f:%0*.*f",ep[0],ep[1],ep[2],
		ep[3],ep[4],n<=0?2:n+3,n<=0?0:n,ep[5]);
	return buff;
}

extern double str2num(const char *s, int i, int n)
{
	double value;
	char str[256],*p=str;

	if (i<0||(int)strlen(s)<i||(int)sizeof(str)-1<n) return 0.0;
	for (s+=i;*s&&--n>=0;s++) *p++=*s=='d'||*s=='D'?'E':*s; *p='\0';
	return sscanf_s(str,"%lf",&value)==1?value:0.0;
}

extern void matmul(const char *tr, int n, int k, int m, double alpha,
	const double *A, const double *B, double beta, double *C)
{
	double d;
	int i,j,x,f=tr[0]=='N'?(tr[1]=='N'?1:2):(tr[1]=='N'?3:4);

	for (i=0;i<n;i++) for (j=0;j<k;j++) {
		d=0.0;
		switch (f) {
		case 1: for (x=0;x<m;x++) d+=A[i+x*n]*B[x+j*m]; break;
		case 2: for (x=0;x<m;x++) d+=A[i+x*n]*B[j+x*k]; break;
		case 3: for (x=0;x<m;x++) d+=A[x+i*m]*B[x+j*m]; break;
		case 4: for (x=0;x<m;x++) d+=A[x+i*m]*B[j+x*k]; break;
		}
		if (beta==0.0) C[i+j*n]=alpha*d; else C[i+j*n]=alpha*d+beta*C[i+j*n];
	}
}
/* transform ecef to geodetic postion ------------------------------------------
* transform ecef position to geodetic position
* args   : double *r        I   ecef position {x,y,z} (m)
*          double *pos      O   geodetic position {lat,lon,h} (rad,m)
* return : none
* notes  : WGS84, ellipsoidal height
*-----------------------------------------------------------------------------*/
extern void ecef2pos(const double *r, double *pos)
{
    double e2=FE_WGS84*(2.0-FE_WGS84),r2=dot(r,r,2),z,zk,v=RE_WGS84,sinp;
    
    for (z=r[2],zk=0.0;fabs(z-zk)>=1E-4;) {
        zk=z;
        sinp=z/sqrt(r2+z*z);
        v=RE_WGS84/sqrt(1.0-e2*sinp*sinp);
        z=r[2]+v*e2*sinp;
    }
    pos[0]=r2>1E-12?atan(z/sqrt(r2)):(r[2]>0.0?PI/2.0:-PI/2.0);
    pos[1]=r2>1E-12?atan2(r[1],r[0]):0.0;
    pos[2]=sqrt(r2+z*z)-v;
}
/* transform geodetic to ecef position -----------------------------------------
* transform geodetic position to ecef position
* args   : double *pos      I   geodetic position {lat,lon,h} (rad,m)
*          double *r        O   ecef position {x,y,z} (m)
* return : none
* notes  : WGS84, ellipsoidal height
*-----------------------------------------------------------------------------*/
extern void pos2ecef(const double *pos, double *r)
{
	double sinp=sin(pos[0]),cosp=cos(pos[0]),sinl=sin(pos[1]),cosl=cos(pos[1]);
	double e2=FE_WGS84*(2.0-FE_WGS84),v=RE_WGS84/sqrt(1.0-e2*sinp*sinp);

	r[0]=(v+pos[2])*cosp*cosl;
	r[1]=(v+pos[2])*cosp*sinl;
	r[2]=(v*(1.0-e2)+pos[2])*sinp;
}
/* ecef to local coordinate transfromation matrix ------------------------------
* compute ecef to local coordinate transfromation matrix
* args   : double *pos      I   geodetic position {lat,lon} (rad)
*          double *E        O   ecef to local coord transformation matrix (3x3)
* return : none
* notes  : matirix stored by column-major order (fortran convention)
*-----------------------------------------------------------------------------*/
extern void xyz2enu(const double *pos, double *E)
{
	double sinp=sin(pos[0]),cosp=cos(pos[0]),sinl=sin(pos[1]),cosl=cos(pos[1]);

	E[0]=-sinl;      E[3]=cosl;       E[6]=0.0;
	E[1]=-sinp*cosl; E[4]=-sinp*sinl; E[7]=cosp;
	E[2]=cosp*cosl;  E[5]=cosp*sinl;  E[8]=sinp;
}
/* transform ecef vector to local tangental coordinate -------------------------
* transform ecef vector to local tangental coordinate
* args   : double *pos      I   geodetic position {lat,lon} (rad)
*          double *r        I   vector in ecef coordinate {x,y,z}
*          double *e        O   vector in local tangental coordinate {e,n,u}
* return : none
*-----------------------------------------------------------------------------*/
extern void ecef2enu(const double *pos, const double *r, double *e)
{
	double E[9];

	xyz2enu(pos,E);
	matmul("NN",3,1,3,1.0,E,r,0.0,e);
}
/* transform local vector to ecef coordinate -----------------------------------
* transform local tangental coordinate vector to ecef
* args   : double *pos      I   geodetic position {lat,lon} (rad)
*          double *e        I   vector in local tangental coordinate {e,n,u}
*          double *r        O   vector in ecef coordinate {x,y,z}
* return : none
*-----------------------------------------------------------------------------*/
extern void enu2ecef(const double *pos, const double *e, double *r)
{
	double E[9];

	xyz2enu(pos,E);
	matmul("TN",3,1,3,1.0,E,e,0.0,r);
}
/* inner product ---------------------------------------------------------------
* inner product of vectors
* args   : double *a,*b     I   vector a,b (n x 1)
*          int    n         I   size of vector a,b
* return : a'*b
*-----------------------------------------------------------------------------*/
extern double dot(const double *a, const double *b, int n)
{
	double c=0.0;

	while (--n>=0) c+=a[n]*b[n];
	return c;
}
/* euclid norm -----------------------------------------------------------------
* euclid norm of vector
* args   : double *a        I   vector a (n x 1)
*          int    n         I   size of vector a
* return : || a ||
*-----------------------------------------------------------------------------*/
extern double norm(const double *a, int n)
{
	return sqrt(dot(a,a,n));
}
/* outer product of 3d vectors -------------------------------------------------
* outer product of 3d vectors 
* args   : double *a,*b     I   vector a,b (3 x 1)
*          double *c        O   outer product (a x b) (3 x 1)
* return : none
*-----------------------------------------------------------------------------*/
extern void cross3(const double *a, const double *b, double *c)
{
	c[0]=a[1]*b[2]-a[2]*b[1];
	c[1]=a[2]*b[0]-a[0]*b[2];
	c[2]=a[0]*b[1]-a[1]*b[0];
}

extern void getHMS(double ggat,double ep[6])
{
	ep[0]=0;ep[1]=0;ep[2]=0;
	ep[3]=int(ggat/10000.0);
	ep[4]=int(fmod(ggat,10000.0)/100.0);
	ep[5]=fmod(ggat,100.0);
}

extern void getPOS_rad( double lat,double lon,double hgt,double blh[3])
{
	double deg,min;

	deg=(int)(lat/100.0);
	min=fmod(lat,100.0);
	blh[0]=(deg+min/60.0)*PI/180.0;

	deg=(int)(lon/100.0);
	min=fmod(lon,100.0);
	blh[1]=(deg+min/60.0)*PI/180.0;

	blh[2]=hgt;
}
#pragma endregion

#pragma region Matrixs
void Maddn(double *a,double *b,double *c,int m,int n)
{   
	for(int i=0;i<m;i++)
		for(int j=0;j<n;j++)
		{
			*(c+i*n+j)=*(a+i*n+j)+*(b+i*n+j);
		}
}

void Madd(double *a,double *b,int m,int n)
{   
	for(int i=0;i<m;i++)
		for(int j=0;j<n;j++)
		{
			*(a+i*n+j)=*(a+i*n+j)+*(b+i*n+j);
		}
}

void Mminn(double *a,double *b,double *c,int m,int n)
{
	int i,j;

	for(i=0;i<m;i++)
		for(j=0;j<n;j++)
		{
			*(c+i*n+j)=*(a+i*n+j)-*(b+i*n+j);		
		}
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

void Mmul(double *a,int m,int n,double b)
{
   for (int i=0;i<m;i++)
   {
	   for (int j=0;j<n;j++)
	   {
		   *(a+i*n+j) = *(a+i*n+j)*b;
	   }
   }
}

void Mmuln(double *a,int m,int n,double b,double *c)
{
	for (int i=0;i<m;i++)
   {
	   for (int j=0;j<n;j++)
	   {
		   *(c+i*n+j) = *(a+i*n+j)*b;
	   }
   }
}

void Mtn(double *a,int m,int n,double *b) 
{
	for(int l=0;l<n;l++)
		for(int k=0;k<m;k++)
		{b[l*m+k]=a[k*n+l];}

}

 void Mt(double *a,int m,int n)
{
	double *at;
	at=(double *)malloc(n*m*sizeof(double));
	for(int i=0;i<m;i++)
		for(int j=0;j<n;j++)
		{
			*(at+i*n+j)=*(a+i*n+j);
		}
	for(int l=0;l<n;l++)
		for(int k=0;k<m;k++)
		{a[l*m+k]=at[k*n+l];}
    free(at);
}

double Minv(double a[],int n)
{
   int *is,*js,i,j,k,l,u,v;
   double d,p;
   is=(int *)malloc(n*sizeof(int));
   js=(int *)malloc(n*sizeof(int));
   for(k=0;k<=n-1;k++)
    { 
       d=0.0;
       for(i=k;i<=n-1;i++)
         for(j=k;j<=n-1;j++)
           {
             l=i*n+j; 
             p=fabs(a[l]);
             if(p>d)
              {  
                d=p;
			    is[k]=i;
			    js[k]=j;
              }
           }
      if(d==0)//d+1.0==1.0 lcc
        {
          free(is);
          free(js);
		  printf("\nerror:inverse matrix is not exist\n");
          return (0);
        }
      if(is[k]!=k)
        for(j=0;j<=n-1;j++)
          {
             u=k*n+j;v=is[k]*n+j;
             p=a[u];a[u]=a[v];a[v]=p;
          }
      if(js[k]!=k)
        for(i=0;i<=n-1;i++)
          {
            u=i*n+k;
            v=i*n+js[k];
            p=a[u];
            a[u]=a[v];a[v]=p;
          }
      l=k*n+k;
      a[l]=1.0/a[l];
      for(j=0;j<=n-1;j++)
        if(j!=k)
         {
            u=k*n+j;
            a[u]=a[u]*a[l];
         }
      for(i=0;i<=n-1;i++)
        if(i!=k)
          for(j=0;j<=n-1;j++)
            if(j!=k)
             {
               u=i*n+j;
               a[u]=a[u]-a[i*n+k]*a[k*n+j];
             }
      for(i=0;i<=n-1;i++)
         if(i!=k)
           {
             u=i*n+k;a[u]=-a[u]*a[l];
           }
    }     
    for(k=n-1;k>=0;k--)
      {
        if(js[k]!=k)
          for(j=0;j<=n-1;j++)
            {
               u=k*n+j;v=js[k]*n+j;
               p=a[u];
               a[u]=a[v];
               a[v]=p;
            }  
        if(is[k]!=k)
          for(i=0;i<=n-1;i++)
            {
               u=i*n+k;
			   v=i*n+is[k];	
               p=a[u];
			   a[u]=a[v];a[v]=p;
            }           
      }

     free(is);
     free(js);
     return(1);
}

double Minvn(double a[],int n,double *b)
{
	for(int i=0;i<n;i++)
		for(int j=0;j<n;j++)
		{
			*(b+i*n+j)=*(a+i*n+j);
		}

   int *is,*js,i,j,k,l,u,v;
   double d,p;
   is=(int *)malloc(n*sizeof(int));
   js=(int *)malloc(n*sizeof(int));
   for(k=0;k<=n-1;k++)
    { 
       d=0.0;
       for(i=k;i<=n-1;i++)
         for(j=k;j<=n-1;j++)
           {
             l=i*n+j; 
             p=fabs(b[l]);
             if(p>d)
              {  
                d=p;
			    is[k]=i;
			    js[k]=j;
              }
           }
      if(d==0)//d+1.0==1.0 lcc
        {
          free(is);
          free(js);
		  printf("\nerror:inverse matrix is not exist\n");
          return (0);
        }
      if(is[k]!=k)
        for(j=0;j<=n-1;j++)
          {
             u=k*n+j;v=is[k]*n+j;
             p=b[u];b[u]=b[v];b[v]=p;
          }
      if(js[k]!=k)
        for(i=0;i<=n-1;i++)
          {
            u=i*n+k;
            v=i*n+js[k];
            p=b[u];
            b[u]=b[v];b[v]=p;
          }
      l=k*n+k;
      b[l]=1.0/b[l];
      for(j=0;j<=n-1;j++)
        if(j!=k)
         {
            u=k*n+j;
            b[u]=b[u]*b[l];
         }
      for(i=0;i<=n-1;i++)
        if(i!=k)
          for(j=0;j<=n-1;j++)
            if(j!=k)
             {
               u=i*n+j;
               b[u]=b[u]-b[i*n+k]*b[k*n+j];
             }
      for(i=0;i<=n-1;i++)
         if(i!=k)
           {
             u=i*n+k;b[u]=-b[u]*b[l];
           }
    }     
    for(k=n-1;k>=0;k--)
      {
        if(js[k]!=k)
          for(j=0;j<=n-1;j++)
            {
               u=k*n+j;v=js[k]*n+j;
               p=b[u];
               b[u]=b[v];
               b[v]=p;
            }  
        if(is[k]!=k)
          for(i=0;i<=n-1;i++)
            {
               u=i*n+k;
			   v=i*n+is[k];	
               p=b[u];
			   b[u]=b[v];b[v]=p;
            }           
      }

     free(is);
     free(js);
     return(1);
}

double Mrem(double *a,int i,int j,int n)        
{      
	int k,m;      
	double  *pTemp = new double[(n-1)*(n-1)];        

	for(k=0;k<i;k++)      
	{      
		for(m=0;m<j;m++) 
		{
			pTemp[k*(n-1)+m] = a[k*n+m]; 
		}
		for(m=j;m<n-1;m++)  
		{
			pTemp[k*(n-1)+m] = a[k*n+m+1]; 
		}
	}      
	for(k=i;k<n-1;k++)      
	{      
		for(m=0;m<j;m++)
		{
			pTemp[k*(n-1)+m]=a[(k+1)*n+m];
		}
		for(m=j;m<n-1;m++)  
		{
			pTemp[k*(n-1)+m]=a[(k+1)*n+m+1];  
		}
	}      
	double  dResult = (((i+j)%2==1)?-1:1)*Mdet(pTemp,n-1);      
	delete[] pTemp;      
	return  dResult;      
} 

double Mdet(double *a,int n)   //求行列式的值    
{      
	if(n==1) 
	{
		return a[0];     
	}
	double sum=0;      
	for(int j=0;j<n;j++)
	{
		sum+=a[0*n+j]*Mrem(a,0,j,n); 
	}
	return sum;      
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

void Mequal(double *M,int m,int n,double a)
{
	   for (int i=0;i<m;i++)
   {
	   for (int j=0;j<n;j++)
	   {
		   *(M+i*n+j) = a;
	   }
   }
}

double Mmean(double *a,int m)
{
	double sum=0;
	for (int i=0;i<m;i++)
	{
		sum=sum+a[i];
	}
	return sum/m;
} 

int eejcb(double a[],int n,double v[],double eps,int jt) 
{ 
	int i,j,p,q,u,w,t,s,l; 
	double fm,cn,sn,omega,x,y,d; 
	l=1; 
	for (i=0; i<=n-1; i++) 
	{ 
		v[i*n+i]=1.0; 
		for (j=0; j<=n-1; j++) 
		{ 
			if (i!=j) 
			{ v[i*n+j]=0.0; } 
		} 
	} 
	while (1==1) 
	{ 
		fm=0.0; 
		for (i=0; i<=n-1; i++) 
		{ 
			for (j=0; j<=n-1; j++) 
			{ 
				d=fabs(a[i*n+j]); 
				if ((i!=j)&&(d>fm)) 
				{ 
					fm=d; 
					p=i; 
					q=j; 
				} 
			} 
		} 
		if (fm<eps) 
		{ return(1); } 
		if (l>jt) 
		{ return(-1); } 
		l=l+1; 
		u=p*n+q; 
		w=p*n+p; 
		t=q*n+p; 
		s=q*n+q; 
		x=-a[u]; 
		y=(a[s]-a[w])/2.0; 
		omega=x/sqrt(x*x+y*y); 
		if (y<0.0) 
		{ omega=-omega; } 
		sn=1.0+sqrt(1.0-omega*omega); 
		sn=omega/sqrt(2.0*sn); 
		cn=sqrt(1.0-sn*sn); 
		fm=a[w]; 
		a[w]=fm*cn*cn+a[s]*sn*sn+a[u]*omega; 
		a[s]=fm*sn*sn+a[s]*cn*cn-a[u]*omega; 
		a[u]=0.0; 
		a[t]=0.0; 
		for (j=0; j<=n-1; j++) 
		{ 
			if ((j!=p)&&(j!=q)) 
			{ 
				u=p*n+j; 
				w=q*n+j; 
				fm=a[u]; 
				a[u]=fm*cn+a[w]*sn; 
				a[w]=-fm*sn+a[w]*cn; 
			} 
		} 
		for (i=0; i<=n-1; i++) 
		{ 
			if ((i!=p)&&(i!=q)) 
			{ 
				u=i*n+p; 
				w=i*n+q; 
				fm=a[u]; 
				a[u]=fm*cn+a[w]*sn; 
				a[w]=-fm*sn+a[w]*cn; 
			} 
		} 
		for (i=0; i<=n-1; i++) 
		{ 
			u=i*n+p; 
			w=i*n+q; 
			fm=v[u]; 
			v[u]=fm*cn+v[w]*sn; 
			v[w]=-fm*sn+v[w]*cn; 
		} 
	} 
	return(1); 
}

void Munit(double* a,int n)
{
	for(int i=0;i<n;i++)
	{
		for(int j=0;j<n;j++)
		{a[i*n+j]=0.0;}
	}
	for(int i=0;i<n;i++)
	{a[i*n+i]=1.0;}
}
#pragma endregion

void m2qua_ned(double m[],double q[])
{
	double s[5]={0.0};
	s[4]=m[0*3+0]+m[1*3+1]+m[2*3+2];
	s[0]=1.0+s[4];
	s[1]=1.0+2.0*m[0*3+0]-s[4];
	s[2]=1.0+2.0*m[1*3+1]-s[4];
	s[3]=1.0+2.0*m[2*3+2]-s[4];
	int index=0;
	double max=s[0];
	for(int k=1;k<4;k++)
	{
		if(s[k]>max)
		{
			index=k;
			max=s[k];
		}
	}
switch(index)
	{
	case 0:
		q[0]=0.5*sqrt(s[0]);
		q[1]=0.25*(m[2*3+1]-m[1*3+2])/q[0];
		q[2]=0.25*(m[0*3+2]-m[2*3+0])/q[0];
		q[3]=0.25*(m[1*3+0]-m[0*3+1])/q[0];
		break;
	case 1:
		q[1]=0.5*sqrt(s[1]);
		q[2]=0.25*(m[1*3+0]+m[0*3+1])/q[1];
		q[3]=0.25*(m[0*3+2]+m[2*3+0])/q[1];
		q[0]=0.25*(m[2*3+1]-m[1*3+2])/q[1];
		break;
	case 2:
		q[2]=0.5*sqrt(s[2]);
		q[3]=0.25*(m[2*3+1]+m[1*3+2])/q[2];
		q[0]=0.25*(m[0*3+2]-m[2*3+0])/q[2];
		q[1]=0.25*(m[1*3+0]+m[0*3+1])/q[2];
		break;
	case 3:
		q[3]=0.5*sqrt(s[3]);
		q[0]=0.25*(m[1*3+0]-m[0*3+1])/q[3];
		q[1]=0.25*(m[0*3+2]+m[2*3+0])/q[3];
		q[2]=0.25*(m[2*3+1]+m[1*3+2])/q[3];
		break;
	}
}

void q2mat_ned(double qua[],double m[])
{
	double q11,q12,q13,q14,q22,q23,q24,q33,q34,q44;
	q11=qua[0]*qua[0]; q12=qua[0]*qua[1]; q13=qua[0]*qua[2]; q14=qua[0]*qua[3];
	q22=qua[1]*qua[1]; q23=qua[1]*qua[2]; q24=qua[1]*qua[3];
	q33=qua[2]*qua[2]; q34=qua[2]*qua[3];
	q44=qua[3]*qua[3];
	m[0*3+0]=q11+q22-q33-q44; m[0*3+1]=2*(q23-q14);     m[0*3+2]=2*(q24+q13);
	m[1*3+0]=2*(q23+q14);     m[1*3+1]=q11-q22+q33-q44; m[1*3+2]=2*(q34-q12);
	m[2*3+0]=2*(q24-q13);     m[2*3+1]=2*(q34+q12);     m[2*3+2]=q11-q22-q33+q44;
}

void m2att_ned(double m[],double a[])
{
	a[0]=atan2(m[2*3+1],m[2*3+2]);
	a[1]=asin(-m[2*3+0]);
	a[2]=atan2(m[1*3+0],m[0*3+0]);
}

void a2mat_ned(double att[],double m[])
{
	double phi=att[0];
	double theta=att[1];
	double psi=att[2];
	double cpsi=cos(psi); double spsi=sin(psi);
	double cthe=cos(theta); double sthe=sin(theta);
	double cphi=cos(phi); double sphi=sin(phi);
	double C1[9]={0.0};
	double C2[9]={0.0};
	double C3[9]={0.0};
	double m1[9]={0.0};
	double m2[9]={0.0};
	C1[0*3+0]=cpsi;  C1[0*3+1]=spsi;
	C1[1*3+0]=-spsi; C1[1*3+1]=cpsi;
	                                 C1[2*3+2]=1.0;
	C2[0*3+0]=cthe;                  C2[0*3+2]=-sthe;
	                 C2[1*3+1]=1.0;
	C2[2*3+0]=sthe;                  C2[2*3+2]=cthe;
	C3[0*3+0]=1.0;
	                 C3[1*3+1]=cphi; C3[1*3+2]=sphi;
	                 C3[2*3+1]=-sphi;C3[2*3+2]=cphi;
	Mmulnm(C3,C2,3,3,3,m1);
	Mmulnm(m1,C1,3,3,3,m2);
	Mtn(m2,3,3,m);
}
/*-----------------log define by DHF ----------------------*/
static FILE *fpoutdhf=NULL; 
extern void fopendhf(const char *file)
{
    if (!*file||!(fpoutdhf=fopen(file,"w"))) 
	{
		printf("can not open outfile crated by dhf!");
		fpoutdhf=NULL;
	}
}
extern void fclosedhf(void)
{
    if (fpoutdhf) fclose(fpoutdhf);
    fpoutdhf=NULL;
}
extern void outdhf(const char *format, ...)
{
    va_list ap;
    
    if (!fpoutdhf) return;
    va_start(ap,format); vfprintf(fpoutdhf,format,ap); va_end(ap);
    fflush(fpoutdhf);
}
/* end --------------------------------------------------------- */


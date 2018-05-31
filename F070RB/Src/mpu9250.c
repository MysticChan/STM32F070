#include "mpu9250.h"
#include "i2c.h"

#define PRINT_MESSAGE 1

struct MPU9250_RAW_Struct MPU9250_RAW;

HAL_StatusTypeDef MPU_Write_Byte(uint8_t addr,uint8_t reg,uint8_t data){
  HAL_StatusTypeDef status;
  status = HAL_I2C_Mem_Write(&hi2c1,addr,reg,I2C_MEMADD_SIZE_8BIT,&data,1,128);
  return status;
}

HAL_StatusTypeDef MPU_Read_Byte(uint8_t addr,uint8_t reg,uint8_t* data){
  HAL_StatusTypeDef status;
  status = HAL_I2C_Mem_Read(&hi2c1,addr,reg,I2C_MEMADD_SIZE_8BIT,data,1,128);
  return status;
}

//fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
HAL_StatusTypeDef MPU_Set_Gyro_Fsr(uint8_t fsr){
  return MPU_Write_Byte(MPU9250_ADDR,MPU_GYRO_CFG_REG,fsr<<3);
}

//fsr:0,±2g;1,±4g;2,±8g;3,±16g
HAL_StatusTypeDef MPU_Set_Accel_Fsr(uint8_t fsr){
  return MPU_Write_Byte(MPU9250_ADDR,MPU_ACCEL_CFG_REG,fsr<<3);
}

HAL_StatusTypeDef MPU_Set_LPF(uint16_t lpf)
{
	uint8_t data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
  return MPU_Write_Byte(MPU9250_ADDR,MPU_CFG_REG,data);
}

uint8_t MPU_Set_Rate(uint16_t rate)
{
	uint8_t data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	MPU_Write_Byte(MPU9250_ADDR,MPU_SAMPLE_RATE_REG,data);	//设置数字低通滤波器	0x68 0x19  0x13
 	return MPU_Set_LPF(rate/2);	//自动设置LPF为采样率的一半
}

HAL_StatusTypeDef MPU9250_Init(void){
  uint8_t res=0;
  if(MPU_Read_Byte(MPU9250_ADDR,MPU_DEVICE_ID_REG,&res) != HAL_OK){
#if PRINT_MESSAGE
    printf("Read reg.MPU_DEVICE_ID_REG faile.\r\n");
#endif
    return HAL_ERROR;
  }
  if(res != MPU6500_ID){
#if PRINT_MESSAGE
    printf("Wrong Device ID.\r\n");
#endif    
    return HAL_ERROR;
  }
  else{
#if PRINT_MESSAGE
    printf("MPU9250 ready.\r\n");
#endif 
    MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0x80);
    HAL_Delay(1);
    MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0x00);
    MPU_Set_Gyro_Fsr(3);
    MPU_Set_Accel_Fsr(3); 
    MPU_Write_Byte(MPU9250_ADDR,MPU_INT_EN_REG,0x00);
    MPU_Write_Byte(MPU9250_ADDR,MPU_USER_CTRL_REG,0x00);
    MPU_Write_Byte(MPU9250_ADDR,MPU_FIFO_EN_REG,0X00);
    MPU_Write_Byte(MPU9250_ADDR,MPU_INTBP_CFG_REG,0x82);
    MPU_Set_Rate(50);
    MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X01);  	//设置CLKSEL,PLL X轴为参考
    MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT2_REG,0X00);  	//加速度与陀螺仪都工作
  }
  MPU_Read_Byte(AK8963_ADDR,MAG_WIA,&res);
  if(res!=AK8963_ID){
#if PRINT_MESSAGE
    printf("Bypass mode is disable.\r\n");
#endif
    MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X40);
    return HAL_ERROR;
  }
  else{
#if PRINT_MESSAGE
    printf("AK8963 ready.\r\n");
#endif    
    MPU_Write_Byte(AK8963_ADDR,MAG_CNTL1,0X11);
  }
  HAL_Delay(30);
  MPU9250_RAW.Ready = RESET;
  return HAL_OK;
}

void MPU9250_GetRaw(void){
  uint8_t *p,i;
  uint8_t temp[14];
  p=(uint8_t*)&MPU9250_RAW.Gyro.z;
  HAL_I2C_Mem_Read(&hi2c1,MPU9250_ADDR,MPU_ACCEL_XOUTH_REG,I2C_MEMADD_SIZE_8BIT,temp,14,128);
  for(i=0;i<14;i++){
    p[i]=temp[13-i];
  }
  p=(uint8_t*)&MPU9250_RAW.Magneto.x;
  HAL_I2C_Mem_Read(&hi2c1,MPU9250_ADDR,MPU_ACCEL_XOUTH_REG,I2C_MEMADD_SIZE_8BIT,p,6,128);
  MPU_Write_Byte(AK8963_ADDR,MAG_CNTL1,0X11);
  MPU9250_RAW.Ready = SET;
}

void MPU9250_Convert(void){
  MPU9250_RAW.G.x = ((float)MPU9250_RAW.Gyro.x)*16*9.8/32768;
  MPU9250_RAW.G.y = ((float)MPU9250_RAW.Gyro.y)*16*9.8/32768;
  MPU9250_RAW.G.z = ((float)MPU9250_RAW.Gyro.z)*16*9.8/32768;
  
  MPU9250_RAW.A.x = ((float)MPU9250_RAW.Accel.x)*2000/32768;
  MPU9250_RAW.A.y = ((float)MPU9250_RAW.Accel.y)*2000/32768;
  MPU9250_RAW.A.z = ((float)MPU9250_RAW.Accel.z)*2000/32768;
  
  MPU9250_RAW.M.x = (float)MPU9250_RAW.Magneto.x;
  MPU9250_RAW.M.y = (float)MPU9250_RAW.Magneto.y;
  MPU9250_RAW.M.z = (float)MPU9250_RAW.Magneto.z;
}


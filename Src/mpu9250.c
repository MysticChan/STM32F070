#include "mpu9250.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "elecompass.h"
struct TransStruct IMU_Transmit;
struct MPU9250_RAW_Struct MPU9250_RAW;
SemaphoreHandle_t BinarySemaphore_MPU9250;

type_imu IMU_Data;
type_ahrs IMU_Angle;

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
 	return MPU_Set_LPF(rate>>1);	//自动设置LPF为采样率的一半
}

HAL_StatusTypeDef MPU9250_Init(void){
  uint8_t res=0;
  MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0x80);
  HAL_Delay(5);
  MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0x00);
  MPU_Read_Byte(MPU9250_ADDR,MPU_DEVICE_ID_REG,&res);
  if(res != MPU6500_ID){
//    printf("Can not find MPU9250.\r\n");   
    return HAL_ERROR;
  }
  else{
//    printf("MPU9250 device ID: 0x%c%c.\r\n",((res&0xf0)>>4)+'0',(res&0x0f)+'0');
    MPU_Set_Gyro_Fsr(1);
    MPU_Set_Accel_Fsr(0); 
    MPU_Write_Byte(MPU9250_ADDR,MPU_INT_EN_REG,0x00);
    MPU_Write_Byte(MPU9250_ADDR,MPU_USER_CTRL_REG,0x00);
    MPU_Write_Byte(MPU9250_ADDR,MPU_FIFO_EN_REG,0X00);
    MPU_Write_Byte(MPU9250_ADDR,MPU_INTBP_CFG_REG,0x82);
    MPU_Set_Rate(200);
    MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X01);  	//设置CLKSEL,PLL X轴为参考
    MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT2_REG,0X00);  	//加速度与陀螺仪都工作
  }
  MPU_Read_Byte(AK8963_ADDR,MAG_WIA,&res);
  if(res!=AK8963_ID){
//     printf("Can not find AK8963\r\n");
    MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X40);
    return HAL_ERROR;
  }
  else{
//    printf("AK8963 device ID: 0x%c%c.\r\n",((res&0xf0)>>4)+'0',(res&0x0f)+'0');
    MPU_Write_Byte(AK8963_ADDR,MAG_CNTL1,0x11);
  }
  HAL_Delay(30);
  
  IMU_Transmit.T.h = 0;
  IMU_Transmit.T.m = 0;
  IMU_Transmit.T.s = 0;
  IMU_Transmit.T.ms = 0;
  
  IMU_Angle.azimuth = 0;
  IMU_Angle.pitch = 0;
  IMU_Angle.roll = 0;
  IMU_Angle.tilt = 0;
  IMU_Angle.time = 0;
  IMU_Angle.yaw = 0;
  
  BinarySemaphore_MPU9250 = xSemaphoreCreateBinary();
  if(BinarySemaphore_MPU9250 != NULL)
  {
    HAL_TIM_Base_Start_IT(&htim15);
    xTaskCreate(vMPU9250DataProcess_task,
                "MPU9250DataProcess",
                500,
                NULL,
                4,
                NULL); 
    
  }
  HAL_TIM_Base_Start_IT(&htim15);
  HAL_TIM_Base_Start_IT(&htim3);
  return HAL_OK;
}



void vMPU9250DataProcess_task(void *pvParameters)
{
  uint8_t i,*q;
  uint8_t temp;
  short *p;
  q = MPU9250_RAW.i2c_Rx;
  while(1)
  {
    xSemaphoreTake(BinarySemaphore_MPU9250,portMAX_DELAY);
    HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
    HAL_I2C_Mem_Read(&hi2c1,MPU9250_ADDR,MPU_ACCEL_XOUTH_REG,I2C_MEMADD_SIZE_8BIT,q,14,128);
    for(i=0;i<7;i++)
    {
      temp = q[i];
      q[i] = q[13-i];
      q[13-i] = temp;
    }
    p = (short*)q; 
    
//    IMU_Transmit.Gyro.z = p[0];
//    IMU_Transmit.Gyro.y = p[1];
//    IMU_Transmit.Gyro.x = p[2];
//    IMU_Transmit.Accel.z = p[4];
//    IMU_Transmit.Accel.y = p[5];
//    IMU_Transmit.Accel.x = p[6];
//    MPU9250_RAW.Gyro.z      = (float)p[0]*2000/32768;
//    MPU9250_RAW.Gyro.y      = (float)p[1]*2000/32768;
//    MPU9250_RAW.Gyro.x      = (float)p[2]*2000/32768;
//    MPU9250_RAW.Temperature = (float)p[3]/100;
//    MPU9250_RAW.Accel.z     = (float)p[4]*16*9.8/32768;
//    MPU9250_RAW.Accel.y     = (float)p[5]*16*9.8/32768;
//    MPU9250_RAW.Accel.x     = (float)p[6]*16*9.8/32768;
   
    IMU_Data.gz      = (float)p[0]*500/32768;
    IMU_Data.gy      = (float)p[1]*500/32768;
    IMU_Data.gx      = (float)p[2]*500/32768;
    IMU_Data.az     = (float)p[4]*2/32768;
    IMU_Data.ay     = (float)p[5]*2/32768;
    IMU_Data.ax     = (float)p[6]*2/32768;
  
    HAL_I2C_Mem_Read(&hi2c1,AK8963_ADDR,MAG_XOUT_L,I2C_MEMADD_SIZE_8BIT,q,6,128);
//    IMU_Transmit.Magneto.x = p[0];
//    IMU_Transmit.Magneto.y = p[1];
//    IMU_Transmit.Magneto.z = p[2];
//    MPU9250_RAW.Magneto.x   = (float)p[0]*0.15;
//    MPU9250_RAW.Magneto.y   = (float)p[1]*0.15;
//    MPU9250_RAW.Magneto.z   = (float)p[2]*0.15;
    IMU_Data.mx   = (float)p[0]*0.15;
    IMU_Data.my   = (float)p[1]*0.15;
    IMU_Data.mz   = (float)p[2]*0.15;
    MPU_Write_Byte(AK8963_ADDR,MAG_CNTL1,0X11);    
//    HAL_UART_Transmit(&huart2,(uint8_t*)&IMU_Transmit.T.h,26,128);
    ahrsupdate(IMU_Data,IMU_Angle);
//    HAL_UART_Transmit(&huart2,(uint8_t*)&IMU_Transmit.T.ms,2,128);
//    printf("\r\n");
//printf("%d:%d:%d.%d\r\n",IMU_Transmit.T.h,IMU_Transmit.T.m,IMU_Transmit.T.s,IMU_Transmit.T.ms);
//    printf("ax %.3f ay %.3f az %.3f\r\n",IMU_Data.ax,IMU_Data.ay,IMU_Data.az);
//    printf("gx %.3f gy %.3f gz %.3f\r\n",IMU_Data.gx,IMU_Data.gy,IMU_Data.gz);
//    printf("mx %.3f my %.3f mz %.3f\r\n",IMU_Data.mx,IMU_Data.my,IMU_Data.mz);
//    printf("%d:%d:%d.%04d roll %.3f pitch %.3f yaw %.3f \r\n",IMU_Transmit.T.h,IMU_Transmit.T.m,IMU_Transmit.T.s,IMU_Transmit.T.ms,IMU_Angle.roll,IMU_Angle.pitch,IMU_Angle.yaw);
//    printf("\r\n");
//    printf("%04d %.4f %.4f %.4f %.4f\r\n",IMU_Transmit.T.ms,IMU_Angle.roll,IMU_Angle.pitch,IMU_Angle.yaw,IMU_Angle.tilt);
//     printf("%04d %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f /n",IMU_Transmit.T.ms,     
//            IMU_Data.ax,IMU_Data.ay,IMU_Data.az,
//            IMU_Data.gx,IMU_Data.gy,IMU_Data.gz,
//            IMU_Data.mx,IMU_Data.my,IMU_Data.mz);

  } 
}





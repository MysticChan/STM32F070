#include "JY901.h"
#include "usart.h"
#include "tim.h"
#include "string.h"
uint8_t JY901_Data[JY901_DataSize];
//struct JY901_DataStruct JY901_int;


struct JY901_DataStruct JY901_RAW;

void GetData(){
	JY901_Data[JY901_RAW.Cnt] = RxData;
	switch(JY901_RAW.Cnt)
	{
		case 0:{
			if(JY901_Data[JY901_RAW.Cnt] == 0x55)
				JY901_RAW.Cnt++;
			else
				JY901_RAW.Cnt = 0;
		}break;
		case 1:{
			if(JY901_Data[JY901_RAW.Cnt] == 0x50){
        TimerStart();
        JY901_RAW.Cnt++;
      }		
			else if(JY901_Data[JY901_RAW.Cnt] == 0x55){
				JY901_RAW.Cnt = 1;
				JY901_Data[0] = 0x55;
			}
			else JY901_RAW.Cnt = 0;
		}break;
		default:{
      JY901_RAW.Cnt++;
      if((JY901_RAW.Cnt+1)%12==0)
        JY901_RAW.Cnt++;
    }break;
	}
	if(JY901_RAW.Cnt>=JY901_DataSize){
		JY901_RAW.Cnt=0;
		JY901_RAW.DataReady = SET;
	}
	else
		HAL_UART_Receive_IT(&huart4,&RxData,1);
}


void DataConvert(void){
  uint8_t *p1,*p2,*p3,*p4;
	uint8_t sum[4],i;
  p1=&JY901_Data[0];
  p2=&JY901_Data[12];
  p3=&JY901_Data[24];
  p4=&JY901_Data[36];
  for(i=0;i<4;i++){
    sum[i]=0;
  }
  for(i=0;i<10;i++){
    sum[0]+=p1[i];
    sum[1]+=p2[i];
    sum[2]+=p3[i];
    sum[3]+=p4[i];
  }
  if(sum[0]==p1[10]){
    JY901_RAW.TimeRaw = (TimeFormat*)p1;
    JY901_RAW.Time.year = JY901_RAW.TimeRaw->year;
    JY901_RAW.Time.month = JY901_RAW.TimeRaw->month;
    JY901_RAW.Time.day = JY901_RAW.TimeRaw->day;
    JY901_RAW.Time.hour = JY901_RAW.TimeRaw->hour;
    JY901_RAW.Time.minute = JY901_RAW.TimeRaw->minute;
    JY901_RAW.Time.second = JY901_RAW.TimeRaw->second;
    JY901_RAW.Time.millisecond = JY901_RAW.TimeRaw->millisecond;
  }
  if(sum[1]==p2[10]){
    JY901_RAW.GyroRaw = (RawDataFormat*)p2;
    JY901_RAW.Gyro.x = ((float)JY901_RAW.GyroRaw->x)*16*g/32768;
    JY901_RAW.Gyro.y = ((float)JY901_RAW.GyroRaw->y)*16*g/32768;
    JY901_RAW.Gyro.z = ((float)JY901_RAW.GyroRaw->z)*16*g/32768;
    JY901_RAW.Temp = JY901_RAW.GyroRaw->temp;
  }
  if(sum[2]==p3[10]){
    JY901_RAW.AccelRaw = (RawDataFormat*)p3;
    JY901_RAW.Accel.x = ((float)JY901_RAW.AccelRaw->x)*2000/32768;
    JY901_RAW.Accel.y = ((float)JY901_RAW.AccelRaw->y)*2000/32768;
    JY901_RAW.Accel.z = ((float)JY901_RAW.AccelRaw->z)*2000/32768;
    JY901_RAW.Temp = JY901_RAW.AccelRaw->temp;
  }
  if(sum[3]==p4[10]){
    JY901_RAW.MagnetoRaw = (RawDataFormat*)p4;
    JY901_RAW.Magneto.x = (float)JY901_RAW.MagnetoRaw->x;
    JY901_RAW.Magneto.y = (float)JY901_RAW.MagnetoRaw->y;
    JY901_RAW.Magneto.z = (float)JY901_RAW.MagnetoRaw->z;
    JY901_RAW.Temp = JY901_RAW.MagnetoRaw->temp;
  }
  JY901_RAW.Temperature = (float)JY901_RAW.Temp/100;
  TimerStop();
	JY901_RAW.DataReady = RESET;
}

void PrintTable(void){
  uint8_t *p,temp[16],i;
  uint16_t *r,value; 
  p=temp;
  
  r=(uint16_t*)&JY901_RAW.GyroRaw->x;
  value = *r;
  for(i=0;i<16;i++){
    if(value&0x8000)
      *(p+i)='1';
    else
      *(p+i)='0';
    value<<=1;
  }
  HAL_UART_Transmit(&huart2,(uint8_t*)"Gyro.x = ",9,128);
  HAL_UART_Transmit(&huart2,p,16,128);
  HAL_UART_Transmit(&huart2,(uint8_t*)"\r\n",2,128);

  r=(uint16_t*)&JY901_RAW.GyroRaw->y;
  value = *r;
  for(i=0;i<16;i++){
    if(value&0x8000)
      *(p+i)='1';
    else
      *(p+i)='0';
    value<<=1;
  }
  HAL_UART_Transmit(&huart2,(uint8_t*)"Gyro.y = ",9,128);
  HAL_UART_Transmit(&huart2,p,16,128);
  HAL_UART_Transmit(&huart2,(uint8_t*)"\r\n",2,128);

  r=(uint16_t*)&JY901_RAW.GyroRaw->z;
  value = *r;
  for(i=0;i<16;i++){
    if(value&0x8000)
      *(p+i)='1';
    else
      *(p+i)='0';
    value<<=1;
  }
  HAL_UART_Transmit(&huart2,(uint8_t*)"Gyro.z = ",9,128);
  HAL_UART_Transmit(&huart2,p,16,128);
  HAL_UART_Transmit(&huart2,(uint8_t*)"\r\n",2,128);

  r=(uint16_t*)&JY901_RAW.AccelRaw->x;
  value = *r;
  for(i=0;i<16;i++){
    if(value&0x8000)
      *(p+i)='1';
    else
      *(p+i)='0';
    value<<=1;
  }
  HAL_UART_Transmit(&huart2,(uint8_t*)"Accel.x = ",10,128);
  HAL_UART_Transmit(&huart2,p,16,128);
  HAL_UART_Transmit(&huart2,(uint8_t*)"\r\n",2,128);
  
  r=(uint16_t*)&JY901_RAW.AccelRaw->y;
  value = *r;
  for(i=0;i<16;i++){
    if(value&0x8000)
      *(p+i)='1';
    else
      *(p+i)='0';
    value<<=1;
  }
  HAL_UART_Transmit(&huart2,(uint8_t*)"Accel.y = ",10,128);
  HAL_UART_Transmit(&huart2,p,16,128);
  HAL_UART_Transmit(&huart2,(uint8_t*)"\r\n",2,128);
  
  r=(uint16_t*)&JY901_RAW.AccelRaw->z;
  value = *r;
  for(i=0;i<16;i++){
    if(value&0x8000)
      *(p+i)='1';
    else
      *(p+i)='0';
    value<<=1;
  }
  HAL_UART_Transmit(&huart2,(uint8_t*)"Accel.z = ",10,128);
  HAL_UART_Transmit(&huart2,p,16,128);
  HAL_UART_Transmit(&huart2,(uint8_t*)"\r\n",2,128);
  
  r=(uint16_t*)&JY901_RAW.MagnetoRaw->x;
  value = *r;
  for(i=0;i<16;i++){
    if(value&0x8000)
      *(p+i)='1';
    else
      *(p+i)='0';
    value<<=1;
  }
  HAL_UART_Transmit(&huart2,(uint8_t*)"Magneto.x = ",12,128);
  HAL_UART_Transmit(&huart2,p,16,128);
  HAL_UART_Transmit(&huart2,(uint8_t*)"\r\n",2,128);
  
  r=(uint16_t*)&JY901_RAW.MagnetoRaw->y;
  value = *r;
  for(i=0;i<16;i++){
    if(value&0x8000)
      *(p+i)='1';
    else
      *(p+i)='0';
    value<<=1;
  }
  HAL_UART_Transmit(&huart2,(uint8_t*)"Magneto.y = ",12,128);
  HAL_UART_Transmit(&huart2,p,16,128);
  HAL_UART_Transmit(&huart2,(uint8_t*)"\r\n",2,128);
  
  r=(uint16_t*)&JY901_RAW.MagnetoRaw->z;
  value = *r;
  for(i=0;i<16;i++){
    if(value&0x8000)
      *(p+i)='1';
    else
      *(p+i)='0';
    value<<=1;
  }
  HAL_UART_Transmit(&huart2,(uint8_t*)"Magneto.z = ",12,128);
  HAL_UART_Transmit(&huart2,p,16,128);
  HAL_UART_Transmit(&huart2,(uint8_t*)"\r\n",2,128);
  
}

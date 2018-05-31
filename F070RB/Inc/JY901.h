#ifndef __JY901_H
#define __JY901_H

#include "stm32f0xx_hal.h"
#define g 9.8	
#define JY901_DataSize 48
typedef struct{
	float x;
	float y;
	float z;
}floatAxisStruct;

typedef struct{
	short x;
	short y;
	short z;
}ShortAxisStruct;

typedef struct {
  uint8_t head;
  uint8_t type;
  short x;
  short y;
  short z;
  short temp;
  uint8_t sum;
}RawDataFormat;

typedef struct{
  uint8_t head;
  uint8_t type;
  uint8_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint16_t millisecond;
  uint8_t sum;
}TimeFormat;

typedef struct{
  uint8_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint16_t millisecond;
}TimeStruct;

struct JY901_DataStruct{
  TimeFormat *TimeRaw;
	RawDataFormat *GyroRaw;
	RawDataFormat *AccelRaw;
	RawDataFormat *MagnetoRaw;
  TimeStruct Time;
  floatAxisStruct Gyro;
  floatAxisStruct Accel;
  floatAxisStruct Magneto;
  short Temp;
  float Temperature;
  uint8_t DataReady;
  uint8_t Cnt;
};
extern struct JY901_DataStruct JY901_RAW;

enum JY901PackType{
	GYRO = 0x51,
	ACCEL = 0x52,
	MAGNETO = 0x53
};
void GetData(void);
void DataConvert(void);
void PrintTable(void);
  

#endif

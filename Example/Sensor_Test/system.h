#ifndef __SYSTEM_H__
#define __SYSTEM_H__

#include "stm32f7xx_hal.h"


void System_Launch(void);

typedef enum
{
  TARGET_NUCLEO,
  TARGET_BLUECOIN,
  TARGET_SENSORTILE,
  TARGETS_NUMBER
} TargetType_t;


typedef struct
{
  TargetType_t BoardType;
  int32_t NumTempSensors;


  void *HandlePressSensor;
  void *HandleHumSensor;

  int32_t HWAdvanceFeatures;
  void *HandleAccSensor;
  void *HandleGyroSensor;
  void *HandleMagSensor;
  
  int32_t NumMicSensors;

  uint8_t LedStatus;
  uint8_t bnrg_expansion_board;
} TargetFeatures_t;

#endif //__SYSTEM_H__

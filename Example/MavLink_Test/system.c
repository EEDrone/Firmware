#include "system.h"
#include "config.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "BSP_accelero.h"
#include "BSP_gyro.h"
#include "mavlink.h"
#include <stdio.h>


extern void vRegisterSampleCLICommands( void );
extern void vUARTCommandConsoleStart( uint16_t usStackSize, UBaseType_t uxPriority );
extern void vOutputString( const char * const pcMessage );
TargetFeatures_t TargetBoardFeatures;
  SensorAxes_t GYR_Value;
void Test_Task(void const * argument)
{
  for(;;)
  { 
    osDelay(1000);
 
  }
  /* USER CODE END StartDefaultTask */
}


void Main_Task (void const * argument){
  
   TickType_t xLastWakeTime;
 const TickType_t xFrequency = 1000;
	 // Initialise the xLastWakeTime variable with the current time.
	 xLastWakeTime = xTaskGetTickCount ();
  
  for(;;){
     vTaskDelayUntil( &xLastWakeTime, xFrequency );
        
     GYR_Value.AXIS_X=3;//测试
        char pcWriteBuffer[100];
        char mumber[10];
        strcpy( pcWriteBuffer, "GYR_X:" );        
        sprintf(mumber,"%d",GYR_Value.AXIS_X);
        strncat( pcWriteBuffer, mumber, strlen( mumber ) );
        strncat( pcWriteBuffer, "\r\n", strlen( "\r\n" ) );
        vOutputString(pcWriteBuffer);
  }
}

void Init_Task (void const * argument){
    char  pass = 1;
   
  if(pass){
    //读取，结算，控制主任务
   //  xTaskCreate((TaskFunction_t)Main_Task, MAIN_TASK_NAME,MAIN_TASK_STACKSIZE, NULL,MAIN_TASK_PRI, NULL);
  }
      //Mavlink任务
  xTaskCreate((TaskFunction_t)Mavlink_Task, MAVLINK_TASK_NAME,MAVLINK_TASK_STACKSIZE, NULL,MAVLINK_TASK_PRI, NULL);
  
  
 vTaskDelete( NULL );
}


/* Public functions */
void System_Launch(void)
{
  //初始化任务
  xTaskCreate((TaskFunction_t)Init_Task, INIT_TASK_NAME,INIT_TASK_STACKSIZE, NULL,INIT_TASK_PRI, NULL);
  //测试任务
  xTaskCreate((TaskFunction_t)Test_Task, TEST_TASK_NAME,TEST_TASK_STACKSIZE, NULL,TEST_TASK_PRI, NULL);

  
      vUARTCommandConsoleStart( 1000, 1 );
    vRegisterSampleCLICommands();
   
}
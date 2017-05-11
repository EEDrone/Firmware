/**
 
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "config.h"
/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);

extern void MX_FATFS_Init(void);
extern void MX_LWIP_Init(void);
extern void MX_USB_DEVICE_Init(void);
extern void vRegisterSampleCLICommands( void );
extern void vUARTCommandConsoleStart( uint16_t usStackSize, UBaseType_t uxPriority );
void MX_FREERTOS_Init(void); 





void MX_FREERTOS_Init(void) {
 
  osThreadDef(defaultTask, StartDefaultTask, osPriorityAboveNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);
  
  

  
vUARTCommandConsoleStart( 1000, 1 );
vRegisterSampleCLICommands();

}




void StartDefaultTask(void const * argument)
{
int i=0; 
char pcWriteBuffer[10];
  for(;;)
  { osDelay(1000);
   i++;
    sprintf(pcWriteBuffer,"%d",i);
  vOutputString(pcWriteBuffer);
  // vOutputString("123");
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

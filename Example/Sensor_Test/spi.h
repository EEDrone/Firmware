/**
  ******************************************************************************
  * File Name          : SPI.h
  * Description        : This file provides code for the configuration
  *                      of the SPI instances.
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __spi_H
#define __spi_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "main.h"
#include "accelerometer.h"
#include "gyroscope.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

   #define LSM303AGR_ACC_WHO_AM_I         0x33
#define LSM303AGR_MAG_WHO_AM_I         0x40
#define HTS221_WHO_AM_I_VAL         (uint8_t)0xBC

   typedef enum
{
//  TEMPERATURE_SENSORS_AUTO = -1, /* Always first element and equal to -1 */
  LSM6DSM = 0,                  /* LSM6DSM. */
  LSM303AGR_X,                  /* LSM303AGR Accelerometer */
  LSM303AGR_M,                  /* LSM303AGR Magnetometer */
  LPS22HB                       /* LPS22HB */
} SPI_Device_t;


extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi4;
extern SPI_HandleTypeDef hspi5;




/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

extern void Error_Handler(void);

void MX_SPI2_Init(void);
void MX_SPI4_Init(void);
void MX_SPI5_Init(void);


#define LSM6DSM_INT2_GPIO_PORT           GPIOA
#define LSM6DSM_INT2_GPIO_CLK_ENABLE()   __GPIOA_CLK_ENABLE()
#define LSM6DSM_INT2_GPIO_CLK_DISABLE()  __GPIOA_CLK_DISABLE()
#define LSM6DSM_INT2_PIN                 GPIO_PIN_2
#define LSM6DSM_INT2_EXTI_IRQn           EXTI2_IRQn

                                              
//#define SENSORTILE_SENSORS_SPI                    SPI2

//#define SENSORTILE_SENSORS_SPI_Port               GPIOB
//#define SENSORTILE_SENSORS_SPI_MOSI_Pin           GPIO_PIN_15
//#define SENSORTILE_SENSORS_SPI_SCK_Pin            GPIO_PIN_13

#define SENSORTILE_SENSORS_SPI_CLK_ENABLE()       __SPI2_CLK_ENABLE()
#define SENSORTILE_SENSORS_SPI_GPIO_CLK_ENABLE()  __GPIOB_CLK_ENABLE()

#define SENSORTILE_LSM6DSM_SPI_CS_Port	          GPIOB
#define SENSORTILE_LSM6DSM_SPI_CS_Pin     	  GPIO_PIN_12
#define SENSORTILE_LSM6DSM_SPI_CS_GPIO_CLK_ENABLE()  __GPIOB_CLK_ENABLE()
                                              
#define SENSORTILE_LSM303AGR_X_SPI_CS_Port	  GPIOC
#define SENSORTILE_LSM303AGR_X_SPI_CS_Pin     	  GPIO_PIN_4
#define SENSORTILE_LSM303AGR_X_SPI_CS_GPIO_CLK_ENABLE()  __GPIOC_CLK_ENABLE()
                                              
#define SENSORTILE_LSM303AGR_M_SPI_CS_Port	  GPIOB
#define SENSORTILE_LSM303AGR_M_SPI_CS_Pin     	  GPIO_PIN_1
#define SENSORTILE_LSM303AGR_M_SPI_CS_GPIO_CLK_ENABLE()  __GPIOB_CLK_ENABLE()
                                              
#define SENSORTILE_LPS22HB_SPI_CS_Port	          GPIOA
#define SENSORTILE_LPS22HB_SPI_CS_Pin     	  GPIO_PIN_3
#define SENSORTILE_LPS22HB_SPI_CS_GPIO_CLK_ENABLE()  __GPIOA_CLK_ENABLE()

DrvStatusTypeDef Sensor_IO_I2C_Init( void );
DrvStatusTypeDef Sensor_IO_SPI_Init( void );
uint8_t Sensor_IO_Read( void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead );
uint8_t Sensor_IO_I2C_Read( void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead );
uint8_t Sensor_IO_SPI_Read( void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead );
uint8_t Sensor_IO_Write( void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite );
uint8_t Sensor_IO_I2C_Write( void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite );
uint8_t Sensor_IO_SPI_Write( void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite );
uint8_t Sensor_IO_SPI_CS_Init_All(void);
uint8_t Sensor_IO_SPI_CS_Init(void *handle);
uint8_t Sensor_IO_SPI_CS_Enable(void *handle);
uint8_t Sensor_IO_SPI_CS_Disable(void *handle);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ spi_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

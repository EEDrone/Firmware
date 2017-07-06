/**
  ******************************************************************************
  * File Name          : SPI.c
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

/* Includes ------------------------------------------------------------------*/
#include "spi.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

SPI_HandleTypeDef SPI_Sensor_Handle;
SPI_HandleTypeDef hspi4;
SPI_HandleTypeDef hspi5;

/* SPI2 init function */
void MX_SPI2_Init(void)
{

  SPI_Sensor_Handle.Instance = SPI2;
  SPI_Sensor_Handle.Init.Mode = SPI_MODE_MASTER;
  SPI_Sensor_Handle.Init.Direction = SPI_DIRECTION_2LINES;
  SPI_Sensor_Handle.Init.DataSize = SPI_DATASIZE_4BIT;
  SPI_Sensor_Handle.Init.CLKPolarity = SPI_POLARITY_LOW;
  SPI_Sensor_Handle.Init.CLKPhase = SPI_PHASE_1EDGE;
  SPI_Sensor_Handle.Init.NSS = SPI_NSS_SOFT;
  SPI_Sensor_Handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  SPI_Sensor_Handle.Init.FirstBit = SPI_FIRSTBIT_MSB;
  SPI_Sensor_Handle.Init.TIMode = SPI_TIMODE_DISABLE;
  SPI_Sensor_Handle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  SPI_Sensor_Handle.Init.CRCPolynomial = 7;
  SPI_Sensor_Handle.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  SPI_Sensor_Handle.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&SPI_Sensor_Handle) != HAL_OK)
  {
    Error_Handler();
  }

}
/* SPI4 init function */
void MX_SPI4_Init(void)
{

  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 7;
  hspi4.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }

}
/* SPI5 init function */
void MX_SPI5_Init(void)
{

  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 7;
  hspi5.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi5.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(spiHandle->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspInit 0 */

  /* USER CODE END SPI2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_SPI2_CLK_ENABLE();
  
    /**SPI2 GPIO Configuration    
    PC2     ------> SPI2_MISO
    PC3     ------> SPI2_MOSI
    PD3     ------> SPI2_SCK 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI2_MspInit 1 */

  /* USER CODE END SPI2_MspInit 1 */
  }
  else if(spiHandle->Instance==SPI4)
  {
  /* USER CODE BEGIN SPI4_MspInit 0 */

  /* USER CODE END SPI4_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_SPI4_CLK_ENABLE();
  
    /**SPI4 GPIO Configuration    
    PE12     ------> SPI4_SCK
    PE13     ------> SPI4_MISO
    PE14     ------> SPI4_MOSI 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI4;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI4_MspInit 1 */

  /* USER CODE END SPI4_MspInit 1 */
  }
  else if(spiHandle->Instance==SPI5)
  {
  /* USER CODE BEGIN SPI5_MspInit 0 */

  /* USER CODE END SPI5_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_SPI5_CLK_ENABLE();
  
    /**SPI5 GPIO Configuration    
    PF7     ------> SPI5_SCK
    PF8     ------> SPI5_MISO
    PF11     ------> SPI5_MOSI 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI5;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI5_MspInit 1 */

  /* USER CODE END SPI5_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspDeInit 0 */

  /* USER CODE END SPI2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI2_CLK_DISABLE();
  
    /**SPI2 GPIO Configuration    
    PC2     ------> SPI2_MISO
    PC3     ------> SPI2_MOSI
    PD3     ------> SPI2_SCK 
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_2|GPIO_PIN_3);

    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_3);

  /* USER CODE BEGIN SPI2_MspDeInit 1 */

  /* USER CODE END SPI2_MspDeInit 1 */
  }
  else if(spiHandle->Instance==SPI4)
  {
  /* USER CODE BEGIN SPI4_MspDeInit 0 */

  /* USER CODE END SPI4_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI4_CLK_DISABLE();
  
    /**SPI4 GPIO Configuration    
    PE12     ------> SPI4_SCK
    PE13     ------> SPI4_MISO
    PE14     ------> SPI4_MOSI 
    */
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14);

  /* USER CODE BEGIN SPI4_MspDeInit 1 */

  /* USER CODE END SPI4_MspDeInit 1 */
  }
  else if(spiHandle->Instance==SPI5)
  {
  /* USER CODE BEGIN SPI5_MspDeInit 0 */

  /* USER CODE END SPI5_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI5_CLK_DISABLE();
  
    /**SPI5 GPIO Configuration    
    PF7     ------> SPI5_SCK
    PF8     ------> SPI5_MISO
    PF11     ------> SPI5_MOSI 
    */
    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_11);

  /* USER CODE BEGIN SPI5_MspDeInit 1 */

  /* USER CODE END SPI5_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
uint8_t Sensor_IO_SPI_CS_Init_All(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  
  SENSORTILE_LSM6DSM_SPI_CS_GPIO_CLK_ENABLE();
  GPIO_InitStruct.Pin = SENSORTILE_LSM6DSM_SPI_CS_Pin;
  HAL_GPIO_Init(SENSORTILE_LSM6DSM_SPI_CS_Port, &GPIO_InitStruct);
  HAL_GPIO_WritePin(SENSORTILE_LSM6DSM_SPI_CS_Port, SENSORTILE_LSM6DSM_SPI_CS_Pin, GPIO_PIN_SET);
  
  SENSORTILE_LSM303AGR_X_SPI_CS_GPIO_CLK_ENABLE();
  GPIO_InitStruct.Pin = SENSORTILE_LSM303AGR_X_SPI_CS_Pin;
  HAL_GPIO_Init(SENSORTILE_LSM303AGR_X_SPI_CS_Port, &GPIO_InitStruct);
  HAL_GPIO_WritePin(SENSORTILE_LSM303AGR_X_SPI_CS_Port, SENSORTILE_LSM303AGR_X_SPI_CS_Pin, GPIO_PIN_SET);
  
  SENSORTILE_LSM303AGR_M_SPI_CS_GPIO_CLK_ENABLE();
  GPIO_InitStruct.Pin = SENSORTILE_LSM303AGR_M_SPI_CS_Pin;
  HAL_GPIO_Init(SENSORTILE_LSM303AGR_M_SPI_CS_Port, &GPIO_InitStruct);
  HAL_GPIO_WritePin(SENSORTILE_LSM303AGR_M_SPI_CS_Port, SENSORTILE_LSM303AGR_M_SPI_CS_Pin, GPIO_PIN_SET);
  
  SENSORTILE_LPS22HB_SPI_CS_GPIO_CLK_ENABLE();
  GPIO_InitStruct.Pin = SENSORTILE_LPS22HB_SPI_CS_Pin;
  HAL_GPIO_Init(SENSORTILE_LPS22HB_SPI_CS_Port, &GPIO_InitStruct);
  HAL_GPIO_WritePin(SENSORTILE_LPS22HB_SPI_CS_Port, SENSORTILE_LPS22HB_SPI_CS_Pin, GPIO_PIN_SET);

  return COMPONENT_OK;
}

uint8_t Sensor_IO_SPI_CS_Init(void *handle)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  
  switch(ctx->spiDevice)
  {
  case LSM6DSM:
    SENSORTILE_LSM6DSM_SPI_CS_GPIO_CLK_ENABLE();
    GPIO_InitStruct.Pin = SENSORTILE_LSM6DSM_SPI_CS_Pin;
    HAL_GPIO_Init(SENSORTILE_LSM6DSM_SPI_CS_Port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(SENSORTILE_LSM6DSM_SPI_CS_Port, SENSORTILE_LSM6DSM_SPI_CS_Pin, GPIO_PIN_SET);
    break;
  case LSM303AGR_X:
    SENSORTILE_LSM303AGR_X_SPI_CS_GPIO_CLK_ENABLE();
    GPIO_InitStruct.Pin = SENSORTILE_LSM303AGR_X_SPI_CS_Pin;
    HAL_GPIO_Init(SENSORTILE_LSM303AGR_X_SPI_CS_Port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(SENSORTILE_LSM303AGR_X_SPI_CS_Port, SENSORTILE_LSM303AGR_X_SPI_CS_Pin, GPIO_PIN_SET);
    break;
  case LSM303AGR_M:
    SENSORTILE_LSM303AGR_M_SPI_CS_GPIO_CLK_ENABLE();
    GPIO_InitStruct.Pin = SENSORTILE_LSM303AGR_M_SPI_CS_Pin;
    HAL_GPIO_Init(SENSORTILE_LSM303AGR_M_SPI_CS_Port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(SENSORTILE_LSM303AGR_M_SPI_CS_Port, SENSORTILE_LSM303AGR_M_SPI_CS_Pin, GPIO_PIN_SET);
    break;
  case LPS22HB:
    SENSORTILE_LPS22HB_SPI_CS_GPIO_CLK_ENABLE();
    GPIO_InitStruct.Pin = SENSORTILE_LPS22HB_SPI_CS_Pin;
    HAL_GPIO_Init(SENSORTILE_LPS22HB_SPI_CS_Port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(SENSORTILE_LPS22HB_SPI_CS_Port, SENSORTILE_LPS22HB_SPI_CS_Pin, GPIO_PIN_SET);
    break;
  default:
    return COMPONENT_NOT_IMPLEMENTED;
  }
  return COMPONENT_OK;
}
/**
 * @brief  Configures sensor SPI interface.
 * @param  None
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef Sensor_IO_SPI_Init( void )
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  if(HAL_SPI_GetState( &SPI_Sensor_Handle) == HAL_SPI_STATE_RESET )
  {
        /* Peripheral clock enable */
    __HAL_RCC_SPI2_CLK_ENABLE();
  
    /**SPI2 GPIO Configuration    
    PC2     ------> SPI2_MISO
    PC3     ------> SPI2_MOSI
    PD3     ------> SPI2_SCK 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
 
      SPI_Sensor_Handle.Instance = SPI2;
  SPI_Sensor_Handle.Init.Mode = SPI_MODE_MASTER;
  SPI_Sensor_Handle.Init.Direction = SPI_DIRECTION_2LINES;
  SPI_Sensor_Handle.Init.DataSize = SPI_DATASIZE_4BIT;
  SPI_Sensor_Handle.Init.CLKPolarity = SPI_POLARITY_LOW;
  SPI_Sensor_Handle.Init.CLKPhase = SPI_PHASE_1EDGE;
  SPI_Sensor_Handle.Init.NSS = SPI_NSS_SOFT;
  SPI_Sensor_Handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  SPI_Sensor_Handle.Init.FirstBit = SPI_FIRSTBIT_MSB;
  SPI_Sensor_Handle.Init.TIMode = SPI_TIMODE_DISABLE;
  SPI_Sensor_Handle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  SPI_Sensor_Handle.Init.CRCPolynomial = 7;
  SPI_Sensor_Handle.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  SPI_Sensor_Handle.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&SPI_Sensor_Handle) != HAL_OK)
  {
    Error_Handler();
  }
    
    SPI_1LINE_TX(&SPI_Sensor_Handle);
    __HAL_SPI_ENABLE(&SPI_Sensor_Handle);
  }  
  return COMPONENT_OK;
}

/**
 * @brief  This function send a command through SPI bus.
 * @param  command : command id.
 * @param  val : value.
 * @retval None
 */
void SPI_Read(SPI_HandleTypeDef* xSpiHandle, uint8_t *val)
{
  /* In master RX mode the clock is automaticaly generated on the SPI enable.
  So to guarantee the clock generation for only one data, the clock must be
  disabled after the first bit and before the latest bit */
  /* Interrupts should be disabled during this operation */
  
  __disable_irq();
  
  __HAL_SPI_ENABLE(xSpiHandle);
  __asm("dsb\n");
  __asm("dsb\n");
  __HAL_SPI_DISABLE(xSpiHandle);
  
  __enable_irq();
  
  while ((xSpiHandle->Instance->SR & SPI_FLAG_RXNE) != SPI_FLAG_RXNE);
  /* read the received data */
  *val = *(__IO uint8_t *) &xSpiHandle->Instance->DR;
  while ((xSpiHandle->Instance->SR & SPI_FLAG_BSY) == SPI_FLAG_BSY);
}



/**
 * @brief  This function send a command through SPI bus.
 * @param  command : command id.
 * @param  val : value.
 * @retval None
 */
void SPI_Write(SPI_HandleTypeDef* xSpiHandle, uint8_t val)
{
  /* check TXE flag */
  while ((xSpiHandle->Instance->SR & SPI_FLAG_TXE) != SPI_FLAG_TXE);
  
  /* Write the data */
  *((__IO uint8_t*) &xSpiHandle->Instance->DR) = val;
  
  /* Wait BSY flag */
  while ((xSpiHandle->Instance->SR & SPI_FLAG_FTLVL) != SPI_FTLVL_EMPTY);
  while ((xSpiHandle->Instance->SR & SPI_FLAG_BSY) == SPI_FLAG_BSY);
}

uint8_t Sensor_IO_SPI_CS_Disable(void *handle)
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  
  switch(ctx->spiDevice)
  {
  case LSM6DSM:
    HAL_GPIO_WritePin(SENSORTILE_LSM6DSM_SPI_CS_Port, SENSORTILE_LSM6DSM_SPI_CS_Pin, GPIO_PIN_SET);
    break;
  case LSM303AGR_X:
    HAL_GPIO_WritePin(SENSORTILE_LSM303AGR_X_SPI_CS_Port, SENSORTILE_LSM303AGR_X_SPI_CS_Pin, GPIO_PIN_SET);
    break;
  case LSM303AGR_M:
    HAL_GPIO_WritePin(SENSORTILE_LSM303AGR_M_SPI_CS_Port, SENSORTILE_LSM303AGR_M_SPI_CS_Pin, GPIO_PIN_SET);
    break;
  case LPS22HB:
    HAL_GPIO_WritePin(SENSORTILE_LPS22HB_SPI_CS_Port, SENSORTILE_LPS22HB_SPI_CS_Pin, GPIO_PIN_SET);
    break;
  }
  return COMPONENT_OK;
}

uint8_t Sensor_IO_SPI_CS_Enable(void *handle)
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  
  switch(ctx->spiDevice)
  {
  case LSM6DSM:
    HAL_GPIO_WritePin(SENSORTILE_LSM6DSM_SPI_CS_Port, SENSORTILE_LSM6DSM_SPI_CS_Pin, GPIO_PIN_RESET);
    break;
  case LSM303AGR_X:
    HAL_GPIO_WritePin(SENSORTILE_LSM303AGR_X_SPI_CS_Port, SENSORTILE_LSM303AGR_X_SPI_CS_Pin, GPIO_PIN_RESET);
    break;
  case LSM303AGR_M:
    HAL_GPIO_WritePin(SENSORTILE_LSM303AGR_M_SPI_CS_Port, SENSORTILE_LSM303AGR_M_SPI_CS_Pin, GPIO_PIN_RESET);
    break;
  case LPS22HB:
    HAL_GPIO_WritePin(SENSORTILE_LPS22HB_SPI_CS_Port, SENSORTILE_LPS22HB_SPI_CS_Pin, GPIO_PIN_RESET);
    break;
  }
  return COMPONENT_OK;
}


/**
 * @brief  Reads a from the sensor to buffer
 * @param  handle instance handle
 * @param  ReadAddr specifies the internal sensor address register to be read from
 * @param  pBuffer pointer to data buffer
 * @param  nBytesToRead number of bytes to be read
 * @retval 0 in case of success
 * @retval 1 in case of failure
 */
uint8_t Sensor_IO_SPI_Read( void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead )
{
  uint8_t i;
  
// Select the correct device
  Sensor_IO_SPI_CS_Enable(handle);
  
  SPI_Write(&SPI_Sensor_Handle, ReadAddr | 0x80);
//  SYNCHRO_WAIT(SYNCHRO_SPI_DELAY);
  __HAL_SPI_DISABLE(&SPI_Sensor_Handle);
  SPI_1LINE_RX(&SPI_Sensor_Handle);
//  SYNCHRO_WAIT(SYNCHRO_SPI_DELAY);

  for(i=0;i<nBytesToRead;i++)
  {
    SPI_Read(&SPI_Sensor_Handle, pBuffer++);
  }
  
  // Deselect the device
  Sensor_IO_SPI_CS_Disable(handle);  
  
//  SYNCHRO_WAIT(SYNCHRO_SPI_DELAY);
  SPI_1LINE_TX(&SPI_Sensor_Handle);
  __HAL_SPI_ENABLE(&SPI_Sensor_Handle);
//  SYNCHRO_WAIT(SYNCHRO_SPI_DELAY);
  
  return COMPONENT_OK;
}


/**
 * @brief  Writes a buffer to the sensor
 * @param  handle instance handle
 * @param  WriteAddr specifies the internal sensor address register to be written to
 * @param  pBuffer pointer to data buffer
 * @param  nBytesToWrite number of bytes to be written
 * @retval 0 in case of success
 * @retval 1 in case of failure
 */
uint8_t Sensor_IO_SPI_Write( void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite )
{
  uint8_t i;
  
// Select the correct device
  Sensor_IO_SPI_CS_Enable(handle);
  
  SPI_Write(&SPI_Sensor_Handle, WriteAddr);

  for(i=0;i<nBytesToWrite;i++)
  {
    SPI_Write(&SPI_Sensor_Handle, pBuffer[i]);
  }
// Deselect the device
  Sensor_IO_SPI_CS_Disable(handle);
  
  return COMPONENT_OK;
}


/**
 * @brief  Writes a buffer to the sensor
 * @param  handle instance handle
 * @param  WriteAddr specifies the internal sensor address register to be written to
 * @param  pBuffer pointer to data buffer
 * @param  nBytesToWrite number of bytes to be written
 * @retval 0 in case of success
 * @retval 1 in case of failure
 */
uint8_t Sensor_IO_Write( void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite )
{
  
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
 
  
  if(ctx->ifType == 1)
  {
    if ( nBytesToWrite > 1 ) {
      switch(ctx->who_am_i)
        {
          case LSM303AGR_ACC_WHO_AM_I: WriteAddr |= 0x40; break; /* Enable I2C multi-bytes Write */
          case LSM303AGR_MAG_WHO_AM_I: break;
          default:;
        }
    }
   return Sensor_IO_SPI_Write( handle, WriteAddr, pBuffer, nBytesToWrite );
  }
  
  return COMPONENT_ERROR;
}


/**
 * @brief  Reads from the sensor to a buffer
 * @param  handle instance handle
 * @param  ReadAddr specifies the internal sensor address register to be read from
 * @param  pBuffer pointer to data buffer
 * @param  nBytesToRead number of bytes to be read
 * @retval 0 in case of success
 * @retval 1 in case of failure
 */
uint8_t Sensor_IO_Read( void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if(ctx->ifType == 1 )
  {
    if ( nBytesToRead > 1 ) {
      switch(ctx->who_am_i)
        {
          case LSM303AGR_ACC_WHO_AM_I: ReadAddr |= 0x40; break; /* Enable I2C multi-bytes Write */
          case LSM303AGR_MAG_WHO_AM_I: break;
          default:;
        }
    }
   return Sensor_IO_SPI_Read( handle, ReadAddr, pBuffer, nBytesToRead );
  }
  
  return COMPONENT_ERROR;
}
/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

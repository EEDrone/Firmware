/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/* USER CODE END 0 */

#define UART_RxBufferSize 256
__IO uint8_t ubSend = 0;
const uint8_t aStringToSend[] = "STM32F7xx USART LL API Example : TX in IT mode\r\nConfiguration UART 115200 bps, 8 data bit/1 stop bit/No parity/No HW flow control\r\n";
uint8_t ubSizeToSend = sizeof(aStringToSend);
volatile uint8_t UART_RxBuffer[UART_RxBufferSize];
volatile uint8_t UART_RxBuffer2[UART_RxBufferSize];
TUart_Engine Uart_Engine;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_rx;
TMsg Msg;
//DMA_HandleTypeDef hdma_tx;
/* USART3 init function */

void MX_USART3_UART_Init(void)
{
__DMA1_CLK_ENABLE();
  
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
      /* Any data received will be stored "aRxBuffer" buffer : the number max of 
     data received is RXBUFFERSIZE */
  /* Enable RXNE and Error interrupts */  
  //LL_USART_EnableIT_RXNE(USART3);
 // LL_USART_EnableIT_ERROR(USART3);
  
//  /* Configure the DMA handler for Transmission process */
//  hdma_tx.Instance                 = DMA1_Stream3;
//  hdma_tx.Init.Channel             = DMA_CHANNEL_4;
//  hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
//  hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
//  hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
//  hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
//  hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
//  hdma_tx.Init.Mode                = DMA_NORMAL;
//  hdma_tx.Init.Priority            = DMA_PRIORITY_LOW;
//  hdma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
//  hdma_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
//  hdma_tx.Init.MemBurst            = DMA_MBURST_INC4;
//  hdma_tx.Init.PeriphBurst         = DMA_PBURST_INC4;
//
//  HAL_DMA_Init(&hdma_tx);
//
//  /* Associate the initialized DMA handle to the UART handle */
//  __HAL_LINKDMA(&huart3, hdmatx, hdma_tx);

  /* Configure the DMA handler for reception process */
  hdma_rx.Instance                 = DMA1_Stream1;
  hdma_rx.Init.Channel             = DMA_CHANNEL_4;
  hdma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  hdma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_rx.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  hdma_rx.Init.Mode                = DMA_CIRCULAR;//DMA_CIRCULAR;
  hdma_rx.Init.Priority            = DMA_PRIORITY_HIGH;
  hdma_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
  hdma_rx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  hdma_rx.Init.MemBurst            = DMA_MBURST_INC4;
  hdma_rx.Init.PeriphBurst         = DMA_PBURST_INC4;

  HAL_DMA_Init(&hdma_rx);

  /* Associate the initialized DMA handle to the the UART handle */
  __HAL_LINKDMA(&huart3, hdmarx, hdma_rx);

  huart3.pRxBuffPtr = (uint8_t*)UART_RxBuffer;
  huart3.RxXferSize = UART_RxBufferSize;
  huart3.ErrorCode = HAL_UART_ERROR_NONE;
  
  /* Enable the DMA transfer for the receiver request by setting the DMAR bit
  in the UART CR3 register */
  HAL_UART_Receive_DMA(&huart3, (uint8_t*)UART_RxBuffer, UART_RxBufferSize);
}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();
  
    /**USART3 GPIO Configuration    
    PD8     ------> USART3_TX
    PD9     ------> USART3_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* Peripheral interrupt init */
  //  HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
 //   HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();
  
    /**USART3 GPIO Configuration    
    PD8     ------> USART3_TX
    PD9     ------> USART3_RX 
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_8|GPIO_PIN_9);

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(USART3_IRQn);

  }
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
} 



int UART_ReceivedMSG(TMsg *Msg)
{
  uint16_t j, k, l;
  uint16_t DmaCounter, length;
  
  
  if(__HAL_DMA_GET_FLAG(&hdma_rx, __HAL_DMA_GET_FE_FLAG_INDEX(&hdma_rx)) == RESET)
  {
  
    DmaCounter = UART_RxBufferSize - __HAL_DMA_GET_COUNTER(&hdma_rx);
    
    if (DmaCounter >= Uart_Engine.StartOfMsg)
    {
      length = DmaCounter - Uart_Engine.StartOfMsg;
    }
    else
    {
      length = UART_RxBufferSize + DmaCounter - Uart_Engine.StartOfMsg;
    }
      if(length<1)
          return 0;
    j = Uart_Engine.StartOfMsg;
    l=j;
         Msg->Len = length;
         
         for(k=0;k<length;k++){
           Msg->Data[k]=UART_RxBuffer[l];
           l++;
            if (l >= UART_RxBufferSize) {
                l = 0;
            }
         }
    Uart_Engine.StartOfMsg =(j+length)% UART_RxBufferSize;
  }
  return 1;
}



PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART3 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);
  //LL_USART_TransmitData8(USART3, ch);
  return ch;
}
/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

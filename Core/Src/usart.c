/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

UART_HandleTypeDef huart1;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */
  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */
  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  /* USER CODE END USART1_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */
  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */
  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */
  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */
  /* USER CODE END USART1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

// ���ڷ��Ϳ�ʵ��

/**
  * @brief  ���͵����ַ�
  * @param  ch: Ҫ���͵��ַ�
  * @retval None
  */
void UART_SendChar(uint8_t ch)
{		
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, &ch, 1, HAL_MAX_DELAY);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
}

/**
  * @brief  �����ַ���
  * @param  str: Ҫ���͵��ַ���ָ��
  * @retval None
  */
void UART_SendString(const char *str)
{
    if(str == NULL) return;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
}

/**
  * @brief  ��������
  * @param  num: Ҫ���͵�����
  * @param  base: ���� (10, 16, 8, 2)
  * @retval None
  */
void UART_SendInt(int num, uint8_t base)
{
    char buffer[35]; // �㹻���32λ����
    
    if(base == 10) {
        sprintf(buffer, "%d", num);
    } else if(base == 16) {
        sprintf(buffer, "0x%X", num);
    } else if(base == 8) {
        sprintf(buffer, "0%o", num);
    } else if(base == 2) {
        // ���������⴦��
        buffer[0] = '0';
        buffer[1] = 'b';
        for(int8_t i = 0; i < 32; i++) {
            buffer[2 + i] = (num & (1 << (31 - i))) ? '1' : '0';
        }
        buffer[34] = '\0';
    } else {
        return; // ��֧�ֵĽ���
    }
    UART_SendString(buffer);
}

/**
  * @brief  ���͸�����
  * @param  num: Ҫ���͵ĸ�����
  * @param  decimals: С��λ��
  * @retval None
  */
void UART_SendFloat(float num, uint8_t decimals)
{
    char format[10];
    char buffer[20];
    
    if(decimals > 6) decimals = 6;
    
    sprintf(format, "%%.%df", decimals);
    sprintf(buffer, format, num);
    UART_SendString(buffer);
}

/**
  * @brief  ��ʽ����� (����printf)
  * @param  format: ��ʽ���ַ���
  * @param  ...: �ɱ����
  * @retval None
  */
void UART_Printf(unsigned BufferIndex, const char *format, ...)
{
    char buffer[128];
    va_list args;
    
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    UART_SendString(buffer);
}

/**
  * @brief  ���ٷ��͵����ַ�(������)
  * @param  ch: Ҫ���͵��ַ�
  * @retval None
  */
void UART_PutChar(uint8_t ch)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
    huart1.Instance->DR = ch;
    while(!__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE));
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
}

/**
  * @brief  ����Ԥ������Ϣ(��ָʾ��)
  * @param  None
  * @retval None
  */
void UART_SendPredefinedMessage(void)
{
    uint8_t UART_BUF[12] = "you press A!";
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, UART_BUF, sizeof(UART_BUF), HAL_MAX_DELAY);  
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
}

/* USER CODE END 1 */

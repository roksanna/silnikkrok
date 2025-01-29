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
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
/* USER CODE BEGIN 0 */
uint16_t buff_length=300;
extern uint8_t *buf_rx;
extern uint8_t *buf_tx;
extern volatile uint16_t tx_empty, tx_busy, rx_empty;


void Send(char* msgToSend, ...) {
    char dataToSend[100];  // Tworzenie tablicy pomocniczej
    int idx;  // Wskaźnik pomocniczy
    va_list arglist;

    va_start(arglist, msgToSend);  // Tworzenie listy argumentów
    vsnprintf(dataToSend, sizeof(dataToSend), msgToSend, arglist);
    va_end(arglist);

    idx = tx_empty;  // Przypisanie pozycji wskaźnika do wsk. pomoc.

    // Przenoszenie danych do bufora
    for (int i = 0; i < strlen(dataToSend); i++) {
        buf_tx[idx] = dataToSend[i];
        idx++;
        if (idx >= buff_length) {
            idx = 0;  // Resetowanie na początek buforu
        }
    }

    __disable_irq();  // Wyłączanie przerwań

    // Sprawdzanie, czy należy aktywować transmisję
    if ((tx_busy == tx_empty) && (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE) == SET)) {
        tx_empty = idx;  // Ustawienie indeksu pustego
        HAL_UART_Transmit_IT(&huart2, &buf_tx[tx_busy], 1);
        tx_busy++;

        if (tx_busy >= buff_length) {
            tx_busy = 0;  // Resetowanie indeksu na początek buforu
        }
    }

    __enable_irq();  // Włączenie przerwań
}
/* USER CODE END 0 */

UART_HandleTypeDef huart2;

/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}
void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOA, USART_TX_Pin|USART_RX_Pin);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart -> Instance == USART2){
		rx_empty++;
		if(rx_empty >= buff_length){
			rx_empty =0; //reset, jesli osiagnieto koniec buforu
		}
		HAL_UART_Receive_IT(&huart2, &buf_rx[rx_empty], 1);
	}
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart -> Instance == USART2){
		if(tx_busy != tx_empty){
			uint8_t tmp = buf_tx[tx_busy];
			tx_busy++;
			if(tx_busy >= buff_length){
						tx_busy =0; //reset, jesli osiagnieto koniec buforu
					}
			if (tx_busy == buff_length/2){
				HAL_UART_Transmit_IT(&huart2, &tmp, 1);
			}
			else{
			HAL_UART_Transmit_IT(&huart2, &tmp, 1);
			}
		}
	}
}
/* USER CODE END 1 */

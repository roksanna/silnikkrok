/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define buff_length 300
#define PWM_STEPS 100
#define PWM_RESOLUTION 256
#define ACC 0x01
#define DEC 0x02
#define SPD 0x03
#define STP 0x04
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t pwm_data[PWM_STEPS]; //tablica wypełnienia PWM
uint32_t pwm_buffer[PWM_RESOLUTION];

volatile uint16_t message_length; //il. znakow skladajacych sie na otrzymana wiadomosc(pom.znak konca transm.)
volatile uint8_t message[buff_length]; //tablica, do ktorej pobierana jest z bufora odb. otrzymana wiadomosc
volatile uint8_t message_idx; //wskaznik pomocniczy dla odebranej wiadomosci

/*====Receive====*/
uint8_t* buf_rx; // Pointer to dynamically allocated receive buffer
volatile uint16_t rx_empty = 0; //wskaznik wskazujacy pierwsze wolne miejsce w buforze
volatile uint16_t rx_busy = 0; //wskaznik wskazujacy na pierwsze miejsce w buforze (nieprzeanalizowany jeszcze znak)

/*====Transmit====*/
uint8_t* buf_tx; // Pointer to dynamically allocated transmit buffer
volatile uint16_t tx_empty = 0; //wskaznik wskazujacy pierwsze wolne miejsce w buforze
volatile uint16_t tx_busy = 0; //wskaznik wskazujacy na pierwsze miejsce w buforze (niewyslany jeszcze znak)

/*====Frame====*/
uint8_t buffer[buff_length]; //tablica pomocnicza uzywana przy odczytywaniu ramki

uint8_t frameState = 0; // 0-czekamy na początek ramki | 1 - ramka jest przetwarzana

const char device_address[3] ="STM";
char source_address[3]= "SIL";
uint8_t frameLen =0; //ilosc odebranych znakow w ramce
int cmdLength; //dlugosc komendy
int comm_data_len; //ilosc danych dla komendy
int cmd_len_check =0; //sprawdzenie, czy w komendzie nie ma zbednych znakow

/*====Step===*/
uint8_t max_speed = 0;        // Maksymalna prędkość
uint8_t current_speed = 0;    // Aktualna prędkość
uint32_t steps_count = 0;     // Liczba impulsów
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Start_PWM_DMA(void);
void Stop_PWM_DMA(void);
void Update_PWM_Duty(uint8_t duty);
void AnalyzeCmd(uint8_t *buffer, uint16_t frameLen, const char* device_address, const char* destination_address);
void ErrorWrongFrame(void);
void ErrorWrongCmdLen(void);
void ErrorEmptyFrame(void);
void handle_accel(const uint8_t* data, uint8_t len);
void handle_decel(const uint8_t* data, uint8_t len);
void handle_max_speed(const uint8_t* data, uint8_t len);
void handle_step_count(const uint8_t* data, uint8_t len);
void simulate_accel(uint16_t time);
void simulate_decel(uint16_t time);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void ProcessFrame(void) {
    uint8_t singleFrameChar;
    if (rx_busy != rx_empty) {
        if (frameState == 0) {
            singleFrameChar = buf_rx[rx_busy];
            if (singleFrameChar == 0x3A /* : */) {
                frameState = 1;
                frameLen = 0;
            }
            rx_busy++;
            if (rx_busy >= buff_length) {
                rx_busy = 0;
            }
        } else if (frameState == 1) {
            singleFrameChar = buf_rx[rx_busy];
            buffer[frameLen] = singleFrameChar;
            frameLen++;

            if (frameLen > (256 + 9 + 1)) {
                frameLen = 0;
                frameState = 0;
                return;
            }
            rx_busy++;
            if (rx_busy >= buff_length) {
                rx_busy = 0;
            }

            if (singleFrameChar == 0x3A /* : */) {
                if (frameLen > 1) {
                    frameLen = 0;
                }
            }

            if (singleFrameChar == 0x3B /* ; */) {
                if (frameLen - 1 > 9) {
                    char destination_address[4];
                    char data_length[4];

                    memcpy(source_address, &buffer[0], 3);
                    memcpy(destination_address, &buffer[3], 3);
                    memcpy(data_length, &buffer[6], 3);

                    source_address[3] = '\0';
                    destination_address[3] = '\0';
                    data_length[3] = '\0';

                    cmdLength = atoi(data_length);

                    if (strncmp(device_address, destination_address, 3) == 0) {
                        if (cmdLength <= 256) {
                            comm_data_len = frameLen - (9 + cmdLength) - 1;

                            if (comm_data_len < 0) {
                                comm_data_len = 0;
                            }

                            if ((9 + cmdLength) == (frameLen - comm_data_len - 1)) {
                                char cmd[cmdLength + 1];
                                uint8_t command_data[comm_data_len];

                                memcpy(command_data, &buffer[9 + cmdLength], comm_data_len);
                                memcpy(cmd, &buffer[9], cmdLength);
                                cmd[cmdLength] = '\0';

                                AnalyzeCmd((uint8_t*)cmd, (uint16_t)strlen(cmd), device_address, destination_address);

                                // Po zakończeniu analizy ramki wyświetlamy komunikat
                                Send("Ramka została przetworzona\n");

                            } else {
                                ErrorWrongFrame(); // Jeśli ramka nie jest poprawna
                            }
                        }
                    } else if (cmdLength == 0) {
                        int skip_frame = 9 + cmdLength;
                        comm_data_len = frameLen - skip_frame - 1;

                        if (comm_data_len > 0) {
                            ErrorWrongCmdLen(); // Jeśli są dane, a komenda = 0
                        } else if (comm_data_len == 0) {
                            ErrorEmptyFrame(); // Pusta ramka
                        } else {
                            ErrorWrongCmdLen();
                        }
                    }
                }
            }
        }
    }
}

void AnalyzeCmd(uint8_t *buffer, uint16_t frameLen, const char* device_address, const char* destination_address) {
    if (strncmp(device_address, destination_address, 3) == 0) {  // Porównaj adres urządzenia
        switch (buffer[4]) { // 4. bajt to komenda
            case ACC: // Rozpędzanie
                handle_accel(buffer + 5, frameLen - 5);  // Przekaż dane do funkcji
                Send("Komenda: Rozpędzanie\n");  // Wyślij komunikat do terminala
                break;
            case DEC: // Hamowanie
                handle_decel(buffer + 5, frameLen - 5);
                Send("Komenda: Hamowanie\n");  // Wyślij komunikat do terminala
                break;
            case SPD: // Maksymalna prędkość
                handle_max_speed(buffer + 5, frameLen - 5);
                Send("Komenda: Maksymalna prędkość\n");  // Wyślij komunikat do terminala
                break;
            case STP: // Ilość impulsów
                handle_step_count(buffer + 5, frameLen - 5);
                Send("Komenda: Liczba impulsów\n");  // Wyślij komunikat do terminala
                break;
            default:
                Send("Nieznana komenda: 0x%02X\n", buffer[4]);  // Wyślij komunikat o nieznanej komendzie
                break;
        }
    }
}


 void handle_accel(const uint8_t* data, uint8_t len) {
     if (len >= 2) {
         uint16_t accel_time = (data[0] << 8) | data[1];  // Przyjmujemy, że dane to 2 bajty (np. czas przyspieszania)
         printf("Rozpędzanie w czasie: %d ms\n", accel_time);

         // Symulacja przyspieszania
         simulate_accel(accel_time);  // Funkcja symulująca przyspieszanie
     }
 }

 /*=======Comand======*/
 void handle_decel(const uint8_t* data, uint8_t len) {
     if (len >= 2) {
         uint16_t decel_time = (data[0] << 8) | data[1];  // Hamowanie w czasie
         printf("Hamowanie w czasie: %d ms\n", decel_time);

         // Symulacja hamowania
         simulate_decel(decel_time);  // Funkcja symulująca hamowanie
     }
 }

 void handle_max_speed(const uint8_t* data, uint8_t len) {
     if (len >= 1) {
         uint8_t speed = data[0];  // Maksymalna prędkość w jednym bajcie
         printf("Maksymalna prędkość: %d\n", speed);

         max_speed = speed;  // Ustawienie maksymalnej prędkości
     }
 }

 void handle_step_count(const uint8_t* data, uint8_t len) {
     if (len >= 4) {
         uint32_t step_count = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
         printf("Liczba impulsów: %ld\n", step_count);

         // Ustawienie liczby impulsów
         steps_count = step_count;
     }
 }

/*============Simulate==========*/
 void simulate_accel(uint16_t time) {
     uint16_t start_time = HAL_GetTick();  // Czas początkowy
     uint8_t target_speed = max_speed;     // Docelowa prędkość
     uint8_t speed_increase;               // Zmienna do obliczania wzrostu prędkości

     while (HAL_GetTick() - start_time < time) {
         uint16_t elapsed_time = HAL_GetTick() - start_time;  // Miniony czas
         speed_increase = (uint8_t)((elapsed_time * target_speed) / time);  // Jak przyspiesza prędkość

         current_speed = (speed_increase > target_speed) ? target_speed : speed_increase;  // Prędkość nie może przekroczyć max_speed

         // Wyświetlanie aktualnej prędkości
         printf("Prędkość: %d\n", current_speed);

         // Sprawdzanie czasu co 10 ms
         if (HAL_GetTick() - start_time >= 10) {
             start_time = HAL_GetTick();  // Resetujemy czas co 10 ms
         }
     }

     current_speed = target_speed;  // Osiągnięcie maksymalnej prędkości
     printf("Osiągnięto maksymalną prędkość: %d\n", current_speed);
 }
 void simulate_decel(uint16_t time) {
     uint16_t start_time = HAL_GetTick();  // Czas początkowy
     uint8_t initial_speed = current_speed;  // Początkowa prędkość

     while (HAL_GetTick() - start_time < time) {
         uint16_t elapsed_time = HAL_GetTick() - start_time;  // Miniony czas
         uint8_t speed_decrease = (uint8_t)((elapsed_time * initial_speed) / time);  // Jak zmniejsza się prędkość

         current_speed = (initial_speed > speed_decrease) ? (initial_speed - speed_decrease) : 0;  // Prędkość nie może spaść poniżej 0

         // Wyświetlanie aktualnej prędkości
         printf("Prędkość: %d\n", current_speed);

         // Sprawdzanie czasu co 10 ms
         if (HAL_GetTick() - start_time >= 10) {
             start_time = HAL_GetTick();  // Resetujemy czas co 10 ms
         }
     }

     current_speed = 0;  // Osiągnięcie prędkości zerowej (po hamowaniu)
     printf("Zatrzymano silnik.\n");
 }


 /*======PWM=====*/
 // Inicjalizacja i start PWM z DMA
  void Start_PWM_DMA(void) {
      for (uint16_t i = 0; i < PWM_RESOLUTION; i++) {
          pwm_buffer[i] = (i < (PWM_RESOLUTION / 2)) ? PWM_RESOLUTION / 4 : PWM_RESOLUTION / 2;
      }
      if (HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, pwm_buffer, PWM_RESOLUTION) != HAL_OK) {
          Error_Handler();
      }
  }

  // Zatrzymanie PWM z DMA
  void Stop_PWM_DMA(void) {
      HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_1);
  }

  // Aktualizacja wypełnienia PWM
  void Update_PWM_Duty(uint8_t duty) {
      uint32_t value = (duty * PWM_RESOLUTION) / 100;
      for (uint16_t i = 0; i < PWM_RESOLUTION; i++) {
          pwm_buffer[i] = value;
      }
  }


  /*========Error==========*/
  void ErrorWrongFrame(void) {
      Send("Blad: Zla ramka!\r\n");
  }
  void ErrorWrongCmdLen(void) {
      Send("Blad: Za dluga komenda!\r\n");
  }
  void ErrorEmptyFrame(void) {
      Send("Blad: Pusta ramka!\r\n");
  }
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  ProcessFrame();
	      if (frameLen > 0) {
	          AnalyzeCmd(buffer, frameLen, device_address, source_address);
	      }
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void handle_pwm_command(const uint8_t* data, uint8_t len) {
    if (len >= 1) {
        uint8_t pwm_duty = data[0];
        if (pwm_duty == 0) {
            Stop_PWM_DMA();
        } else {
            Update_PWM_Duty(pwm_duty);
            Start_PWM_DMA();
        }
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {

  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

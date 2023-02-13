/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void SWITCH_CTRL(uint8_t const* data);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint8_t rx_len=0;                     //接收数据长度
volatile uint8_t recv_end_flag=0;        //接收完成标记�???
uint8_t RxBuffer[100];      //接收缓存
char  BUFFER_SIZE=100;      //
uint8_t SPIBuffer[2] = {0,0};

#define NUM_OF_NSS_PINS 4

GPIO_TypeDef* NSS_PORT[NUM_OF_NSS_PINS] = {GPIOB, GPIOB, GPIOA,GPIOA};
const uint16_t NSS_PIN[NUM_OF_NSS_PINS] = {GPIO_PIN_10, GPIO_PIN_1, GPIO_PIN_7,GPIO_PIN_5};

void SPI_NSS_Pin_Control(uint8_t PinNumber, uint8_t PinState){
    HAL_GPIO_WritePin(NSS_PORT[PinNumber], NSS_PIN[PinNumber], (GPIO_PinState)PinState);
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
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_SPI3_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
    HAL_TIM_Base_Start_IT(&htim2);
    HAL_TIM_Base_Start_IT(&htim3);
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart2, RxBuffer, BUFFER_SIZE);   //
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1){
      if(recv_end_flag){
          HAL_UART_Transmit_DMA(&huart2,RxBuffer,rx_len);

          SWITCH_CTRL(RxBuffer);
          SPI_NSS_Pin_Control((RxBuffer[2]-1)/2,0);
          HAL_SPI_Transmit(&hspi2,RxBuffer+1,1,20);
          while( hspi2.State == HAL_SPI_STATE_BUSY_TX){};  // wait
          SPI_NSS_Pin_Control((RxBuffer[2]-1)/2,1);
          SPIBuffer[0] = RxBuffer[4];
          SPIBuffer[1]= RxBuffer[3];
          HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
          HAL_SPI_Transmit(&hspi3,SPIBuffer,1,20);
          while(hspi3.State == HAL_SPI_STATE_BUSY_TX){};  // wait
          HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);

          for(uint8_t i=0;i<rx_len;i++){   //clear
              RxBuffer[i]=0;
          }
          rx_len=0;
          recv_end_flag=0;
      }
      HAL_UART_Receive_DMA(&huart2,RxBuffer,BUFFER_SIZE);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    static uint32_t time_cnt =0;
    static uint32_t time_cnt3 =0;
    if(htim->Instance == TIM2)    // 0.005s
    {
        if(++time_cnt >= 200)
        {
            time_cnt =0;
            HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
        }
    }
    if(htim->Instance == TIM3)
    {
        if(++time_cnt3 >= 1000)
        {
            time_cnt3 =0;
        }
    }
}


void SWITCH_CTRL(uint8_t const*data){
    if(data[0]==1){
        HAL_GPIO_WritePin( GPIOA, GPIO_PIN_1, GPIO_PIN_RESET );   // low
        HAL_GPIO_WritePin( GPIOA, GPIO_PIN_0, GPIO_PIN_SET );     // high
        HAL_GPIO_WritePin( GPIOA, GPIO_PIN_15, GPIO_PIN_RESET );  // low
        HAL_GPIO_WritePin( GPIOA, GPIO_PIN_12, GPIO_PIN_SET );    // high
        HAL_GPIO_WritePin( GPIOB, GPIO_PIN_8, GPIO_PIN_RESET );   // low
        HAL_GPIO_WritePin( GPIOB, GPIO_PIN_9, GPIO_PIN_RESET );   // low
        HAL_GPIO_WritePin( GPIOA, GPIO_PIN_8, GPIO_PIN_RESET );   // low
        HAL_GPIO_WritePin( GPIOA, GPIO_PIN_11, GPIO_PIN_RESET );  // low
    }else if(data[0]==2){
        HAL_GPIO_WritePin( GPIOA, GPIO_PIN_1, GPIO_PIN_RESET );  // low
        HAL_GPIO_WritePin( GPIOA, GPIO_PIN_0, GPIO_PIN_RESET );  // low
        HAL_GPIO_WritePin( GPIOA, GPIO_PIN_15, GPIO_PIN_RESET ); // low
        HAL_GPIO_WritePin( GPIOA, GPIO_PIN_12, GPIO_PIN_RESET ); // low
        HAL_GPIO_WritePin( GPIOB, GPIO_PIN_8, GPIO_PIN_RESET );  // low
        HAL_GPIO_WritePin( GPIOB, GPIO_PIN_9, GPIO_PIN_SET );    // high
        HAL_GPIO_WritePin( GPIOA, GPIO_PIN_8, GPIO_PIN_RESET );  // low
        HAL_GPIO_WritePin( GPIOA, GPIO_PIN_11, GPIO_PIN_SET );   // high
    }else if(data[0]==3){
        HAL_GPIO_WritePin( GPIOA, GPIO_PIN_1, GPIO_PIN_SET );    // high
        HAL_GPIO_WritePin( GPIOA, GPIO_PIN_0, GPIO_PIN_RESET );  // low
        HAL_GPIO_WritePin( GPIOA, GPIO_PIN_15, GPIO_PIN_SET );   // high
        HAL_GPIO_WritePin( GPIOA, GPIO_PIN_12, GPIO_PIN_RESET ); // low
        HAL_GPIO_WritePin( GPIOB, GPIO_PIN_8, GPIO_PIN_SET );    // high
        HAL_GPIO_WritePin( GPIOB, GPIO_PIN_9, GPIO_PIN_RESET );  // low
        HAL_GPIO_WritePin( GPIOA, GPIO_PIN_8, GPIO_PIN_SET );    // low
        HAL_GPIO_WritePin( GPIOA, GPIO_PIN_11, GPIO_PIN_RESET ); // high
    }else{
        HAL_GPIO_WritePin( GPIOA, GPIO_PIN_1, GPIO_PIN_RESET );  // low
        HAL_GPIO_WritePin( GPIOA, GPIO_PIN_0, GPIO_PIN_RESET );  // low
        HAL_GPIO_WritePin( GPIOA, GPIO_PIN_15, GPIO_PIN_RESET ); // low
        HAL_GPIO_WritePin( GPIOA, GPIO_PIN_12, GPIO_PIN_RESET ); // low
        HAL_GPIO_WritePin( GPIOB, GPIO_PIN_8, GPIO_PIN_RESET );  // low
        HAL_GPIO_WritePin( GPIOB, GPIO_PIN_9, GPIO_PIN_RESET );  // low
        HAL_GPIO_WritePin( GPIOA, GPIO_PIN_8, GPIO_PIN_RESET );  // low
        HAL_GPIO_WritePin( GPIOA, GPIO_PIN_11, GPIO_PIN_RESET ); // low
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    if(huart==&huart2){
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
    if(huart == &huart2){
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
//  __disable_irq();
//  while (1)
//  {
//  }
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

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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_hid.h"
#include "main.h"
#include <stdio.h>
#include "stm32f4xx_hal_uart.h"
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  uint8_t MODIFIER;
  uint8_t RESERVED;
  uint8_t Keycode1;
  uint8_t Keycode2;
  uint8_t Keycode3;
  uint8_t Keycode4;
  uint8_t Keycode5;
  uint8_t Keycode6;
} HID_SendKeycode;

HID_SendKeycode keyboardReport = {0};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define KEY_NUMBER 5


#define KC_A 0x04
#define KC_B 0x05
#define KC_C 0x06
#define KC_D 0x07
#define KC_E 0x08
#define KC_F 0x09
#define KC_G 0x10
#define KC_H 0x11
#define KC_I 0x12

// Custom Keycode
#define KC_LCTL 10000
#define KC_LSFT 10001
#define KC_LALT 10002
#define KC_LGUI 10003
#define KC_RCTL 10004
#define KC_RSFT 10005
#define KC_RALT 10006
#define KC_RGUI 10007

#define KC_FN1 20000




#define BIT_LCTL 0b00000001
#define BIT_LSFT 0b00000010
#define BIT_LALT 0b00000100
#define BIT_LGUI 0b00001000
#define BIT_RCTL 0b00010000
#define BIT_RSFT 0b00100000
#define BIT_RALT 0b01000000
#define BIT_RGUI 0b10000000






/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;

/* USER CODE BEGIN PV */

GPIO_PinState pinState;
GPIO_PinState pinState1;



int Scanrate = 0; // GPIO 읽기 카운트

int Timer = 0;  // 경과 시간 (ms)

int LastTimer = 0;



GPIO_TypeDef* GPIO_ABC[KEY_NUMBER] = { GPIOD, GPIOD, GPIOD, GPIOD, GPIOD };

int GPIO_Num[KEY_NUMBER] = { GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_3, GPIO_PIN_4, GPIO_PIN_5 };




uint16_t Keycode[][KEY_NUMBER] = {    { KC_LCTL, KC_LSFT, KC_A, KC_B, KC_FN1 },{ KC_D, KC_E, KC_F, KC_G, KC_FN1 }    };

int Modifier_Bit[8] = {BIT_LCTL,BIT_LSFT,BIT_LALT,BIT_LGUI,BIT_RCTL,BIT_RSFT,BIT_RALT,BIT_RGUI};


uint8_t Modifier_Sum = 0b00000000;




int MatrixState[KEY_NUMBER] = { 0 };
int LastMatrixState[KEY_NUMBER] = { 0 };



int LayerState = 0;

int WhichLayer[KEY_NUMBER] = { 0 };

int TempKeycode = 0;











/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

extern USBD_HandleTypeDef hUsbDeviceHS;




void PinToMatrix( int i ) { //SwitchNumber starts from 0 to KET_NUMBER - 1
   pinState = HAL_GPIO_ReadPin(GPIO_ABC[i], GPIO_Num[i]);
   MatrixState[i] = pinState; // High 1, Low 0
}

void PressKeycodes(int i)
{
    if(Keycode[LayerState][i] >= 10000)
    { //MODIFIER
		Modifier_Sum = Modifier_Sum | Modifier_Bit[Keycode[LayerState][i]-10000];
		keyboardReport.MODIFIER = Modifier_Sum;

        char message1[100];
        sprintf(message1, "ModifierPress \n\r");
        HAL_UART_Transmit(&huart4, (uint8_t*)message1, strlen(message1), HAL_MAX_DELAY);
    }
    else if ( Keycode[LayerState][i] == KC_FN1)
    {
    	LayerState = 1; // Layer1 when pressed
    }
    else
    {
    	TempKeycode = 0;
    	TempKeycode = Keycode[LayerState][i];

        if (keyboardReport.Keycode1 == 0x00)
        {
            keyboardReport.Keycode1 = TempKeycode;
        }
        else if (keyboardReport.Keycode2 == 0x00 && Keycode[LayerState][i] != keyboardReport.Keycode1)
        {
            keyboardReport.Keycode2 = TempKeycode;
        }
        else if (keyboardReport.Keycode3 == 0x00 && Keycode[LayerState][i] != keyboardReport.Keycode2 && Keycode[LayerState][i] != keyboardReport.Keycode1)
        {
            keyboardReport.Keycode3 = TempKeycode;
        }
        else if (keyboardReport.Keycode4 == 0x00 && Keycode[LayerState][i] != keyboardReport.Keycode3 && Keycode[LayerState][i] != keyboardReport.Keycode2 && Keycode[LayerState][i] != keyboardReport.Keycode1)
        {
            keyboardReport.Keycode4 = TempKeycode;
        }
        else if (keyboardReport.Keycode5 == 0x00 && Keycode[LayerState][i] != keyboardReport.Keycode4 && Keycode[LayerState][i] != keyboardReport.Keycode3 && Keycode[LayerState][i] != keyboardReport.Keycode2 && Keycode[LayerState][i] != keyboardReport.Keycode1)
        {
            keyboardReport.Keycode5 = TempKeycode;
        }
        else if (keyboardReport.Keycode6 == 0x00 && Keycode[LayerState][i] != keyboardReport.Keycode5 && Keycode[LayerState][i] != keyboardReport.Keycode4 && Keycode[LayerState][i] != keyboardReport.Keycode3 && Keycode[LayerState][i] != keyboardReport.Keycode2 && Keycode[LayerState][i] != keyboardReport.Keycode1)
        {
            keyboardReport.Keycode6 = TempKeycode;
        }

        WhichLayer[i] = LayerState;

		char message4[100];
		sprintf(message4, "PressKeycodes \n\r");
		HAL_UART_Transmit(&huart4, (uint8_t*)message4, strlen(message4), HAL_MAX_DELAY);
    }
}

void ReleaseKeycodes(int i)
{
    if(Keycode[LayerState][i] >= 10000)
    { //MODIFIER
		Modifier_Sum = Modifier_Sum & ~(Modifier_Bit[Keycode[LayerState][i]-10000]);
		keyboardReport.MODIFIER = Modifier_Sum;

        char message2[100];
        sprintf(message2, "ModifierRelease \n\r");
        HAL_UART_Transmit(&huart4, (uint8_t*)message2, strlen(message2), HAL_MAX_DELAY);
    }
    else if ( Keycode[LayerState][i] == KC_FN1)
    {
    	LayerState = 0; // Layer0 when released
    }
    else
    {
    	TempKeycode = 0;
    	TempKeycode = Keycode[WhichLayer[i]][i];

        if (keyboardReport.Keycode1 == TempKeycode)
        {
            keyboardReport.Keycode1 = 0x00;
        }
        if (keyboardReport.Keycode2 == TempKeycode)
        {
            keyboardReport.Keycode2 = 0x00;
        }
        if (keyboardReport.Keycode3 == TempKeycode)
        {
            keyboardReport.Keycode3 = 0x00;
        }
        if (keyboardReport.Keycode4 == TempKeycode)
        {
            keyboardReport.Keycode4 = 0x00;
        }
        if (keyboardReport.Keycode5 == TempKeycode)
        {
            keyboardReport.Keycode5 = 0x00;
        }
        if (keyboardReport.Keycode6 == TempKeycode)
        {
            keyboardReport.Keycode6 = 0x00;
        }

		char message5[100];
		sprintf(message5, "ReleaseKeycodes \n\r");
		HAL_UART_Transmit(&huart4, (uint8_t*)message5, strlen(message5), HAL_MAX_DELAY);
    }
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
  MX_USB_DEVICE_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

//ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ

//Scanrate Check


   Timer = HAL_GetTick();


   if ( Timer - LastTimer >= 1000)
   {
      char message[100];
      sprintf(message, "Time(ms) = %d |  Scanrate(Hz) = %d \n\r",Timer,Scanrate);
      HAL_UART_Transmit(&huart4, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);


      //Time = XXX, Scanrate = XXX

      LastTimer = Timer;
      Scanrate = 0;
   }

   Scanrate = Scanrate + 1;

//ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ

   memcpy(LastMatrixState, MatrixState, sizeof(MatrixState));
   for (int i=0; i<KEY_NUMBER; ++i)
   {
     PinToMatrix(i);
   }

   if ( memcmp(LastMatrixState, MatrixState, sizeof(MatrixState)) != 0)
   {

       char message3[100];
       sprintf(message3, "%d \n\r", memcmp(LastMatrixState, MatrixState, sizeof(MatrixState)));
       HAL_UART_Transmit(&huart4, (uint8_t*)message3, strlen(message3), HAL_MAX_DELAY);



	   for (int i=0; i<KEY_NUMBER; ++i)
	   {
		   if (LastMatrixState[i] != MatrixState[i])
		   {
			   if ( MatrixState[i] == 0 )
			   {
				  PressKeycodes(i);
			   }
			   else
			   {
				  ReleaseKeycodes(i);
			   }
		   }
	   }




      USBD_HID_SendReport(&hUsbDeviceHS, (uint8_t *)&keyboardReport, sizeof(keyboardReport));


   }








//ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ














  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin : PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD1 PD3 PD4
                           PD5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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

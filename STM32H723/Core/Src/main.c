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
#include "stm32h7xx_hal_uart.h"
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
    uint8_t MODIFIER;
    uint8_t RESERVED;
    uint8_t Keycode1;
    uint8_t Keycode2;
    uint8_t Keycode3;
    uint8_t Keycode4;
    uint8_t Keycode5;
    uint8_t Keycode6;
} HID_SendKeycode;

typedef struct {
    int WhereIsOne;
    int PinState;
} MatrixScanResult;










HID_SendKeycode keyboardReport = {0};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define KEY_NUMBER 100
#define HOLD_TIME 250 // (ms)
#define DEBOUNCE_TIME 100 // (ms)
#define TAP_DELAY 50 // (us)

// Keycode are in main.h


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

/* USER CODE BEGIN PV */
GPIO_PinState pinState;

int Scanrate = 0; // GPIO 읽기 카운트

int Timer = 0; // 경과 시간 (ms)

int LastTimer = 0;

GPIO_TypeDef *GPIO_ABC[KEY_NUMBER] = {GPIOD, GPIOD, GPIOD, GPIOD, GPIOD, GPIOD, GPIOD, GPIOD, GPIOD, GPIOD, GPIOD, GPIOD, GPIOD, GPIOD, GPIOD, GPIOD, GPIOD };

int GPIO_Num[KEY_NUMBER] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3, GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_7, GPIO_PIN_8, GPIO_PIN_9, GPIO_PIN_10, GPIO_PIN_11, GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15 };

uint16_t Keycode[][KEY_NUMBER] = {{GPIOA0_0, GPIOA1_0, GPIOA2_0, GPIOA3_0, GPIOA4_0, GPIOA5_0, GPIOA6_0, GPIOA7_0, GPIOA8_0, GPIOA9_0, GPIOA10_0, GPIOA11_0, GPIOA12_0, GPIOA13_0, GPIOA14_0, GPIOA15_0, GPIOB0_0, GPIOB1_0, GPIOB2_0, GPIOB3_0, GPIOB4_0, GPIOB5_0, GPIOB6_0, GPIOB7_0, GPIOB8_0, GPIOB9_0, GPIOB10_0, GPIOB11_0, GPIOB12_0, GPIOB13_0, GPIOB14_0, GPIOB15_0, GPIOC0_0, GPIOC1_0, GPIOC2_0, GPIOC3_0, GPIOC4_0, GPIOC5_0, GPIOC6_0, GPIOC7_0, GPIOC8_0, GPIOC9_0, GPIOC10_0, GPIOC11_0, GPIOC12_0, GPIOC13_0, GPIOC14_0, GPIOC15_0, GPIOD0_0, GPIOD1_0, GPIOD2_0, GPIOD3_0, GPIOD4_0, GPIOD5_0, GPIOD6_0, GPIOD7_0, GPIOD8_0, GPIOD9_0, GPIOD10_0, GPIOD11_0, GPIOD12_0, GPIOD13_0, GPIOD14_0, GPIOD15_0, GPIOE0_0, GPIOE1_0, GPIOE2_0, GPIOE3_0, GPIOE4_0, GPIOE5_0, GPIOE6_0, GPIOE7_0, GPIOE8_0, GPIOE9_0, GPIOE10_0, GPIOE11_0, GPIOE12_0, GPIOE13_0, GPIOE14_0, GPIOE15_0}, {GPIOA0_1, GPIOA1_1, GPIOA2_1, GPIOA3_1, GPIOA4_1, GPIOA5_1, GPIOA6_1, GPIOA7_1, GPIOA8_1, GPIOA9_1, GPIOA10_1, GPIOA11_1, GPIOA12_1, GPIOA13_1, GPIOA14_1, GPIOA15_1, GPIOB0_1, GPIOB1_1, GPIOB2_1, GPIOB3_1, GPIOB4_1, GPIOB5_1, GPIOB6_1, GPIOB7_1, GPIOB8_1, GPIOB9_1, GPIOB10_1, GPIOB11_1, GPIOB12_1, GPIOB13_1, GPIOB14_1, GPIOB15_1, GPIOC0_1, GPIOC1_1, GPIOC2_1, GPIOC3_1, GPIOC4_1, GPIOC5_1, GPIOC6_1, GPIOC7_1, GPIOC8_1, GPIOC9_1, GPIOC10_1, GPIOC11_1, GPIOC12_1, GPIOC13_1, GPIOC14_1, GPIOC15_1, GPIOD0_1, GPIOD1_1, GPIOD2_1, GPIOD3_1, GPIOD4_1, GPIOD5_1, GPIOD6_1, GPIOD7_1, GPIOD8_1, GPIOD9_1, GPIOD10_1, GPIOD11_1, GPIOD12_1, GPIOD13_1, GPIOD14_1, GPIOD15_1, GPIOE0_1, GPIOE1_1, GPIOE2_1, GPIOE3_1, GPIOE4_1, GPIOE5_1, GPIOE6_1, GPIOE7_1, GPIOE8_1, GPIOE9_1, GPIOE10_1, GPIOE11_1, GPIOE12_1, GPIOE13_1, GPIOE14_1, GPIOE15_1}};

int Modifier_Bit[8] = {BIT_LCTL, BIT_LSFT, BIT_LALT, BIT_LGUI, BIT_RCTL, BIT_RSFT, BIT_RALT, BIT_RGUI};

uint8_t Modifier_Sum = 0b00000000;

int LayerState = 0;

int WhichLayer[KEY_NUMBER] = {0};

int TempKeycode = 0;

char UART_message[100];

int KeyTimer = 0;

int DebounceTimer[KEY_NUMBER] = { 0 };

uint32_t gpioA_state; // GPIOA의 모든 핀 상태
uint32_t gpioB_state; // GPIOB의 모든 핀 상태
uint32_t gpioC_state; // GPIOC의 모든 핀 상태
uint32_t gpioD_state; // GPIOD의 모든 핀 상태
uint32_t gpioE_state; // GPIOE의 모든 핀 상태

uint32_t Last_gpioA_state;
uint32_t Last_gpioB_state;
uint32_t Last_gpioC_state;
uint32_t Last_gpioD_state;
uint32_t Last_gpioE_state;

uint32_t gpioD_state = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern USBD_HandleTypeDef hUsbDeviceHS;



MatrixScanResult MatrixScan()
{
	uint32_t Last_gpioA_state = gpioA_state;
	uint32_t Last_gpioB_state = gpioB_state;
	uint32_t Last_gpioC_state = gpioC_state;
	uint32_t Last_gpioD_state = gpioD_state;
	uint32_t Last_gpioE_state = gpioE_state;

    uint32_t gpioA_state = GPIOA->IDR; // GPIOA의 모든 핀 상태
    uint32_t gpioB_state = GPIOB->IDR; // GPIOB의 모든 핀 상태
    uint32_t gpioC_state = GPIOC->IDR; // GPIOC의 모든 핀 상태
    uint32_t gpioD_state = GPIOD->IDR; // GPIOD의 모든 핀 상태
    uint32_t gpioE_state = GPIOE->IDR; // GPIOE의 모든 핀 상태

    uint32_t changedPinA = gpioA_state ^ Last_gpioA_state;
    uint32_t changedPinB = gpioB_state ^ Last_gpioB_state;
    uint32_t changedPinC = gpioC_state ^ Last_gpioC_state;
    uint32_t changedPinD = gpioD_state ^ Last_gpioD_state;
    uint32_t changedPinE = gpioE_state ^ Last_gpioE_state;



    if ( changedPinA != 0 )
    {
        int WhereIsOne = 0;
        while ((changedPinA & 0) == true)
        {
        	changedPinA = changedPinA >> 1;
            WhereIsOne++;
        }

        MatrixScanResult result;

        uint8_t bitValue = (changedPinA >> WhereIsOne) & 1;

        result.WhereIsOne = WhereIsOne;
        result.PinState = bitValue;

        return result;

    }
    else if ( changedPinB != 0)
    {
        int WhereIsOne = 0;
        while ((changedPinB & 0) == true)
        {
        	changedPinB = changedPinB >> 1;
            WhereIsOne++;
        }

        MatrixScanResult result;

        uint8_t bitValue = (changedPinB >> WhereIsOne) & 1;

        result.WhereIsOne = WhereIsOne+16;
        result.PinState = bitValue;

        return result;

    }
    else if ( changedPinC != 0)
    {
        int WhereIsOne = 0;
        while ((changedPinC & 0) == true)
        {
        	changedPinC = changedPinC >> 1;
            WhereIsOne++;
        }

        MatrixScanResult result;

        uint8_t bitValue = (changedPinC >> WhereIsOne) & 1;

        result.WhereIsOne = WhereIsOne + 32;
        result.PinState = bitValue;

        return result;

    }
    else if ( changedPinD != 0)
    {
        int WhereIsOne = 0;
        while ((changedPinD & 0) == true)
        {
        	changedPinD = changedPinD >> 1;
            WhereIsOne++;
        }

        MatrixScanResult result;

        uint8_t bitValue = (changedPinD >> WhereIsOne) & 1;

        result.WhereIsOne = WhereIsOne + 48;
        result.PinState = bitValue;

        return result;

    }
    else if ( changedPinE != 0)
    {
        int WhereIsOne = 0;
        while ((changedPinE & 0) == true)
        {
        	changedPinE = changedPinE >> 1;
            WhereIsOne++;
        }

        MatrixScanResult result;

        uint8_t bitValue = (changedPinE >> WhereIsOne) & 1;

        result.WhereIsOne = WhereIsOne + 64;
        result.PinState = bitValue;

        return result;

    }

    MatrixScanResult result;

    result.WhereIsOne = -1;
    result.PinState = -1;

    return result;
}








void SetKeycode(int keycode)
{
    if (keyboardReport.Keycode1 == 0x00)
    {
        keyboardReport.Keycode1 = keycode;
    }
    else if (keyboardReport.Keycode2 == 0x00 && keycode != keyboardReport.Keycode1)
    {
        keyboardReport.Keycode2 = keycode;
    }
    else if (keyboardReport.Keycode3 == 0x00 && keycode != keyboardReport.Keycode2 && keycode != keyboardReport.Keycode1)
    {
        keyboardReport.Keycode3 = keycode;
    }
    else if (keyboardReport.Keycode4 == 0x00 && keycode != keyboardReport.Keycode3 && keycode != keyboardReport.Keycode2 && keycode != keyboardReport.Keycode1)
    {
        keyboardReport.Keycode4 = keycode;
    }
    else if (keyboardReport.Keycode5 == 0x00 && keycode != keyboardReport.Keycode4 && keycode != keyboardReport.Keycode3 && keycode != keyboardReport.Keycode2 && keycode != keyboardReport.Keycode1)
    {
        keyboardReport.Keycode5 = keycode;
    }
    else if (keyboardReport.Keycode6 == 0x00 && keycode != keyboardReport.Keycode5 && keycode != keyboardReport.Keycode4 && keycode != keyboardReport.Keycode3 && keycode != keyboardReport.Keycode2 && keycode != keyboardReport.Keycode1)
    {
        keyboardReport.Keycode6 = keycode;
    }
}

void ResetKeycode(int keycode)
{
    if (keyboardReport.Keycode1 == keycode)
    {
        keyboardReport.Keycode1 = 0x00;
    }
    if (keyboardReport.Keycode2 == keycode)
    {
        keyboardReport.Keycode2 = 0x00;
    }
    if (keyboardReport.Keycode3 == keycode)
    {
        keyboardReport.Keycode3 = 0x00;
    }
    if (keyboardReport.Keycode4 == keycode)
    {
        keyboardReport.Keycode4 = 0x00;
    }
    if (keyboardReport.Keycode5 == keycode)
    {
        keyboardReport.Keycode5 = 0x00;
    }
    if (keyboardReport.Keycode6 == keycode)
    {
        keyboardReport.Keycode6 = 0x00;
    }
}

void KeycodeSend()
{
    USBD_HID_SendReport(&hUsbDeviceHS, (uint8_t *)&keyboardReport, sizeof(keyboardReport));

    char message[100];
    sprintf(message, "KeycodeSend");
    HAL_UART_Transmit(&huart4, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
}

void PressKeycodes(int i)
{

	if (Keycode[LayerState][i] >= HOLDTAP_SHIFTING)
	{
		KeyTimer = HAL_GetTick();

		char message5[100];
		sprintf(message5, "HoldTapPress = %d \n\r", Keycode[LayerState][i]);
		HAL_UART_Transmit(&huart4, (uint8_t *)message5, strlen(message5), HAL_MAX_DELAY);
	}
	else if (Keycode[LayerState][i] >= 10000)
	{ // MODIFIER
		Modifier_Sum = Modifier_Sum | Modifier_Bit[Keycode[LayerState][i] - 10000];
		keyboardReport.MODIFIER = Modifier_Sum;

		char message1[100];
		sprintf(message1, "ModifierPress = %d \n\r", Modifier_Bit[Keycode[LayerState][i] - 10000]);
		HAL_UART_Transmit(&huart4, (uint8_t *)message1, strlen(message1), HAL_MAX_DELAY);
	}
	else if (Keycode[LayerState][i] >= 5000)
	{

		if ( Keycode[LayerState][i] == KC_FN1)
		{
			LayerState = 1; // Layer1 when pressed
		}
		else if ( Keycode[LayerState][i] == KC_TD0)
		{
			LayerState = 0;
		}
		else if ( Keycode[LayerState][i] == KC_TD1)
		{
			LayerState = 1;
		}

		char message1[100];
		sprintf(message1, "Current Layer = %d \n\r", LayerState);
		HAL_UART_Transmit(&huart4, (uint8_t *)message1, strlen(message1), HAL_MAX_DELAY);
	}
	else
	{
		SetKeycode(Keycode[LayerState][i]);

		WhichLayer[i] = LayerState;

		char message4[100];
		sprintf(message4, "PressKeycodes = %d \n\r", Keycode[LayerState][i]);
		HAL_UART_Transmit(&huart4, (uint8_t *)message4, strlen(message4), HAL_MAX_DELAY);
	}
}

void ReleaseKeycodes(int i)
{
    if (Keycode[LayerState][i] >= HOLDTAP_SHIFTING)
    {
        if (HAL_GetTick() - KeyTimer > HOLD_TIME)
        {
            TempKeycode = Keycode[LayerState + 1][i]; // Hold 작동시 다음 레이어에 있는 키코드 전송
        }
        else
        {
            TempKeycode = Keycode[LayerState][i] - 30000; // Tap 작동시 현재 레이어에 있는 키코드 -30000 전송
        }
        SetKeycode(TempKeycode);
        KeycodeSend();
        HAL_Delay(TAP_DELAY);
        ResetKeycode(TempKeycode);
        KeycodeSend();
        HAL_Delay(TAP_DELAY);

        char message5[100];
        sprintf(message5, "HoldTapRelease = %d \n\r", Keycode[LayerState][i]);
        HAL_UART_Transmit(&huart4, (uint8_t *)message5, strlen(message5), HAL_MAX_DELAY);
    }
    else if (Keycode[LayerState][i] >= 10000)
    { // MODIFIER
        Modifier_Sum = Modifier_Sum & ~(Modifier_Bit[Keycode[LayerState][i] - 10000]);
        keyboardReport.MODIFIER = Modifier_Sum;

        char message2[100];
        sprintf(message2, "ModifierRelease = %d\n\r", Keycode[LayerState][i]);
        HAL_UART_Transmit(&huart4, (uint8_t *)message2, strlen(message2), HAL_MAX_DELAY);
    }
    else if (Keycode[LayerState][i] >= 5000 )
    {
    	if ( Keycode[LayerState][i] == KC_FN1 )
    	{
            LayerState = 0; // Layer0 when released
            char message1[100];
            sprintf(message1, "Current Layer = %d \n\r", LayerState);
            HAL_UART_Transmit(&huart4, (uint8_t *)message1, strlen(message1), HAL_MAX_DELAY);
    	}
    }
    else
    {
        TempKeycode = Keycode[WhichLayer[i]][i];

        ResetKeycode(TempKeycode);

        char message5[100];
        sprintf(message5, "ReleaseKeycodes = %d \n\r", TempKeycode);
        HAL_UART_Transmit(&huart4, (uint8_t *)message5, strlen(message5), HAL_MAX_DELAY);
    }

}

void CheckScanrate()
{
    if (HAL_GetTick() - LastTimer >= 10000)
    {
        char message[100];
        sprintf(message, "Time(ms) = %d |  Scanrate(Hz) = %d \n\r", HAL_GetTick(), Scanrate / 10);
        HAL_UART_Transmit(&huart4, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);

        LastTimer = HAL_GetTick();
        Scanrate = 0;
    }

    Scanrate = Scanrate + 1;
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
        //Timer = HAL_GetTick(); // Get Current Time

    	//CheckScanrate();

        if (HAL_GetTick() - LastTimer >= 10000)
        {
            char message[100];
            sprintf(message, "Time(ms) = %d |  Scanrate(Hz) = %d \n\r", HAL_GetTick(), Scanrate / 10);
            HAL_UART_Transmit(&huart4, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);

            LastTimer = HAL_GetTick();
            Scanrate = 0;
        }

        Scanrate = Scanrate + 1;

        // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ


        MatrixScanResult Matrix = MatrixScan();
/*
        uint32_t Temp = GPIOD->IDR;
        char message[100];
        sprintf(message, "1 = %d \n\r", Temp);
        HAL_UART_Transmit(&huart4, (uint8_t *)message, strlen(message), HAL_MAX_DELAY); */


        char message[100];
        sprintf(message, "1 = %d |  2 =  %d | 3 = %d \n\r", Matrix.WhereIsOne, Matrix.PinState, gpioD_state);
        HAL_UART_Transmit(&huart4, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);



        /*
        if ( Matrix.WhereIsOne != -1 )
        {

			if ( Matrix.PinState == 0)
			{
				PressKeycodes(Matrix.WhereIsOne);
			}
			else
			{
				ReleaseKeycodes(Matrix.WhereIsOne);
			}
        	KeycodeSend();

        	Timer++;
        }

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 110;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 1;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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

  /*Configure GPIO pins : PD8 PD9 PD10 PD11
                           PD12 PD13 PD14 PD15
                           PD0 PD1 PD2 PD3
                           PD4 PD5 PD6 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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

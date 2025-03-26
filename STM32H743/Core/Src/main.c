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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_hid.h"
#include <stdio.h>
#include "stm32h7xx_hal_uart.h"
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "main.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define KEY_NUMBER 16*9
#define HOLD_TIME 200 // (ms)
#define DEBOUNCE_TIME 5 // (ms)
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

#define EXCLUDE_PIN_A4 (1U << 4)

#define COMBO_MAX_LENGTH 10
#define COMBO_NUMBER 1
#define COMBO_TIME 50


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */

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
    int pinNumber;   // 변경된 핀 번호 (0 ~ 79, 포트 A~E 포함)
    uint8_t pinState; // 핀의 현재 상태 (HIGH: 1, LOW: 0)
} MatrixScanResult;

typedef struct {
    int dance_index;
    bool pressed;
    int count;
    bool activated;
    int last_pressed;
} TapDanceState;

typedef struct {
    int trigger_key_num;
    uint16_t trigger_keys[COMBO_MAX_LENGTH];
    bool trigger_pressed[COMBO_MAX_LENGTH];
    uint16_t keycode;
} Combo;

MatrixScanResult MatrixScan(void);
void KeyboardInit(void);
void SetKeycode(int keycode);
void ResetKeycode(int keycode);
void KeycodeSend();
int GetLayer(int SwitchIndex);
void TapDance(int tdindex, bool pressed);
void TapDanceTask(void);
bool TryComboPress(uint16_t keycode);
void TryRegisterCombo(void);
void ComboTask(void);
void PressSwitch(int SwitchIndex);
void PressKeycode(uint16_t keycode);
void ReleaseSwitch(int SwitchIndex);
void ReleaseKeycode(uint16_t keycode);
void HousekeepingTask (void);

HID_SendKeycode keyboardReport = {0};


GPIO_PinState pinState;

int Scanrate = 0; // GPIO 읽기 카운트

uint32_t Timer = 0; // 경과 시간 (ms)

uint32_t LastTimer = 0;

GPIO_TypeDef *GPIO_ABC[KEY_NUMBER] = {GPIOD, GPIOD, GPIOD, GPIOD, GPIOD, GPIOD, GPIOD, GPIOD, GPIOD, GPIOD, GPIOD, GPIOD, GPIOD, GPIOD, GPIOD, GPIOD, GPIOD };

int GPIO_Num[KEY_NUMBER] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3, GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_7, GPIO_PIN_8, GPIO_PIN_9, GPIO_PIN_10, GPIO_PIN_11, GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15 };

uint16_t Keycode[][KEY_NUMBER] = {
    {
        GPIOA0_0, GPIOA1_0, GPIOA2_0, GPIOA3_0, GPIOA4_0, GPIOA5_0, GPIOA6_0, GPIOA7_0, GPIOA8_0, GPIOA9_0, GPIOA10_0, GPIOA11_0, GPIOA12_0, GPIOA13_0, GPIOA14_0, GPIOA15_0,
        GPIOB0_0, GPIOB1_0, GPIOB2_0, GPIOB3_0, GPIOB4_0, GPIOB5_0, GPIOB6_0, GPIOB7_0, GPIOB8_0, GPIOB9_0, GPIOB10_0, GPIOB11_0, GPIOB12_0, GPIOB13_0, GPIOB14_0, GPIOB15_0,
        GPIOC0_0, GPIOC1_0, GPIOC2_0, GPIOC3_0, GPIOC4_0, GPIOC5_0, GPIOC6_0, GPIOC7_0, GPIOC8_0, GPIOC9_0, GPIOC10_0, GPIOC11_0, GPIOC12_0, GPIOC13_0, GPIOC14_0, GPIOC15_0,
        GPIOD0_0, GPIOD1_0, GPIOD2_0, GPIOD3_0, GPIOD4_0, GPIOD5_0, GPIOD6_0, GPIOD7_0, GPIOD8_0, GPIOD9_0, GPIOD10_0, GPIOD11_0, GPIOD12_0, GPIOD13_0, GPIOD14_0, GPIOD15_0,
        GPIOE0_0, GPIOE1_0, GPIOE2_0, GPIOE3_0, GPIOE4_0, GPIOE5_0, GPIOE6_0, GPIOE7_0, GPIOE8_0, GPIOE9_0, GPIOE10_0, GPIOE11_0, GPIOE12_0, GPIOE13_0, GPIOE14_0, GPIOE15_0
    },

    {
        GPIOA0_1, GPIOA1_1, GPIOA2_1, GPIOA3_1, GPIOA4_1, GPIOA5_1, GPIOA6_1, GPIOA7_1, GPIOA8_1, GPIOA9_1, GPIOA10_1, GPIOA11_1, GPIOA12_1, GPIOA13_1, GPIOA14_1, GPIOA15_1,
        GPIOB0_1, GPIOB1_1, GPIOB2_1, GPIOB3_1, GPIOB4_1, GPIOB5_1, GPIOB6_1, GPIOB7_1, GPIOB8_1, GPIOB9_1, GPIOB10_1, GPIOB11_1, GPIOB12_1, GPIOB13_1, GPIOB14_1, GPIOB15_1,
        GPIOC0_1, GPIOC1_1, GPIOC2_1, GPIOC3_1, GPIOC4_1, GPIOC5_1, GPIOC6_1, GPIOC7_1, GPIOC8_1, GPIOC9_1, GPIOC10_1, GPIOC11_1, GPIOC12_1, GPIOC13_1, GPIOC14_1, GPIOC15_1,
        GPIOD0_1, GPIOD1_1, GPIOD2_1, GPIOD3_1, GPIOD4_1, GPIOD5_1, GPIOD6_1, GPIOD7_1, GPIOD8_1, GPIOD9_1, GPIOD10_1, GPIOD11_1, GPIOD12_1, GPIOD13_1, GPIOD14_1, GPIOD15_1,
        GPIOE0_1, GPIOE1_1, GPIOE2_1, GPIOE3_1, GPIOE4_1, GPIOE5_1, GPIOE6_1, GPIOE7_1, GPIOE8_1, GPIOE9_1, GPIOE10_1, GPIOE11_1, GPIOE12_1, GPIOE13_1, GPIOE14_1, GPIOE15_1
    }
};

int ModifierBit[8] = {BIT_LCTL, BIT_LSFT, BIT_LALT, BIT_LGUI, BIT_RCTL, BIT_RSFT, BIT_RALT, BIT_RGUI};

uint8_t ModifierSum = 0b00000000;

uint32_t LayerState = 1;

int FromWhichLayer[KEY_NUMBER] = { -1 };

uint16_t TempKeycode = 0;

char UART_message[100];

uint32_t KeyTimer = 0;

uint32_t DebounceTimer[128] = { 0 };

uint32_t gpioA_state = 0; // GPIOA의 모든 핀 상태
uint32_t gpioB_state = 0; // GPIOB의 모든 핀 상태
uint32_t gpioC_state = 0; // GPIOC의 모든 핀 상태
uint32_t gpioD_state = 0; // GPIOD의 모든 핀 상태
uint32_t gpioE_state = 0; // GPIOE의 모든 핀 상태
uint32_t gpioF_state = 0; // GPIOD의 모든 핀 상태
uint32_t gpioG_state = 0; // GPIOE의 모든 핀 상태
uint32_t gpioH_state = 0; // GPIOD의 모든 핀 상태
uint32_t gpioI_state = 0; // GPIOE의 모든 핀 상태

uint32_t Last_gpioA_state = 0;
uint32_t Last_gpioB_state = 0;
uint32_t Last_gpioC_state = 0;
uint32_t Last_gpioD_state = 0;
uint32_t Last_gpioE_state = 0;
uint32_t Last_gpioF_state = 0;
uint32_t Last_gpioG_state = 0;
uint32_t Last_gpioH_state = 0;
uint32_t Last_gpioI_state = 0;

uint32_t changedPinA = 0; //변한 값만 1로 바귐
uint32_t changedPinB = 0;
uint32_t changedPinC = 0;
uint32_t changedPinD = 0;
uint32_t changedPinE = 0;
uint32_t changedPinF = 0;
uint32_t changedPinG = 0;
uint32_t changedPinH = 0;
uint32_t changedPinI = 0;

uint32_t CurrentTime = 0;

TapDanceState CurrentTapDance;

Combo combo_list[COMBO_NUMBER] = {{2, {KC_B, KC_C}, {false, false}, KC_D}};
uint16_t combo_pressed_keys[KEY_NUMBER] = {0};
uint32_t last_combo_pressed = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern USBD_HandleTypeDef hUsbDeviceHS;






// GPIO 상태를 스캔하고 변경된 핀 정보를 반환하는 함수
MatrixScanResult MatrixScan(void) {
    // 이전 GPIO 상태를 static으로 유지



    static uint32_t Last_gpioA_state = 0;
    static uint32_t Last_gpioB_state = 0;
    static uint32_t Last_gpioC_state = 0;
    static uint32_t Last_gpioD_state = 0;
    static uint32_t Last_gpioE_state = 0;
    static uint32_t Last_gpioF_state = 0;
    static uint32_t Last_gpioG_state = 0;
    static uint32_t Last_gpioH_state = 0;
    static uint32_t Last_gpioI_state = 0;

    // 현재 GPIO 상태 읽기
    uint32_t gpioA_state = (~(GPIOA->IDR)) & 0b1001111111010000; //핀 마스킹
    uint32_t gpioB_state = (~(GPIOB->IDR)) & 0b1100001111011100;
    uint32_t gpioC_state = (~(GPIOC->IDR)) & 0b1111111111110010;
    uint32_t gpioD_state = (~(GPIOD->IDR)) & 0b1111111111111111;
    uint32_t gpioE_state = (~(GPIOE->IDR)) & 0b1111111111110111;
    uint32_t gpioF_state = (~(GPIOF->IDR)) & 0b1111111111111111;
    uint32_t gpioG_state = (~(GPIOG->IDR)) & 0b1111111111111111;
    uint32_t gpioH_state = (~(GPIOH->IDR)) & 0b0011111111111100;
    uint32_t gpioI_state = (-(GPIOI->IDR)) & 0b1111111111111111;


    // 변경된 비트 계산 (XOR 연산)
    uint32_t changedPinA = gpioA_state ^ Last_gpioA_state;
    uint32_t changedPinB = gpioB_state ^ Last_gpioB_state;
    uint32_t changedPinC = gpioC_state ^ Last_gpioC_state;
    uint32_t changedPinD = gpioD_state ^ Last_gpioD_state;
    uint32_t changedPinE = gpioE_state ^ Last_gpioE_state;
    uint32_t changedPinF = gpioF_state ^ Last_gpioF_state;
    uint32_t changedPinG = gpioG_state ^ Last_gpioG_state;
    uint32_t changedPinH = gpioH_state ^ Last_gpioH_state;
    uint32_t changedPinI = gpioI_state ^ Last_gpioI_state;

    MatrixScanResult result;
    result.pinNumber = -1; // 초기값 (-1: 변경 없음)
    result.pinState = -1;


    // 포트 A~E 순서대로 변경된 핀 확인
    if (changedPinA != 0)
    {
        int bitPosition = -1;
        while ( changedPinA )
        {
            changedPinA >>= 1; // 오른쪽으로 시프트
            bitPosition++;
        }

        result.pinNumber = bitPosition; // 핀 번호 (포트 A는 0 ~ 15)
        result.pinState = (gpioA_state >> bitPosition) & 1; // 현재 상태

        if ( CurrentTime - DebounceTimer[result.pinNumber] > DEBOUNCE_TIME )
        {
        	Last_gpioA_state = gpioA_state;
        	DebounceTimer[result.pinNumber] = CurrentTime;
        }
        else
        {
            result.pinNumber = -1; // 초기값 (-1: 변경 없음)
            result.pinState = -1;
        }
        return result;
    }
    else if ( changedPinB != 0 )
    {
        int bitPosition = -1;
        while ( changedPinB )
        {
            changedPinB >>= 1;
            bitPosition++;
        }

        result.pinNumber = bitPosition + 16; // 핀 번호 (포트 B는 16 ~ 31)
        result.pinState = (gpioB_state >> bitPosition) & 1;

        if ( CurrentTime - DebounceTimer[result.pinNumber] > DEBOUNCE_TIME )
        {
        	Last_gpioB_state = gpioB_state;
        	DebounceTimer[result.pinNumber] = CurrentTime;
        }
        else
        {
            result.pinNumber = -1; // 초기값 (-1: 변경 없음)
            result.pinState = -1;
        }
        return result;
    }
    else if ( changedPinC != 0 )
    {
        int bitPosition = -1;
        while ( changedPinC )
        {
            changedPinC >>= 1;
            bitPosition++;
        }

        result.pinNumber = bitPosition + 32; // 핀 번호 (포트 C는 32 ~ 47)
        result.pinState = (gpioC_state >> bitPosition) & 1;

        if ( CurrentTime - DebounceTimer[result.pinNumber] > DEBOUNCE_TIME )
        {
            Last_gpioC_state = gpioC_state;
            DebounceTimer[result.pinNumber] = CurrentTime;
        }
        else
        {
            result.pinNumber = -1; // 초기값 (-1: 변경 없음)
            result.pinState = -1;
        }
        return result;
    }
    else if ( changedPinD != 0 )
    {
        int bitPosition = -1;
        while ( changedPinD )
        {
            changedPinD >>= 1;
            bitPosition++;
        }

        result.pinNumber = bitPosition + 48; // 핀 번호 (포트 D는 48 ~ 63)
        result.pinState = (gpioD_state >> bitPosition) & 1;
        char message[100];
        sprintf(message, "D | pinNumber = %d | pinState = %d \n\r", result.pinNumber, result.pinState);
        HAL_UART_Transmit(&huart4, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);

        if ( CurrentTime - DebounceTimer[result.pinNumber] > DEBOUNCE_TIME )
        {
        	Last_gpioD_state = gpioD_state;
        	DebounceTimer[result.pinNumber] = CurrentTime;
        }
        else
        {
            result.pinNumber = -1; // 초기값 (-1: 변경 없음)
            result.pinState = -1;
        }
        return result;
    }
    else if ( changedPinE != 0 )
    {
        int bitPosition = -1;
        while ( changedPinE )
        {
            changedPinE >>= 1;
            bitPosition++;
        }

        result.pinNumber = bitPosition + 64; // 핀 번호 (포트 E는 64 ~ 79)
        result.pinState = (gpioE_state >> bitPosition) & 1;

        if ( CurrentTime - DebounceTimer[result.pinNumber] > DEBOUNCE_TIME )
        {
        	Last_gpioE_state = gpioE_state;
        	DebounceTimer[result.pinNumber] = CurrentTime;
        }
        else
        {
            result.pinNumber = -1; // 초기값 (-1: 변경 없음)
            result.pinState = -1;
        }
        return result;
    }
    else if ( changedPinF != 0 )
    {
        int bitPosition = -1;
        while ( changedPinF )
        {
            changedPinF >>= 1;
            bitPosition++;
        }

        result.pinNumber = bitPosition + 80; // 핀 번호 (포트 E는 64 ~ 79)
        result.pinState = (gpioF_state >> bitPosition) & 1;

        if ( CurrentTime - DebounceTimer[result.pinNumber] > DEBOUNCE_TIME )
        {
        	Last_gpioF_state = gpioF_state;
        	DebounceTimer[result.pinNumber] = CurrentTime;
        }
        else
        {
            result.pinNumber = -1; // 초기값 (-1: 변경 없음)
            result.pinState = -1;
        }
        return result;
    }
    else if ( changedPinG != 0 )
    {
        int bitPosition = 0;
        while ( changedPinG )
        {
        	changedPinG >>= 1;
            bitPosition++;
        }

        result.pinNumber = bitPosition + 96; // 핀 번호 (포트 E는 64 ~ 79)
        result.pinState = (gpioG_state >> bitPosition) & 1;

        if ( CurrentTime - DebounceTimer[result.pinNumber] > DEBOUNCE_TIME )
        {
        	Last_gpioG_state = gpioG_state;
        	DebounceTimer[result.pinNumber] = CurrentTime;
        }
        else
        {
            result.pinNumber = -1; // 초기값 (-1: 변경 없음)
            result.pinState = -1;
        }
        return result;

    }
    else if ( changedPinH != 0 )
    {
        int bitPosition = 0;
        while ( changedPinH )
        {
        	changedPinH >>= 1;
            bitPosition++;
        }

        result.pinNumber = bitPosition + 112; // 핀 번호 (포트 E는 64 ~ 79)
        result.pinState = (gpioH_state >> bitPosition) & 1;

        if ( CurrentTime - DebounceTimer[result.pinNumber] > DEBOUNCE_TIME )
        {
        	Last_gpioH_state = gpioH_state;
        	DebounceTimer[result.pinNumber] = CurrentTime;
        }
        else
        {
            result.pinNumber = -1; // 초기값 (-1: 변경 없음)
            result.pinState = -1;
        }
        return result;
    }
	else if ( changedPinI != 0 )
	{
		int bitPosition = 0;
		while ( changedPinI )
		{
			changedPinI >>= 1;
			bitPosition++;
		}

		result.pinNumber = bitPosition + 128; // 핀 번호 (포트 E는 64 ~ 79)
		result.pinState = (gpioI_state >> bitPosition) & 1;

		if ( CurrentTime - DebounceTimer[result.pinNumber] > DEBOUNCE_TIME )
		{
			Last_gpioI_state = gpioI_state;
			DebounceTimer[result.pinNumber] = CurrentTime;
		}
		else
		{
			result.pinNumber = -1; // 초기값 (-1: 변경 없음)
			result.pinState = -1;
		}
		return result;
        

        char message[100];
        sprintf(message, "E | pinNumber = %d | pinState = %d \n\r", result.pinNumber, result.pinState);
        HAL_UART_Transmit(&huart4, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);

    }

    return result;
}



void KeyboardInit(void) {
    for(int i=0; i<KEY_NUMBER; ++i) {
        FromWhichLayer[i] = -1;
    }
    CurrentTapDance.dance_index = -1;
    CurrentTapDance.pressed = false;
    CurrentTapDance.count = 0;
    CurrentTapDance.activated = false;
    CurrentTapDance.last_pressed = 0;
    return;
}



void SetKeycode(int keycode)
{   
    if (keycode == keyboardReport.Keycode1 || keycode == keyboardReport.Keycode2 || keycode == keyboardReport.Keycode3 || keycode == keyboardReport.Keycode4 || keycode == keyboardReport.Keycode5 || keycode == keyboardReport.Keycode6)
    {
        return;
    }
    
    if (keyboardReport.Keycode1 == 0x00)
    {
        keyboardReport.Keycode1 = keycode;
    }
    else if (keyboardReport.Keycode2 == 0x00)
    {
        keyboardReport.Keycode2 = keycode;
    }
    else if (keyboardReport.Keycode3 == 0x00)
    {
        keyboardReport.Keycode3 = keycode;
    }
    else if (keyboardReport.Keycode4 == 0x00)
    {
        keyboardReport.Keycode4 = keycode;
    }
    else if (keyboardReport.Keycode5 == 0x00)
    {
        keyboardReport.Keycode5 = keycode;
    }
    else if (keyboardReport.Keycode6 == 0x00)
    {
        keyboardReport.Keycode6 = keycode;
    }
    return;
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
    sprintf(message, "KeycodeSend \n\r");
    HAL_UART_Transmit(&huart4, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
}

int GetLayer(int SwitchIndex) 
{   
    uint8_t now = 31;
    while (now!=0)
    {
        if( (LayerState&(1<<now)) && (Keycode[now][SwitchIndex]!=KC_TRNS))
        {
            break;
        }
        --now;
    }
    LayerState|=1; //레이어0이 혹시나 꺼졌을 경우 레이어 확인때 자동으로 켜지게
    return now;
}


void TapDance(int tdindex, bool pressed)
{
    if (pressed) {
        if (CurrentTapDance.dance_index == -1) {
            CurrentTapDance.dance_index = tdindex;
            CurrentTapDance.pressed = pressed;
            CurrentTapDance.count = 1;
            CurrentTapDance.activated = false;
            CurrentTapDance.last_pressed = CurrentTime;
            return;
        }

        CurrentTapDance.pressed = pressed;
        CurrentTapDance.count++;
        CurrentTapDance.activated = false;
        CurrentTapDance.last_pressed = CurrentTime;
        return;

    }else {
        if (CurrentTapDance.last_pressed>=TAPPING_TERM) {
            switch(CurrentTapDance.dance_index) {
                case 1: {
                    ReleaseKeycode(KC_B);
                    break;
                }
                default: {
                    break;
                }
            }
            CurrentTapDance.dance_index = -1;
            CurrentTapDance.activated = false;
            CurrentTapDance.count = 0;
            CurrentTapDance.pressed = false;
        }
        CurrentTapDance.pressed = pressed;
        return;
    }
    return;
}

void TapDanceTask(void) {
    if (CurrentTapDance.dance_index == -1) {
        return;
    }

    if (CurrentTime-CurrentTapDance.last_pressed>TAPPING_TERM) {
        CurrentTapDance.activated = true;
    }

    if (CurrentTapDance.activated) {
        // do tap dance things
        switch(CurrentTapDance.dance_index) {
            case 1: {
                if (CurrentTapDance.pressed && CurrentTime-CurrentTapDance.last_pressed>TAPPING_TERM) {
                    PressKeycode(KC_B);
                }else {
                    PressKeycode(KC_A);
                    KeycodeSend();
                    HAL_Delay(TAP_DELAY);
                    ReleaseKeycode(KC_A);
                    KeycodeSend();
                }
                break;
            }
            default: {
                break;
            }
        }
        CurrentTapDance.dance_index = -1;
        CurrentTapDance.activated = false;
        CurrentTapDance.count = 0;
        CurrentTapDance.pressed = false;
    }
    return;
}

bool TryComboPress(uint16_t keycode) {
    bool is_combo_key = false;
    #ifdef COMBO_STRICT_ORDER
    for (int i=0; i<COMBO_NUMBER; ++i) {
        for (int j==0; j<combo_list[i].trigger_key_num; ++j) {
            if (combo_list[i].trigger_pressed[j]) continue;
            if (combo_list[i].trigger_keys[j] == keycode) {
                is_combo_key = true;
                combo_list[i].trigger_pressed[j] = true;
            }
            break;
        }
    }
    #else // not COMBO_STRICT_ORDER
    int pressed_combo_keys_num = 0;
    
    while (pressed_combo_keys_num<COMBO_MAX_LENGTH && combo_pressed_keys[pressed_combo_keys_num]!=0) ++pressed_combo_keys_num;
    
    for (int i=0; i<COMBO_NUMBER; ++i) {
        if (combo_list[i].trigger_key_num>pressed_combo_keys_num) continue;
        
        is_combo_key = false;
        for (int j=0; j<pressed_combo_keys_num; ++j) {
            bool exist_in_list = false;
            for (int k=0; k<combo_list[i].trigger_key_num; ++k) {
                if (combo_list[i].trigger_keys[k]==combo_pressed_keys[j]) {
                    exist_in_list=true;
                    combo_list[i].trigger_pressed[k] = true;
                    break;
                }
            }
            is_combo_key |= exist_in_list;
        }
    }
    #endif // COMBO_STRICT_ORDER
    
    return is_combo_key;
}

void TryRegisterCombo(void) {
#ifdef COMBO_STRICT_ORDER
    for (int i=0; i<COMBO_NUMBER; ++i) {
        if (combo_list[i].trigger_pressed[combo_list[i].trigger_key_num]) {
            PressKeycode(combo_list[i].keycode);
            for (int j=0; j<COMBO_MAX_LENGTH; ++j) {
                combo_pressed_keys[j] = 0;
                combo_list[i].trigger_pressed[j] = false;
            }
        }
    }
    return;
#else // not COMBO_STRICT_ORDER
    for (int i=0; i<COMBO_NUMBER; ++i) {
        bool trigger = true;
        for (int j=0; j<combo_list[i].trigger_key_num; ++j) {
            if (!combo_list[i].trigger_pressed[j]) {
                trigger = false;
                break;
            }
        }
        if (trigger) {
            PressKeycode(combo_list[i].keycode);
            for (int j=0; j<COMBO_MAX_LENGTH; ++j) {
                combo_pressed_keys[j] = 0;
                combo_list[i].trigger_pressed[j] = false;
            }
        }

    }
    return;
#endif // COMBO_STRICT_ORDER
}

void ComboTask(void) {
    uint32_t now=HAL_GetTick();
    if (now-last_combo_pressed>COMBO_TIME) {
        for (int i=0; i<COMBO_MAX_LENGTH; ++i) {
            if (combo_pressed_keys[i]!=0) {
                PressKeycode(combo_pressed_keys[i]);
                combo_pressed_keys[i] = 0;
                KeycodeSend();
                HAL_Delay(TAP_DELAY);
            }
        }
        for (int i=0; i<COMBO_NUMBER; ++i) {
            for (int j=0; j<COMBO_MAX_LENGTH; ++j) {
                combo_pressed_keys[j] = 0;
                combo_list[i].trigger_pressed[j] = false;
            }
        }
    }
    return;
}

void PressSwitch(int SwitchIndex) {
    int layer = GetLayer(SwitchIndex);
    
    FromWhichLayer[SwitchIndex] = layer;
    
    if(TryComboPress(Keycode[layer][SwitchIndex])) {
        last_combo_pressed = HAL_GetTick();
        for (int i=0; i<COMBO_MAX_LENGTH; ++i) {
            if (combo_pressed_keys[i]==0) {
                combo_pressed_keys[i] = Keycode[layer][SwitchIndex];
                break;
            }
        }
        TryRegisterCombo();
        return;
    }
    
    PressKeycode(Keycode[layer][SwitchIndex]);
    return;
}

void PressKeycode(uint16_t keycode)
{
    if (keycode==0||keycode==1) return;

    // 콤보 가능성 없는 키가 들어오면 콤보가 깨지도록
    if (combo_pressed_keys[0] != 0) {
        last_combo_pressed = 0;
        ComboTask();
    }

    // 탭댄스 중에 다른 키 입력이 들어왔을 때 선입력된 탭댄스 발동 후에 키 처리
    if (CurrentTapDance.dance_index!=-1 && ((!IS_TD(keycode))||(CurrentTapDance.dance_index!=TD_TO_INDEX(keycode)))) {
        CurrentTapDance.activated = true;
        TapDanceTask();
    }

	if (IS_MOD(keycode))
	{
		ModifierSum = ModifierSum | ModifierBit[keycode - KC_MOD_MIN];
		keyboardReport.MODIFIER = ModifierSum;

		char message1[100];
		sprintf(message1, "ModifierPress = %d \n\r", ModifierBit[keycode - KC_MOD_MIN]);
		HAL_UART_Transmit(&huart4, (uint8_t *)message1, strlen(message1), HAL_MAX_DELAY);
	}
	else if (IS_FN(keycode) || IS_TD(keycode))
	{

		if ( IS_FN(keycode) )
		{
            LayerState|=(1<<FN_TO_LAYER_NUMBER(keycode));
		}
		else if ( IS_TD(keycode))
		{
			TapDance(TD_TO_INDEX(keycode), true/*pressed*/);
		}
		

		char message1[100];
		sprintf(message1, "Current Layer = %ld \n\r", LayerState);
		HAL_UART_Transmit(&huart4, (uint8_t *)message1, strlen(message1), HAL_MAX_DELAY);
	}
	else
	{
		SetKeycode(keycode);

		char message4[100];
		sprintf(message4, "PressKeycode = %d \n\r", keycode);
		HAL_UART_Transmit(&huart4, (uint8_t *)message4, strlen(message4), HAL_MAX_DELAY);
	}

    return;
}


void ReleaseSwitch(int SwitchIndex)
{
    int layer = FromWhichLayer[SwitchIndex];
    ReleaseKeycode(Keycode[layer][SwitchIndex]);
    FromWhichLayer[SwitchIndex] = -1;
    return;
}


void ReleaseKeycode(uint16_t keycode)
{
    if (keycode==KC_NO || keycode==KC_TRNS)
    {
        return;
    }

    if (combo_pressed_keys[0] != 0) {
        last_combo_pressed = 0;
        ComboTask();
    }
    
    if (IS_MOD(keycode))
    {
        ModifierSum = ModifierSum & ~(MOD_TO_BIT(keycode));
        keyboardReport.MODIFIER = ModifierSum;

        char message2[100];
        sprintf(message2, "ModifierRelease = %d\n\r", keycode);
        HAL_UART_Transmit(&huart4, (uint8_t *)message2, strlen(message2), HAL_MAX_DELAY);
    }
    else if (IS_FN(keycode))
    {
        LayerState &= (~(1<<FN_TO_LAYER_NUMBER(keycode)));
        LayerState |= 1; //레이어0은 항상 on
    }
    else if (IS_TD(keycode))
    {
        TapDance(TD_TO_INDEX(keycode), false/*released*/);
    }
    else
    {

        ResetKeycode(keycode);

        char message5[100];
        sprintf(message5, "ReleaseKeycode = %d \n\r", keycode);
        HAL_UART_Transmit(&huart4, (uint8_t *)message5, strlen(message5), HAL_MAX_DELAY);
    }

    return;

}

void HousekeepingTask (void) {
    TapDanceTask();
    ComboTask();
    return;
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

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */

  KeyboardInit();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {

    	CurrentTime = HAL_GetTick();

        if (CurrentTime - LastTimer >= 10000)
        {
            char message[100];
            sprintf(message, "Time(ms) = %ld |  Scanrate(Hz) = %d \n\r", CurrentTime, Scanrate / 10);
            HAL_UART_Transmit(&huart4, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);

            LastTimer = HAL_GetTick();
            Scanrate = 0;
        }

        Scanrate = Scanrate + 1;

        // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ

        MatrixScanResult Matrix = MatrixScan();

        if ( Matrix.pinNumber != -1 )
        {
			if ( Matrix.pinState == 1)
			{
				PressSwitch(Matrix.pinNumber);
				KeycodeSend();
			}
			else
			{
				ReleaseSwitch(Matrix.pinNumber);
				KeycodeSend();
			}

        }

        HousekeepingTask();

//        char message[100];
//        sprintf(message, "Test \n\r");
//        HAL_UART_Transmit(&huart4, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);


        //HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_2);


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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 10;
  RCC_OscInitStruct.PLL.PLLN = 384;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
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
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInitStruct.PLL3.PLL3M = 5;
  PeriphClkInitStruct.PLL3.PLL3N = 96;
  PeriphClkInitStruct.PLL3.PLL3P = 2;
  PeriphClkInitStruct.PLL3.PLL3Q = 10;
  PeriphClkInitStruct.PLL3.PLL3R = 2;
  PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_2;
  PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
  PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLL3;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pins : PE2 PE3 PE4 PE5
                           PE6 PE7 PE8 PE9
                           PE10 PE11 PE12 PE13
                           PE14 PE15 PE0 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC13 PC14 PC15 PC1
                           PC4 PC5 PC6 PC7
                           PC8 PC9 PC10 PC11
                           PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_1
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA4 PA6 PA7
                           PA8 PA9 PA10 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB14 PB15 PB3
                           PB4 PB6 PB7 PB8
                           PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

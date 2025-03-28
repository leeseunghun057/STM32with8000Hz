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
#define PIN_NUMBER 16*9
#define KEY_NUMBER 30
#define HOLD_TIME 200 // (ms)
#define DEBOUNCE_TIME 5 // (ms)
#define TAP_DELAY 50 // (us)

// Keycode are in main.h

#define EXCLUDE_PIN_A4 (1U << 4)

#define COMBO_MAX_LENGTH 10
#define COMBO_NUMBER 1
#define COMBO_TIME 50


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
void LayerOn(int layer);
void LayerOff(int layer);
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
int PinToSwitch(int PinNumber);

HID_SendKeycode keyboardReport = {0};


GPIO_PinState pinState;

int Scanrate = 0; // GPIO 읽기 카운트

uint32_t LastScanrateTimer = 0;

uint16_t Keycode[][KEY_NUMBER] = {
    {
        KC_Y,    TD(1),     TD(2),     KC_U,      KC_J,                      KC_K,    KC_D,     TD(3),     TD(4),     KC_W,
        TD(5),   TD(6),     TD(7),     TD(8),     KC_BSPC,                   KC_M,    TD(9),    TD(10),    TD(11),    TD(12),
        KC_Q,    KC_Z,      KC_SCLN,   KC_A,      KC_ENT,                    KC_B,    KC_F,     KC_G,      KC_V,      KC_X
    },

    {
        KC_Q,    TD(13),    TD(14),    KC_R,      KC_T,                      KC_Y,    KC_U,     TD(15),    TD(16),    KC_P,
        TD(17),  TD(18),    TD(19),    TD(20),    KC_G,                      KC_H,    TD(21),   TD(22),    TD(23),    TD(24),
        KC_Z,    KC_X,      KC_C,      KC_V,      KC_B,                      KC_N,    KC_M,     KC_SPC,    KC_BSPC,   KC_ENT
    },

    {
        KC_GRV,  KC_TRNS,    TD(25),      KC_TRNS,  C(KC_W), KC_TRNS, KC_TRNS,    TD(26),   KC_RALT,     KC_RCTL,
        KC_TAB,  KC_LEFT,    KC_DOWN,     KC_RIGHT, KC_DOT,  KC_DEL,  TD(27),     TD(28),   TD(29),      KC_TRNS,
        KC_ESC,  DF(0),      DF(1),       KC_COMM,  KC_QUOT, KC_TRNS, G(KC_LEFT), G(KC_UP), G(KC_RIGHT), KC_CAPS
    },

    {
        KC_PLUS, TD(30), KC_ASTR, KC_SLSH, KC_PERC, KC_BSLS, KC_LPRN, KC_RPRN, TD(31),  KC_RBRC,
        TD(32),  TD(33), TD(34),  TD(35),  KC_5,    KC_6,    TD(36),  TD(37),  TD(38),  TD(39),
        KC_EXLM, KC_GRV, KC_SCLN, KC_AMPR, KC_QUOT, KC_EQL,  KC_LT,   KC_GT,   KC_LCBR, KC_RCBR
    },

    {
        KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_F12,
        KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_F11,
        KC_F1,   KC_F2,   KC_F3,   KC_F4,   KC_F5,   KC_F6,   KC_F7,   KC_F8,   KC_F9,   KC_F10
    }


};

uint32_t LayerState = 1;
int BaseLayer = 0;

int FromWhichLayer[KEY_NUMBER] = { -1 };

char UART_message[100];

uint32_t DebounceTimer[PIN_NUMBER] = { 0 };

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

int Matrix[] = { PIN(GPIO_B, GPIO_4),  PIN(GPIO_B, GPIO_3),  PIN(GPIO_A, GPIO_15), PIN(GPIO_A, GPIO_14), PIN(GPIO_A, GPIO_13), PIN(GPIO_B, GPIO_13), PIN(GPIO_B, GPIO_12), PIN(GPIO_B, GPIO_11), PIN(GPIO_B, GPIO_10), PIN(GPIO_B, GPIO_2),
                 PIN(GPIO_B, GPIO_5),  PIN(GPIO_B, GPIO_6),  PIN(GPIO_B, GPIO_9),  PIN(GPIO_B, GPIO_8),  PIN(GPIO_B, GPIO_7),  PIN(GPIO_A, GPIO_5),  PIN(GPIO_A, GPIO_6),  PIN(GPIO_A, GPIO_7),  PIN(GPIO_B, GPIO_0),  PIN(GPIO_B, GPIO_1),
                 PIN(GPIO_C, GPIO_13), PIN(GPIO_C, GPIO_14), PIN(GPIO_C, GPIO_15), PIN(GPIO_F, GPIO_0),  PIN(GPIO_F, GPIO_1),  PIN(GPIO_A, GPIO_0),  PIN(GPIO_A, GPIO_1),  PIN(GPIO_A, GPIO_2),  PIN(GPIO_A, GPIO_3),  PIN(GPIO_A, GPIO_4)};
int PinMap[PIN_NUMBER] = {0};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
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

    // 현재 GPIO 상태 읽기
    uint32_t gpioA_state = (~(GPIOA->IDR)) & 0b1110000011111111; //& 0b1001111111010000; //핀 마스킹
    uint32_t gpioB_state = (~(GPIOB->IDR)) & 0b0011111111111111; //& 0b1100001111011100;
    uint32_t gpioC_state = (~(GPIOC->IDR)) & 0b1110000000000000; //& 0b1111111111110010;
    uint32_t gpioD_state = (~(GPIOD->IDR)) & 0b0000000000000000; //& 0b1111111111111111;
    uint32_t gpioE_state = (~(GPIOE->IDR)) & 0b0000000000000000; //& 0b1111111111110111;
    uint32_t gpioF_state = (~(GPIOF->IDR)) & 0b0000000000000011; //& 0b1111111111111111;


    // 변경된 비트 계산 (XOR 연산)
    uint32_t changedPinA = gpioA_state ^ Last_gpioA_state;
    uint32_t changedPinB = gpioB_state ^ Last_gpioB_state;
    uint32_t changedPinC = gpioC_state ^ Last_gpioC_state;
    uint32_t changedPinD = gpioD_state ^ Last_gpioD_state;
    uint32_t changedPinE = gpioE_state ^ Last_gpioE_state;
    uint32_t changedPinF = gpioF_state ^ Last_gpioF_state;

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

    for(int i=0; i<PIN_NUMBER; ++i) {
        PinMap[i] = -1;
    }
    for(int i=0; i<KEY_NUMBER; ++i) {
        PinMap[Matrix[i]] = i;
    }
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

    return;
}

void KeycodeSend()
{
    USBD_HID_SendReport(&hUsbDeviceH/S, (uint8_t *)&keyboardReport, sizeof(keyboardReport));

    return;
}

void LayerOn(int layer) {
    LayerState |= (1<<layer);
    return;
}
void LayerOff(int layer) {
    LayerState &= ~(1<<layer);
    return;
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

    if(now==0) {
        now=BaseLayer;
    }

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

        if ( CurrentTapDance.dance_index != tdindex) {
            CurrentTapDance.activated = true;
            TapDanceTask();
            CurrentTapDance.dance_index = tdindex;
            CurrentTapDance.pressed = pressed;
            CurrentTapDance.count = 1;
            CurrentTapDance.activated = false;
            CurrentTapDance.last_pressed = CurrentTime;
        }

        CurrentTapDance.pressed = pressed;
        CurrentTapDance.count++;
        CurrentTapDance.activated = false;
        CurrentTapDance.last_pressed = CurrentTime;
        return;

    }else {
        // tap의 release의 경우 뗌 시점이 아니라 tapping term이 끝날때 결정되므로 TapDancteTask에서 해결
        if (CurrentTime-CurrentTapDance.last_pressed>=TAPPING_TERM) {
            // Hold시의 release
            switch(tdindex) {
                case 1: LayerOff(2); break;
                case 2: LayerOff(3); break;
                case 3: LayerOff(3); break;
                case 4: LayerOff(2); break;
                case 5: ReleaseKeycode(KC_LGUI); break;
                case 6: ReleaseKeycode(KC_LSFT); break;
                case 7: ReleaseKeycode(KC_LCTL); break;
                case 8: ReleaseKeycode(KC_LALT); break;
                case 9: ReleaseKeycode(KC_LALT); break;
                case 10: ReleaseKeycode(KC_LCTL); break;
                case 11: ReleaseKeycode(KC_LSFT); break;
                case 12: ReleaseKeycode(KC_LGUI); break;
                case 13: LayerOff(2); break;
                case 14: LayerOff(3); break;
                case 15: LayerOff(3); break;
                case 16: LayerOff(2); break;
                case 17: ReleaseKeycode(KC_LGUI); break;
                case 18: ReleaseKeycode(KC_LSFT); break;
                case 19: ReleaseKeycode(KC_LCTL); break;
                case 20: ReleaseKeycode(KC_LALT); break;
                case 21: ReleaseKeycode(KC_LALT); break;
                case 22: ReleaseKeycode(KC_LCTL); break;
                case 23: ReleaseKeycode(KC_LSFT); break;
                case 24: ReleaseKeycode(KC_LGUI); break;
                case 25: LayerOff(4); break;
                case 26: LayerOff(4); break;
                case 27: ReleaseKeycode(KC_LALT); break;
                case 28: ReleaseKeycode(KC_LCTL); break;
                case 29: ReleaseKeycode(KC_LSFT); break;
                case 30: LayerOff(4); break;
                case 31: LayerOff(4); break;
                case 32: ReleaseKeycode(KC_LGUI); break;
                case 33: ReleaseKeycode(KC_LSFT); break;
                case 34: ReleaseKeycode(KC_LCTL); break;
                case 35: ReleaseKeycode(KC_LALT); break;
                case 36: ReleaseKeycode(KC_LALT); break;
                case 37: ReleaseKeycode(KC_LCTL); break;
                case 38: ReleaseKeycode(KC_LSFT); break;
                case 39: ReleaseKeycode(KC_LGUI); break;
                default: break;
            }
            CurrentTapDance.dance_index = -1;
            CurrentTapDance.activated = false;
            CurrentTapDance.count = 0;
            CurrentTapDance.pressed = false;
            CurrentTapDance.last_pressed = 0;
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
                    LayerOn(2);
                }else {
                    PressKeycode(KC_P);
                    KeycodeSend();
                    HAL_Delay(TAP_DELAY);
                    ReleaseKeycode(KC_P);
                    KeycodeSend();
                }
                break;
            }
            case 2: {
                if (CurrentTapDance.pressed && CurrentTime-CurrentTapDance.last_pressed>TAPPING_TERM) {
                    LayerOn(3);
                }else {
                    PressKeycode(KC_O);
                    KeycodeSend();
                    HAL_Delay(TAP_DELAY);
                    ReleaseKeycode(KC_O);
                    KeycodeSend();
                }
                break;
            }
            case 3: {
                if (CurrentTapDance.pressed && CurrentTime-CurrentTapDance.last_pressed>TAPPING_TERM) {
                    LayerOn(3);
                }else {
                    PressKeycode(KC_L);
                    KeycodeSend();
                    HAL_Delay(TAP_DELAY);
                    ReleaseKeycode(KC_L);
                    KeycodeSend();
                }
                break;
            }
            case 4: {
                if (CurrentTapDance.pressed && CurrentTime-CurrentTapDance.last_pressed>TAPPING_TERM) {
                    LayerOn(2);
                }else {
                    PressKeycode(KC_C);
                    KeycodeSend();
                    HAL_Delay(TAP_DELAY);
                    ReleaseKeycode(KC_C);
                    KeycodeSend();
                }
                break;
            }
            case 5: {
                if (CurrentTapDance.pressed && CurrentTime-CurrentTapDance.last_pressed>TAPPING_TERM) {
                    PressKeycode(KC_LGUI);
                }else {
                    PressKeycode(KC_I);
                    KeycodeSend();
                    HAL_Delay(TAP_DELAY);
                    ReleaseKeycode(KC_I);
                    KeycodeSend();
                }
                break;
            }
            case 6: {
                if (CurrentTapDance.pressed && CurrentTime-CurrentTapDance.last_pressed>TAPPING_TERM) {
                    PressKeycode(KC_LSFT);
                }else {
                    PressKeycode(KC_N);
                    KeycodeSend();
                    HAL_Delay(TAP_DELAY);
                    ReleaseKeycode(KC_N);
                    KeycodeSend();
                }
                break;
            }
            case 7: {
                if (CurrentTapDance.pressed && CurrentTime-CurrentTapDance.last_pressed>TAPPING_TERM) {
                    PressKeycode(KC_LCTL);
                }else {
                    PressKeycode(KC_E);
                    KeycodeSend();
                    HAL_Delay(TAP_DELAY);
                    ReleaseKeycode(KC_E);
                    KeycodeSend();
                }
                break;
            }
            case 8: {
                if (CurrentTapDance.pressed && CurrentTime-CurrentTapDance.last_pressed>TAPPING_TERM) {
                    PressKeycode(KC_LALT);
                }else {
                    PressKeycode(KC_SPC);
                    KeycodeSend();
                    HAL_Delay(TAP_DELAY);
                    ReleaseKeycode(KC_SPC);
                    KeycodeSend();
                }
                break;
            }
            case 9: {
                if (CurrentTapDance.pressed && CurrentTime-CurrentTapDance.last_pressed>TAPPING_TERM) {
                    PressKeycode(KC_LALT);
                }else {
                    PressKeycode(KC_H);
                    KeycodeSend();
                    HAL_Delay(TAP_DELAY);
                    ReleaseKeycode(KC_H);
                    KeycodeSend();
                }
                break;
            }
            case 10: {
                if (CurrentTapDance.pressed && CurrentTime-CurrentTapDance.last_pressed>TAPPING_TERM) {
                    PressKeycode(KC_LCTL);
                }else {
                    PressKeycode(KC_T);
                    KeycodeSend();
                    HAL_Delay(TAP_DELAY);
                    ReleaseKeycode(KC_T);
                    KeycodeSend();
                }
                break;
            }
            case 11: {
                if (CurrentTapDance.pressed && CurrentTime-CurrentTapDance.last_pressed>TAPPING_TERM) {
                    PressKeycode(KC_LSFT);
                }else {
                    PressKeycode(KC_S);
                    KeycodeSend();
                    HAL_Delay(TAP_DELAY);
                    ReleaseKeycode(KC_S);
                    KeycodeSend();
                }
                break;
            }
            case 12: {
                if (CurrentTapDance.pressed && CurrentTime-CurrentTapDance.last_pressed>TAPPING_TERM) {
                    PressKeycode(KC_LGUI);
                }else {
                    PressKeycode(KC_R);
                    KeycodeSend();
                    HAL_Delay(TAP_DELAY);
                    ReleaseKeycode(KC_R);
                    KeycodeSend();
                }
                break;
            }
            case 13: {
                if (CurrentTapDance.pressed && CurrentTime-CurrentTapDance.last_pressed>TAPPING_TERM) {
                    LayerOn(2);
                }else {
                    PressKeycode(KC_W);
                    KeycodeSend();
                    HAL_Delay(TAP_DELAY);
                    ReleaseKeycode(KC_W);
                    KeycodeSend();
                }
                break;
            }
            case 14: {
                if (CurrentTapDance.pressed && CurrentTime-CurrentTapDance.last_pressed>TAPPING_TERM) {
                    LayerOn(3);
                }else {
                    PressKeycode(KC_E);
                    KeycodeSend();
                    HAL_Delay(TAP_DELAY);
                    ReleaseKeycode(KC_E);
                    KeycodeSend();
                }
                break;
            }
            case 15: {
                if (CurrentTapDance.pressed && CurrentTime-CurrentTapDance.last_pressed>TAPPING_TERM) {
                    LayerOn(3);
                }else {
                    PressKeycode(KC_I);
                    KeycodeSend();
                    HAL_Delay(TAP_DELAY);
                    ReleaseKeycode(KC_I);
                    KeycodeSend();
                }
                break;
            }
            case 16: {
                if (CurrentTapDance.pressed && CurrentTime-CurrentTapDance.last_pressed>TAPPING_TERM) {
                    LayerOn(2);
                }else {
                    PressKeycode(KC_O);
                    KeycodeSend();
                    HAL_Delay(TAP_DELAY);
                    ReleaseKeycode(KC_O);
                    KeycodeSend();
                }
                break;
            }
            case 17: {
                if (CurrentTapDance.pressed && CurrentTime-CurrentTapDance.last_pressed>TAPPING_TERM) {
                    PressKeycode(KC_LGUI);
                }else {
                    PressKeycode(KC_A);
                    KeycodeSend();
                    HAL_Delay(TAP_DELAY);
                    ReleaseKeycode(KC_A);
                    KeycodeSend();
                }
                break;
            }
            case 18: {
                if (CurrentTapDance.pressed && CurrentTime-CurrentTapDance.last_pressed>TAPPING_TERM) {
                    PressKeycode(KC_LSFT);
                }else {
                    PressKeycode(KC_S);
                    KeycodeSend();
                    HAL_Delay(TAP_DELAY);
                    ReleaseKeycode(KC_S);
                    KeycodeSend();
                }
                break;
            }
            case 19: {
                if (CurrentTapDance.pressed && CurrentTime-CurrentTapDance.last_pressed>TAPPING_TERM) {
                    PressKeycode(KC_LCTL);
                }else {
                    PressKeycode(KC_D);
                    KeycodeSend();
                    HAL_Delay(TAP_DELAY);
                    ReleaseKeycode(KC_D);
                    KeycodeSend();
                }
                break;
            }
            case 20: {
                if (CurrentTapDance.pressed && CurrentTime-CurrentTapDance.last_pressed>TAPPING_TERM) {
                    PressKeycode(KC_LALT);
                }else {
                    PressKeycode(KC_F);
                    KeycodeSend();
                    HAL_Delay(TAP_DELAY);
                    ReleaseKeycode(KC_F);
                    KeycodeSend();
                }
                break;
            }
            case 21: {
                if (CurrentTapDance.pressed && CurrentTime-CurrentTapDance.last_pressed>TAPPING_TERM) {
                    PressKeycode(KC_LALT);
                }else {
                    PressKeycode(KC_J);
                    KeycodeSend();
                    HAL_Delay(TAP_DELAY);
                    ReleaseKeycode(KC_J);
                    KeycodeSend();
                }
                break;
            }
            case 22: {
                if (CurrentTapDance.pressed && CurrentTime-CurrentTapDance.last_pressed>TAPPING_TERM) {
                    PressKeycode(KC_LCTL);
                }else {
                    PressKeycode(KC_K);
                    KeycodeSend();
                    HAL_Delay(TAP_DELAY);
                    ReleaseKeycode(KC_K);
                    KeycodeSend();
                }
                break;
            }
            case 23: {
                if (CurrentTapDance.pressed && CurrentTime-CurrentTapDance.last_pressed>TAPPING_TERM) {
                    PressKeycode(KC_LSFT);
                }else {
                    PressKeycode(KC_L);
                    KeycodeSend();
                    HAL_Delay(TAP_DELAY);
                    ReleaseKeycode(KC_L);
                    KeycodeSend();
                }
                break;
            }
            case 24: {
                if (CurrentTapDance.pressed && CurrentTime-CurrentTapDance.last_pressed>TAPPING_TERM) {
                    PressKeycode(KC_LGUI);
                }else {
                    PressKeycode(KC_SCLN);
                    KeycodeSend();
                    HAL_Delay(TAP_DELAY);
                    ReleaseKeycode(KC_SCLN);
                    KeycodeSend();
                }
                break;
            }
            case 25: {
                if (CurrentTapDance.pressed && CurrentTime-CurrentTapDance.last_pressed>TAPPING_TERM) {
                    LayerOn(4);
                }else {
                    PressKeycode(KC_UP);
                    KeycodeSend();
                    HAL_Delay(TAP_DELAY);
                    ReleaseKeycode(KC_UP);
                    KeycodeSend();
                }
                break;
            }
            case 26: {
                if (CurrentTapDance.pressed && CurrentTime-CurrentTapDance.last_pressed>TAPPING_TERM) {
                    LayerOn(4);
                }else {
                    PressKeycode(KC_PGUP);
                    KeycodeSend();
                    HAL_Delay(TAP_DELAY);
                    ReleaseKeycode(KC_PGUP);
                    KeycodeSend();
                }
                break;
            }
            case 27: {
                if (CurrentTapDance.pressed && CurrentTime-CurrentTapDance.last_pressed>TAPPING_TERM) {
                    PressKeycode(KC_HOME);
                }else {
                    PressKeycode(KC_J);
                    KeycodeSend();
                    HAL_Delay(TAP_DELAY);
                    ReleaseKeycode(KC_J);
                    KeycodeSend();
                }
                break;
            }
            case 28: {
                if (CurrentTapDance.pressed && CurrentTime-CurrentTapDance.last_pressed>TAPPING_TERM) {
                    PressKeycode(KC_LCTL);
                }else {
                    PressKeycode(KC_PGDN);
                    KeycodeSend();
                    HAL_Delay(TAP_DELAY);
                    ReleaseKeycode(KC_PGDN);
                    KeycodeSend();
                }
                break;
            }
            case 29: {
                if (CurrentTapDance.pressed && CurrentTime-CurrentTapDance.last_pressed>TAPPING_TERM) {
                    PressKeycode(KC_LSFT);
                }else {
                    PressKeycode(KC_END);
                    KeycodeSend();
                    HAL_Delay(TAP_DELAY);
                    ReleaseKeycode(KC_END);
                    KeycodeSend();
                }
                break;
            }
            case 30: {
                if (CurrentTapDance.pressed && CurrentTime-CurrentTapDance.last_pressed>TAPPING_TERM) {
                    LayerOn(4);
                }else {
                    PressKeycode(KC_MINS);
                    KeycodeSend();
                    HAL_Delay(TAP_DELAY);
                    ReleaseKeycode(KC_MINS);
                    KeycodeSend();
                }
                break;
            }
            case 31: {
                if (CurrentTapDance.pressed && CurrentTime-CurrentTapDance.last_pressed>TAPPING_TERM) {
                    LayerOn(4);
                }else {
                    PressKeycode(KC_LBRC);
                    KeycodeSend();
                    HAL_Delay(TAP_DELAY);
                    ReleaseKeycode(KC_LBRC);
                    KeycodeSend();
                }
                break;
            }
            case 32: {
                if (CurrentTapDance.pressed && CurrentTime-CurrentTapDance.last_pressed>TAPPING_TERM) {
                    PressKeycode(KC_LGUI);
                }else {
                    PressKeycode(KC_1);
                    KeycodeSend();
                    HAL_Delay(TAP_DELAY);
                    ReleaseKeycode(KC_1);
                    KeycodeSend();
                }
                break;
            }
            case 33: {
                if (CurrentTapDance.pressed && CurrentTime-CurrentTapDance.last_pressed>TAPPING_TERM) {
                    PressKeycode(KC_LSFT);
                }else {
                    PressKeycode(KC_2);
                    KeycodeSend();
                    HAL_Delay(TAP_DELAY);
                    ReleaseKeycode(KC_2);
                    KeycodeSend();
                }
                break;
            }
            case 34: {
                if (CurrentTapDance.pressed && CurrentTime-CurrentTapDance.last_pressed>TAPPING_TERM) {
                    PressKeycode(KC_LCTL);
                }else {
                    PressKeycode(KC_3);
                    KeycodeSend();
                    HAL_Delay(TAP_DELAY);
                    ReleaseKeycode(KC_3);
                    KeycodeSend();
                }
                break;
            }
            case 35: {
                if (CurrentTapDance.pressed && CurrentTime-CurrentTapDance.last_pressed>TAPPING_TERM) {
                    PressKeycode(KC_LALT);
                }else {
                    PressKeycode(KC_4);
                    KeycodeSend();
                    HAL_Delay(TAP_DELAY);
                    ReleaseKeycode(KC_4);
                    KeycodeSend();
                }
                break;
            }
            case 36: {
                if (CurrentTapDance.pressed && CurrentTime-CurrentTapDance.last_pressed>TAPPING_TERM) {
                    PressKeycode(KC_LALT);
                }else {
                    PressKeycode(KC_7);
                    KeycodeSend();
                    HAL_Delay(TAP_DELAY);
                    ReleaseKeycode(KC_7);
                    KeycodeSend();
                }
                break;
            }
            case 37: {
                if (CurrentTapDance.pressed && CurrentTime-CurrentTapDance.last_pressed>TAPPING_TERM) {
                    PressKeycode(KC_LCTL);
                }else {
                    PressKeycode(KC_8);
                    KeycodeSend();
                    HAL_Delay(TAP_DELAY);
                    ReleaseKeycode(KC_8);
                    KeycodeSend();
                }
                break;
            }
            case 38: {
                if (CurrentTapDance.pressed && CurrentTime-CurrentTapDance.last_pressed>TAPPING_TERM) {
                    PressKeycode(KC_LSFT);
                }else {
                    PressKeycode(KC_9);
                    KeycodeSend();
                    HAL_Delay(TAP_DELAY);
                    ReleaseKeycode(KC_9);
                    KeycodeSend();
                }
                break;
            }
            case 39: {
                if (CurrentTapDance.pressed && CurrentTime-CurrentTapDance.last_pressed>TAPPING_TERM) {
                    PressKeycode(KC_LGUI);
                }else {
                    PressKeycode(KC_0);
                    KeycodeSend();
                    HAL_Delay(TAP_DELAY);
                    ReleaseKeycode(KC_0);
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
    if(SwitchIndex==-1) {return;}

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

	if (IS_MOD(keycode))
	{
		keyboardReport.MODIFIER = keyboardReport.MODIFIER | (1<<(keycode - KC_MOD_MIN));
	}
	else if (IS_FN(keycode) || IS_TD(keycode))
	{

		if ( IS_FN(keycode) )
		{
            LayerState|=(1<<FN_TO_LAYER(keycode));
		}
		else if ( IS_TD(keycode))
		{
			TapDance(TD_TO_INDEX(keycode), true);
		}



	}else if (IS_DF(keycode)){
        BaseLayer=DF_TO_LAYER(keycode);
    }
	else
	{
		SetKeycode(keycode);
	}

    return;
}


void ReleaseSwitch(int SwitchIndex)
{
    if(SwitchIndex==-1) {return;}
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
        keyboardReport.MODIFIER = keyboardReport.MODIFIER & ~(MOD_TO_BIT(keycode));
    }
    else if (IS_FN(keycode))
    {
        LayerState &= (~(1<<FN_TO_LAYER(keycode)));
        LayerState |= 1; //레이어0은 항상 on
    }
    else if (IS_TD(keycode))
    {
        TapDance(TD_TO_INDEX(keycode), false);
    }
    else
    {
        ResetKeycode(keycode);
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


              LastScanrateTimer = HAL_GetTick();
              Scanrate = 0;
          }

          Scanrate = Scanrate + 1;

          // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ

          MatrixScanResult Matrix = MatrixScan();

          if ( Matrix.pinNumber != -1 )
          {
  			if ( Matrix.pinState == 1)
  			{
  				PressSwitch(PinMap[Matrix.pinNumber]);
  				KeycodeSend();
  			}
  			else
  			{
  				ReleaseSwitch(PinMap[Matrix.pinNumber]);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_SET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PF0 PF1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA4 PA5 PA6 PA7
                           PA13 PA14 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB11 PB12 PB13 PB3
                           PB4 PB5 PB6 PB7
                           PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

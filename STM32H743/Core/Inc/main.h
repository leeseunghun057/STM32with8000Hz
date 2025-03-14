/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

#define HOLDTAP_SHIFTING 0x5000
#define HOLDTAP_MIN 0x5000

#define KC_NO 0x0000
#define KC_TRNS 0x0001

#define KC_A 0x0004
#define KC_B 0x0005
#define KC_C 0x0006
#define KC_D 0x0007
#define KC_E 0x0008
#define KC_F 0x0009
#define KC_G 0x000A
#define KC_H 0x000B
#define KC_I 0x000C
#define KC_J 0x000D
#define KC_K 0x000E
#define KC_L 0x000F
#define KC_M 0x0010
#define KC_N 0x0011
#define KC_O 0x0012
#define KC_P 0x0013
#define KC_Q 0x0014
#define KC_R 0x0015
#define KC_S 0x0016
#define KC_T 0x0017
#define KC_U 0x0018
#define KC_V 0x0019
#define KC_W 0x001A
#define KC_X 0x001B
#define KC_Y 0x001C
#define KC_Z 0x001D

// Custom Keycode
#define KC_MOD_MIN 0x0100
#define KC_MOD_MAX 0x0107
#define KC_LCTL 0x0100
#define KC_LSFT 0x0101
#define KC_LALT 0x0102
#define KC_LGUI 0x0103
#define KC_RCTL 0x0104
#define KC_RSFT 0x0105
#define KC_RALT 0x0106
#define KC_RGUI 0x0107
#define IS_MOD(keycode) ((keycode>=KC_MOD_MIN)&&(keycode<=KC_MOD_MAX))
#define MOD_TO_BIT(keycode) (1 << (keycode-KC_MOD_MIN))

#define KC_FN_MIN 0x5000
#define KC_FN_MAX 0x50FF // layer1 when press
#define KC_TD_MIN 0x5100 // switch layer to 0
#define KC_TD_MAX 0x51FF // switch layer to 1
#define FN(layernumber) (KC_FN_MIN+layernumber)
#define FN_TO_LAYER_NUMBER(fnx) (fnx-KC_FN_MIN)
#define IS_FN(keycode) ((keycode>=KC_FN_MIN)&&(keycode<=KC_FN_MAX))
#define TD(index) (KC_TD_MIN+index)
#define TD_TO_INDEX(tdx) (tdx-KC_FN_MIN)
#define IS_TD(keycode) ((keycode>=KC_TD_MIN)&&(keycode<=KC_TD_MAX))

#define GPIOA0_0   0
#define GPIOA1_0   0
#define GPIOA2_0   0
#define GPIOA3_0   0
#define GPIOA4_0   0
#define GPIOA5_0   0
#define GPIOA6_0   0
#define GPIOA7_0   0
#define GPIOA8_0   0
#define GPIOA9_0   0
#define GPIOA10_0  0
#define GPIOA11_0  0
#define GPIOA12_0  0
#define GPIOA13_0  0
#define GPIOA14_0  0
#define GPIOA15_0  0

#define GPIOB0_0   0
#define GPIOB1_0   0
#define GPIOB2_0   0
#define GPIOB3_0   0
#define GPIOB4_0   0
#define GPIOB5_0   0
#define GPIOB6_0   0
#define GPIOB7_0   0
#define GPIOB8_0   0
#define GPIOB9_0   0
#define GPIOB10_0  0
#define GPIOB11_0  0
#define GPIOB12_0  0
#define GPIOB13_0  0
#define GPIOB14_0  0
#define GPIOB15_0  0

#define GPIOC0_0   0
#define GPIOC1_0   0
#define GPIOC2_0   0
#define GPIOC3_0   0
#define GPIOC4_0   0
#define GPIOC5_0   0
#define GPIOC6_0   0
#define GPIOC7_0   0
#define GPIOC8_0   0
#define GPIOC9_0   0
#define GPIOC10_0  0
#define GPIOC11_0  0
#define GPIOC12_0  0
#define GPIOC13_0  0
#define GPIOC14_0  0
#define GPIOC15_0  0

#define GPIOD0_0   TD(1)
#define GPIOD1_0   KC_B
#define GPIOD2_0   KC_C
#define GPIOD3_0   KC_D
#define GPIOD4_0   KC_D
#define GPIOD5_0   KC_D
#define GPIOD6_0   KC_D
#define GPIOD7_0   0
#define GPIOD8_0   0
#define GPIOD9_0   0
#define GPIOD10_0  0
#define GPIOD11_0  0
#define GPIOD12_0  0
#define GPIOD13_0  0
#define GPIOD14_0  0
#define GPIOD15_0  0

#define GPIOE0_0   0
#define GPIOE1_0   0
#define GPIOE2_0   0
#define GPIOE3_0   0
#define GPIOE4_0   0
#define GPIOE5_0   0
#define GPIOE6_0   0
#define GPIOE7_0   0
#define GPIOE8_0   0
#define GPIOE9_0   0
#define GPIOE10_0  0
#define GPIOE11_0  0
#define GPIOE12_0  0
#define GPIOE13_0  0
#define GPIOE14_0  0
#define GPIOE15_0  0



#define GPIOA0_1   0
#define GPIOA1_1   0
#define GPIOA2_1   0
#define GPIOA3_1   0
#define GPIOA4_1   0
#define GPIOA5_1   0
#define GPIOA6_1   0
#define GPIOA7_1   0
#define GPIOA8_1   0
#define GPIOA9_1   0
#define GPIOA10_1  0
#define GPIOA11_1  0
#define GPIOA12_1  0
#define GPIOA13_1  0
#define GPIOA14_1  0
#define GPIOA15_1  0

#define GPIOB0_1   0
#define GPIOB1_1   0
#define GPIOB2_1   0
#define GPIOB3_1   0
#define GPIOB4_1   0
#define GPIOB5_1   0
#define GPIOB6_1   0
#define GPIOB7_1   0
#define GPIOB8_1   0
#define GPIOB9_1   0
#define GPIOB10_1  0
#define GPIOB11_1  0
#define GPIOB12_1  0
#define GPIOB13_1  0
#define GPIOB14_1  0
#define GPIOB15_1  0

#define GPIOC0_1   0
#define GPIOC1_1   0
#define GPIOC2_1   0
#define GPIOC3_1   0
#define GPIOC4_1   0
#define GPIOC5_1   0
#define GPIOC6_1   0
#define GPIOC7_1   0
#define GPIOC8_1   0
#define GPIOC9_1   0
#define GPIOC10_1  0
#define GPIOC11_1  0
#define GPIOC12_1  0
#define GPIOC13_1  0
#define GPIOC14_1  0
#define GPIOC15_1  0

#define GPIOD0_1   KC_C
#define GPIOD1_1   KC_E
#define GPIOD2_1   KC_F
#define GPIOD3_1   0
#define GPIOD4_1   0
#define GPIOD5_1   0
#define GPIOD6_1   0
#define GPIOD7_1   0
#define GPIOD8_1   0
#define GPIOD9_1   0
#define GPIOD10_1  0
#define GPIOD11_1  0
#define GPIOD12_1  0
#define GPIOD13_1  0
#define GPIOD14_1  0
#define GPIOD15_1  0

#define GPIOE0_1   0
#define GPIOE1_1   0
#define GPIOE2_1   0
#define GPIOE3_1   0
#define GPIOE4_1   0
#define GPIOE5_1   0
#define GPIOE6_1   0
#define GPIOE7_1   0
#define GPIOE8_1   0
#define GPIOE9_1   0
#define GPIOE10_1  0
#define GPIOE11_1  0
#define GPIOE12_1  0
#define GPIOE13_1  0
#define GPIOE14_1  0
#define GPIOE15_1  0

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define RS485_DE_Pin GPIO_PIN_15
#define RS485_DE_GPIO_Port GPIOC
#define WAKEUP_Pin GPIO_PIN_0
#define WAKEUP_GPIO_Port GPIOA
#define ADC_SENSOR_Pin GPIO_PIN_1
#define ADC_SENSOR_GPIO_Port GPIOA
#define ADC_PWR_VBAT_Pin GPIO_PIN_2
#define ADC_PWR_VBAT_GPIO_Port GPIOA
#define AVCC_PWR_ON_Pin GPIO_PIN_3
#define AVCC_PWR_ON_GPIO_Port GPIOA
#define I2C2_SCL_Pin GPIO_PIN_10
#define I2C2_SCL_GPIO_Port GPIOB
#define I2C2_SDA_Pin GPIO_PIN_11
#define I2C2_SDA_GPIO_Port GPIOB
#define SYS_PWR_ON_Pin GPIO_PIN_12
#define SYS_PWR_ON_GPIO_Port GPIOB
#define LED4_Pin GPIO_PIN_13
#define LED4_GPIO_Port GPIOB
#define LED5_Pin GPIO_PIN_14
#define LED5_GPIO_Port GPIOB
#define CHRG_nON_Pin GPIO_PIN_8
#define CHRG_nON_GPIO_Port GPIOA
#define RS485_TX_Pin GPIO_PIN_9
#define RS485_TX_GPIO_Port GPIOA
#define RS485_RX_Pin GPIO_PIN_10
#define RS485_RX_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define CHRG_nSTATUS_Pin GPIO_PIN_15
#define CHRG_nSTATUS_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_3
#define LED1_GPIO_Port GPIOB
#define CHRG_nPOWER_GOOD_Pin GPIO_PIN_4
#define CHRG_nPOWER_GOOD_GPIO_Port GPIOB
#define ZUMMER_Pin GPIO_PIN_7
#define ZUMMER_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_8
#define LED2_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_9
#define LED3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define MODBUS_DEVICE_INFO 			"Mobile Detector"
#define	InfoSizeRegNum						60000
#define	InfoRegNum								60001
#define FLASH_LOCATION			((uint32_t)0x08008000-FLASH_PAGE_SIZE)			//FLASH_BANK1_END			0x2000000
#define EndAddr					((uint32_t)08000000)
#define FLASH_SIZE				(((unsigned short *)&Params.crc - (unsigned short *)&Params.var_res_level)/2+1)
#define CONFIG_REGISTERS_COUNT 	((unsigned short *)&Params.crc - (unsigned short *) & Params.var_res_level)

#define MODBUS_DEVICE_INFO 			"Mobile Detector"
#define	InfoSizeRegNum						60000
#define	InfoRegNum								60001


#define TRESH			1
#define DANGER		2

#define READY_ON					Params.Flags_Read|=32;
#define READY_OFF					Params.Flags_Read&=~32;
#define FLAG_WRITE		1

#define V_ref 		3.33

#define UP		1
#define DOWN	0
#define LED1( mode )					(mode == UP ?   HAL_TIM_PWM_Start( &htim2, TIM_CHANNEL_2): HAL_TIM_PWM_Stop( &htim2, TIM_CHANNEL_2))
#define LED2( mode )					(mode == UP ?   HAL_TIM_PWM_Start( &htim10, TIM_CHANNEL_1): HAL_TIM_PWM_Stop( &htim10, TIM_CHANNEL_1))
#define LED3( mode )					(mode == UP ?   HAL_TIM_PWM_Start( &htim11, TIM_CHANNEL_1): HAL_TIM_PWM_Stop( &htim11, TIM_CHANNEL_1))
#define LED4( mode )					(mode == UP ?   HAL_TIM_PWM_Start( &htim9, TIM_CHANNEL_1): HAL_TIM_PWM_Stop( &htim9, TIM_CHANNEL_1))
#define LED5( mode )					(mode == UP ?   HAL_TIM_PWM_Start( &htim9, TIM_CHANNEL_2): HAL_TIM_PWM_Stop( &htim9, TIM_CHANNEL_2))
#define ZUMMER( mode )				(mode == UP ? HAL_GPIO_WritePin(ZUMMER_GPIO_Port, ZUMMER_Pin, GPIO_PIN_SET): HAL_GPIO_WritePin(ZUMMER_GPIO_Port, ZUMMER_Pin, GPIO_PIN_RESET))

/* USER CODE END Private defines */

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

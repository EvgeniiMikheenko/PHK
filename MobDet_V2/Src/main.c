/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l1xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <mb_slave.h>
#include <modbus_config.h>
#include <math.h>


#define ON_OFF_Delay 3000

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void Flash_Store( void );
bool Flash_Restore( void );
void get_sens_data(void);
void Check_alarm( mb_Regs *Params );
void ResetFilter_data( mb_Regs *Params);
void Charger( void );
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
mb_Regs Params;
bool isPassword = false;
bool adc_new = false;
bool i2c_new = false;
bool adc_flag = false;
float i2c_data = 0;
bool i2c_msg_flag = false;
bool flag_T = true;
bool i2c_is_busy = false;
uint8_t duty1 = 50;
MbErrorCodes_t usart_init_msg;
uint32_t greenlight = 0;
uint8_t alarm = 0;
uint8_t tick = 0;
uint8_t init = 1;
uint16_t lpBuf[2];
HAL_StatusTypeDef status;
bool new_adc_data = false;
float battery_voltage;
bool btn_first_press = false;
bool ready = false;
bool charge = false;



unsigned * BUF1 = (unsigned *) &(Params.var_res_level);
unsigned * BUF2 = (unsigned *) ((uint32_t)0x08008000-FLASH_PAGE_SIZE);
mb_Regs * Flash_Params = (mb_Regs *) FLASH_LOCATION;
unsigned short * PARAMS_BUF = (unsigned short *) & (Params.var_res_level);
volatile uint32_t NbrOfPage = 0x00;
uint32_t EraseCounter = 0x00, Address = 0x00;
HAL_StatusTypeDef flash_status;
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  
  HAL_Delay( 1000 );
  
  HAL_GPIO_WritePin( SYS_PWR_ON_GPIO_Port, SYS_PWR_ON_Pin, GPIO_PIN_SET );
  HAL_GPIO_WritePin( AVCC_PWR_ON_GPIO_Port,AVCC_PWR_ON_Pin, GPIO_PIN_SET );
  LED1(UP);
  btn_first_press = true;
  
  
  MX_DMA_Init();
  MX_ADC_Init();
  MX_I2C2_Init();
 // MX_TIM2_Init();
  //MX_TIM9_Init();
  //MX_TIM10_Init();
 // MX_TIM11_Init();
  MX_USART1_UART_Init();
  	NVIC_EnableIRQ(USART1_IRQn);
	__HAL_UART_ENABLE_IT( &huart1, UART_IT_RXNE );
	__HAL_UART_ENABLE_IT( &huart1, UART_IT_TC );
	__HAL_UART_ENABLE_IT( &huart1, UART_IT_ERR);
  
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */

 // SetPeriod(&htim3, 5000, 12000000);
  SetPeriod(&htim4, 5000, 12000000);
  
  SetPeriod(&htim2, 100, 12000000);
  SetPeriod(&htim11, 50, 12000000);
  SetPeriod(&htim10, 50, 12000000);
  SetPeriod(&htim9, 50, 12000000);
  
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);
  
  
  

  
  SetDutyCycle( &htim2, duty1 );
  SetDutyCycle( &htim11, duty1 );
  SetDutyCycle( &htim10, duty1 );
  SetDutyCycle( &htim9, duty1 );
//  SetDutyCycle( &htim9, duty1 );
          
//  HAL_TIM_PWM_Start( &htim2, TIM_CHANNEL_2);
//  HAL_TIM_PWM_Start( &htim11, TIM_CHANNEL_1);
//  HAL_TIM_PWM_Start( &htim10, TIM_CHANNEL_1);
//  HAL_TIM_PWM_Start( &htim9, TIM_CHANNEL_1);
//  HAL_TIM_PWM_Start( &htim9, TIM_CHANNEL_2);
  
  
  HAL_ADC_Start( &hadc );
    
//    
//            LED1(UP);
//            LED2(UP);
//            LED3(UP);
//            LED4(UP);
//            LED5(UP); 
    
    
//  Params.Fact.A = 0;
//  Params.Fact.B = 0;
//  Params.Fact.C = 1360;
//  Params.Fact.D = 0;
  
  
    
  bool read_flash = Flash_Restore();
  
  
  
	
    
    
    
  usart_init_msg = mb_slave_init( &m_mbParam );  
  
   ready = true;
  
  
  greenlight = HAL_GetTick();
  LED1(UP);
  GHARGE_OFF;
    status = HAL_ADC_Start_DMA( &hadc, (uint32_t*)lpBuf, 2 );
   READY_ON; 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    
      	if( alarm == DANGER ) 
		{
		
				ZUMMER(UP);
		
		}
		else if( alarm == 0 )
		{
		
			ZUMMER( DOWN );
		
		}
		
		
		if( tick >= 100 )
		{
            
        
            
            
			
			if( Params.Flags_write & FLAG_WRITE )
			{
				Params.Flags_write &= ~FLAG_WRITE;
				Flash_Store();

			}
	

			

			tick = 0;
			get_sens_data();
			

            
            if(!new_adc_data)
                HAL_ADC_Start_DMA( &hadc, (uint32_t*)lpBuf, 2 );
			
			Check_alarm( &Params );
			
			tick = 0;
				__WFI();
			
		}
		else{
			
			__WFI();
			
			
		}
      
      
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
//        float a1 ;
//        float b1 ;
//        float c1 ;
//        float d1;
uint16_t low_bat = 0;
void get_sens_data(void)
{


		
	//	adc_getval( &Params, adc_flag );
	//	adc_new = true;
	//	Filter_data( &Params );
////	
////		uint16_t tmp = HAL_ADC_GetValue(&hadc);
////		Params.Sensor_voltage = ((tmp * V_ref) / 4096.0f)*1000;
////		HAL_ADC_Start( &hadc );
    
        new_adc_data = false;
    
		float kA = -2327;
		float kA1 = -2.327;
		float kB = 241500;
		float kB1 = 241.5;
		uint16_t tmp = lpBuf[1];
		Params.Sensor_voltage = ((tmp * V_ref) / 4096.0f)*1000;
    
    
         float tmp_vbat = ((lpBuf[0] * V_ref) / 4096.0f);
      //  float ddd = ((float)Params.Fact.A)/1000 * pow(tmp_vbat,3) + ((float)Params.Fact.B)/1000 * pow(tmp_vbat,2) + ((float)Params.Fact.C)/1000 *tmp_vbat +((float)Params.Fact.D)/1000;
//         a1 = ((float)Params.Fact.A * pow(tmp_vbat,3) ) / 1000;
//         b1 = ((float)Params.Fact.B * pow(tmp_vbat,2) ) / 1000;
//         c1 = ((float)Params.Fact.C * tmp_vbat ) / 1000;
//        d1 = ((float)Params.Fact.D/1000);       
        Params.v_bat = (short) (tmp_vbat*1000);
 //      Params.v_bat = (((float)Params.Fact.A * pow(tmp_vbat,3) ) ) + (((float)Params.Fact.B * pow(tmp_vbat,2) ) ) + (((float)Params.Fact.C * tmp_vbat ) ) + ((float)Params.Fact.D);
        
        if( Params.v_bat < MIN_BAT )
        {
            if( Params.v_bat < OFF_BAT && !(Params.write_reg & CHARGING) )
            {
                low_bat++;
                if( low_bat > 3000 )
                  HAL_GPIO_WritePin( SYS_PWR_ON_GPIO_Port, SYS_PWR_ON_Pin, GPIO_PIN_RESET );
            }
            Params.write_reg |= BATTERY_LOW;
            GHARGE_ON;
            if( low_bat > 0 )
                low_bat--;
        
        }
        else if( Params.v_bat >= MAX_BAT )
        {
            if( low_bat > 0 )
                low_bat--;
            Params.write_reg &= ~BATTERY_LOW;
            GHARGE_OFF;
            
        }
        else
        {
            if( low_bat > 0 )
                low_bat--;
            GHARGE_ON;
            Params.write_reg &= ~BATTERY_LOW;
        }
      //   Params.v_bat
               
        Charger();

        HAL_ADC_Start( &hadc );


	
	
	
	
		
	
//		float Sensor_voltage = ((tmp * V_ref) / 4096.0f)*1000;
//		float s_v = (((float)tmp * V_ref) / 4096.0f);
//		float v_r = (float)Params.ref_null/1000;
//		float tst1 = (s_v - v_r)/(kA1 * (float)Params.var_res_level + kB1);
//		float tst = (Sensor_voltage - (float)Params.ref_null)/(kA * (float)Params.var_res_level + kB);
	//	Params.test1 = (short)(tst * 1000);
//		Params.test2 = (short)(tst1 * 1000);
		//Params.Sensor_voltage = (1000*( Sensor_voltage - (Params.ref_null/10) ))/( kA * Params.var_res_level + kB );
		HAL_ADC_Start( &hadc );
	
	
		if(i2c_msg_flag)
		{
			if(flag_T)
			{
				i2c_data = I2C_StartTransmission( T );
				i2c_is_busy = true;
				//if( i2c_data != Write_Error && i2c_data !=Writed_T && i2c_data !=Read_Error && i2c_data !=Crc8_Error && i2c_data != Conversion_Error )
				if( i2c_data != Write_Error && i2c_data !=Writed_T && i2c_data !=Read_Error && i2c_data !=Crc8_Error && i2c_data != Conversion_Error )
				{
					Params.Temperature = i2c_data;
					flag_T = false;
					i2c_is_busy = false;
					
				}
				
				if(i2c_data == Writed_T)
					i2c_is_busy = false;
				
				
				if( i2c_data == Write_Error || i2c_data ==Read_Error || i2c_data ==Crc8_Error || i2c_data == Conversion_Error)
				{
					flag_T = false;
					i2c_is_busy = false;
				}
			}
			else
			{
			
				i2c_data = I2C_StartTransmission( RH );
				i2c_is_busy = true;
				//if( i2c_data != Write_Error && i2c_data !=Writed_T && i2c_data !=Read_Error && i2c_data !=Crc8_Error && i2c_data != Conversion_Error )
				if( i2c_data != Write_Error && i2c_data !=Writed_T && i2c_data !=Read_Error && i2c_data !=Crc8_Error && i2c_data != Conversion_Error )
				{
					Params.Humidity = i2c_data;
					flag_T = true;
					i2c_is_busy = false;
				}
				if( i2c_data == Write_Error || i2c_data ==Read_Error || i2c_data ==Crc8_Error || i2c_data == Conversion_Error)
				{
					flag_T = true;
					i2c_is_busy = false;
				}
				if( i2c_data == Writed_T )
					i2c_is_busy = false;
			
			}
			i2c_msg_flag = false;
			HAL_TIM_Base_Start_IT( &htim4 );
			i2c_new = true;
			if( init == 1 )
			{
				ResetFilter_data( &Params );
				init = 0;
			}
			else
				return;
//				Filter_data( &Params );
		}
		
		


};

void ResetFilter_data( mb_Regs *Params)
{

	i2c_new = false;
	adc_new = false;

	

};

void Check_alarm( mb_Regs *Params )
{
			bool is_alarm = false;
		alarm = 0;
		if( Params->Sensor_voltage > (Params->pos_tresh + Params->ref_null) )
		{
			LED3(UP);
			alarm = TRESH;
			is_alarm = true;
            Params->write_reg |= TOP_TRSH;
		}
		else
		{
			LED3(DOWN);
            Params->write_reg &= ~TOP_TRSH;
			
		}
		
		if( Params->Sensor_voltage > (Params->pos_Tresh_Danger + Params->ref_null) )
		{
			LED2(UP);
			LED3(DOWN);
			alarm = DANGER;
			is_alarm = true;
            Params->write_reg |= TOP_DNG;
		}
		else
		{
		
			LED2(DOWN);
            Params->write_reg &= ~TOP_DNG;
		}
		
		if( Params->Sensor_voltage < (Params->ref_null - Params->neg_tresh ))		// вниз порог
		{
				LED5(UP);
				alarm = TRESH;
				is_alarm = true;
                Params->write_reg |= BOT_TRSH;
		}
		else
		{
			LED5(DOWN);
            Params->write_reg &= ~BOT_TRSH;
			
		}
		
		if( Params->Sensor_voltage < (Params->ref_null - Params->neg_Tresh_Danger) )	//вниз опасно
		{
			LED4(UP);
			LED5(DOWN);
			alarm = DANGER;
			is_alarm = true;
            Params->write_reg |= BOT_DNG;
		}
		else
		{
			LED4(DOWN);
            Params->write_reg &= ~BOT_DNG;
			
		}
		
		
//		if( !is_alarm )
//			alarm = 0;
		
		
		
		
		

};
uint16_t f_size1 = (((unsigned short *)&Params.crc - (unsigned short *)&Params.var_res_level)/2+1);
void  Flash_Store( void )
{

		EnterCritSection();
		
		uint8_t j=0;
		uint32_t end_addr = ((uint32_t)0x08008000);
		uint32_t flash_location = ((uint32_t)0x08008000-FLASH_PAGE_SIZE+4)	;
		
	
	
		for ( int i=0;i<f_size1;i++)
		{
			if (BUF1[i]!=BUF2[i])
				j=1;
		}
        
        
        
		
		if(!j) {
			ExitCritSection();
			return;
		}
	
		HAL_FLASH_Unlock();
		
		
		NbrOfPage = (end_addr - (flash_location-4)) / FLASH_PAGE_SIZE;
//		HAL_FLASH_Unlock();
		//стереть
		for(EraseCounter = 0; (EraseCounter < NbrOfPage); EraseCounter++)
		{
			
			//FLASH_PageErase(flash_location - 4 + (FLASH_PAGE_SIZE * EraseCounter));			
			FLASH_PageErase(FLASH_LOCATION + (FLASH_PAGE_SIZE * EraseCounter));
			while(FLASH->SR & FLASH_SR_BSY )
			{}
			//FLASH_WaitForLastOperation(10);
		}
		CLEAR_BIT(FLASH->PECR,FLASH_PECR_ERASE);
		CLEAR_BIT(FLASH->PECR,FLASH_PECR_PROG);
		HAL_FLASH_Lock();

		Address = FLASH_LOCATION;
		int i=FLASH_SIZE;

		Params.crc = CalcCrc16((unsigned char *) & (Params.var_res_level), f_size1*4-4);
		
		HAL_FLASH_Unlock();

		for (i=0;i<f_size1;i++) {
			if (BUF1[i]!=BUF2[i])
			{
			//	FLASH->CR &= ~(FLASH_CR_PER);
			//	FLASH->CR |= FLASH_CR_PG;
				FLASH->SR |= FLASH_FLAG_PGAERR;
				FLASH->ACR &= ~(1<<2);
				flash_status = HAL_FLASH_Program( FLASH_TYPEPROGRAM_WORD, flash_location-4+(i*4), BUF1[i]);		//BUF1[i]
				while(FLASH->SR & FLASH_SR_BSY )
				{}
			}
		}
		//flash_status = HAL_FLASH_Program( FLASH_TYPEPROGRAM_WORD, 0x08007C96, 0x09);
			
		HAL_FLASH_Lock();
			
		ExitCritSection();
				
	
};







	uint16_t sdf;
uint32_t sdfgdf;

bool Flash_Restore( void )
{
	uint16_t crc;
	uint16_t f_size = (((unsigned short *)&Params.crc - (unsigned short *)&Params.var_res_level)/2+1);
	sdf = ((unsigned short *)&Params.crc - (unsigned short *)&Params.var_res_level);
	sdf = (((unsigned short *)&Params.crc - (unsigned short *)&Params.var_res_level)/2+1);
	sdf = (((unsigned short *)&Params.crc - (unsigned short *)&Params.var_res_level)/2+1) * 4;
	sdf = (((unsigned short *)&Params.crc - (unsigned short *)&Params.var_res_level)/2+1) * 4 - 4;
	sdfgdf = ((unsigned short *)&Params.crc - (unsigned short *) & Params.var_res_level);
	sdfgdf = FLASH_LOCATION + ((unsigned short *)&Params.crc - (unsigned short *) & Params.var_res_level)*2;
	
//	sdfgdf = *((uint16_t *)(FLASH_LOCATION + sdfgdf*2));
//	sdfgdf = FLASH_LOCATION;
//	sdfgdf = FLASH_PAGE_SIZE;
//	sdfgdf = ((uint32_t)0x08008000-FLASH_PAGE_SIZE);
	
//	sdf = FLASH_SIZE;
//	sdf = (FLASH_SIZE)*4;
//	sdf = FLASH_SIZE*4-4;
	
	crc = CalcCrc16((unsigned char *) Flash_Params, sdf);
	if (crc != *((uint16_t *)(sdfgdf)))		//(FLASH_LOCATION + sdfgdf*2)
	
	{
		for (int i = 0; i < f_size ; i++)
			PARAMS_BUF[i] = 0;
		return false;
		
	}
	else
	{
		for (int i = 0; i < f_size  ; i++)
			BUF1[i] = BUF2[i];
		return true;
		
	}
};


void TIM4_IRQHandler(void)			// второе определение закомментировано в stm32f1xx_it.c
{

		__HAL_TIM_CLEAR_IT( &htim4, TIM_IT_UPDATE);
		adc_flag = true;
    
    HAL_GPIO_TogglePin(ZUMMER_GPIO_Port, ZUMMER_Pin);
		
	if( !i2c_is_busy )
		i2c_msg_flag = true;

};

int tyy = 0;
void TIM3_IRQHandler( void )				// второе определение закомментировано в stm32f1xx_it.c
{
			__HAL_TIM_CLEAR_IT( &htim3, TIM_IT_UPDATE);
					mb_slave_timer_expired(&m_mbParam);

}



uint32_t m_hostTimeout = 0;
bool m_isPasswordValid = false;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) 
{
static bool isRidgeEn = false;
		
		if( huart->Instance == USART1)
		{
			
			uint8_t err = HAL_UART_GetError( &huart1 );
			

			
			
			
			uint8_t data = (uint8_t)huart->Instance->DR;
			
			MbErrorCodes_t err_msg = mb_slave_recive( &m_mbParam, data );
			
			m_mbParam.usb = false;
		//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
//			
//			uint8_t data11 = huart->Instance->DR;
//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
//			HAL_UART_Transmit_IT(&huart1, &data11, 1);
//			while(HAL_UART_GetState( &huart1 ) == HAL_UART_STATE_BUSY)
//				{};
//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
		
		};

};


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if( huart->Instance == USART1)
	{
	 if( huart->Instance == USART1 )
			mb_tx(&m_mbParam);
	}
};





MbExceptionCodes_t mb_read_registers( uint8_t *lpDstBuf, uint16_t startAddr, uint16_t count ) {
	if(lpDstBuf == NULL)
		return MB_EXC_ILLEGAL_FUNCTION;
	
	isPassword = false;
	m_hostTimeout = 0;
	m_isPasswordValid = false;
	uint16_t endAddr = startAddr + count;
	
	int registersCount = sizeof(Params) / 2;
	
	if(startAddr < registersCount) {	
		// ?????? ???????????? ?????????
		if( endAddr > registersCount )
			return MB_EXC_ILLEGAL_DATA_ADDR;
		
		uint16_t *lpRegs = (uint16_t*)&Params;
		uint16_t reg;
		//
		for(uint32_t i = startAddr, j = 0; i < endAddr; i++, j += 2) {
			reg = lpRegs[i];
			lpDstBuf[j] = (uint8_t)(reg >> 8);
			lpDstBuf[j + 1] = (uint8_t)(reg);
		}
		//
		return MB_EXC_NONE;
	}
	
	//-------------------------------------------------------------------------
	uint16_t size = strlen( MODBUS_DEVICE_INFO );
	
	// ?????? ?????????? ???????? ? ?????????? ?? ??????????
	if(startAddr == InfoSizeRegNum) {
		
		if(count != 1) {
			return MB_EXC_ILLEGAL_DATA_VALUE;
		}
		//
		lpDstBuf[0] = (uint8_t)size >> 8;
		lpDstBuf[1] = (uint8_t)(size);
		//
		return MB_EXC_NONE;
	}
	
	// -----------------------------------------------------------------------
	// ?????? ?????????? ?? ??????????
	if(startAddr == InfoRegNum) {
		
		if(count != size) {
			return MB_EXC_ILLEGAL_DATA_VALUE;
		}
		
		char *ptr = MODBUS_DEVICE_INFO; // tBuf;
		uint16_t reg;
		
		for(uint32_t i = 0, j = 0; i < size; i++, j += 2) {
			reg = (uint16_t)ptr[i];
			lpDstBuf[j] = (uint8_t)(reg >> 8);
			lpDstBuf[j + 1] = (uint8_t)(reg);
		}
		//
		return MB_EXC_NONE;
	}
	
	return MB_EXC_ILLEGAL_FUNCTION;
};



MbExceptionCodes_t mb_write_registers( uint16_t startAddr, uint16_t count, uint8_t *lpSrcBuf ) {
	
//	if( !isPassword )
//	{
//	
//		if(((lpSrcBuf[0]<<8) + lpSrcBuf[1]) == Password	)
//			isPassword = true;
//			
//		return NULL;
//	}
	
	
	
	if(lpSrcBuf == NULL)
		return MB_EXC_ILLEGAL_FUNCTION;
	
	m_hostTimeout = 0;
	m_isPasswordValid = false;
	uint16_t endAddr = startAddr + count;
	uint16_t *tmp_ptr = (uint16_t *)&Params;
	
	uint16_t tmp_reg_val;
	
	
	//int i = startAddr;
	int j = startAddr;
	
	
	
	for( int i = startAddr; i < endAddr; i++)
	{
		tmp_reg_val = (lpSrcBuf[j - startAddr ] << 8) + lpSrcBuf[j - startAddr+1];
		
		//tmp_ptr[i+2] = lpSrcBuf[i - startAddr];
		tmp_ptr[i] = tmp_reg_val;
		j+=2;
//		tmp_ptr [1] = 0x77;
//		tmp_ptr [0] = 0x9988;
//	
	}
	
	
	
	

		//
		return MB_EXC_NONE;
	
	

	//return MB_EXC_ILLEGAL_FUNCTION;
	
};

void HAL_ADC_ConvCpltCallback( ADC_HandleTypeDef* hadc ) {
	if(hadc->Instance == ADC1) {
		//EnterCritSection();
		

		new_adc_data = true;

		//ExitCritSection();	
		
		
		
		return;
	}
}

bool green = false;
uint16_t pwr_btn = 0;
uint8_t fast_green = 0;
void HAL_SYSTICK_Callback(void)
{
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_SYSTICK_Callback could be implemented in the user file
   */
	if( !ready )
        return;
    
    if( HAL_GPIO_ReadPin(WAKEUP_GPIO_Port, WAKEUP_Pin )== GPIO_PIN_SET )
    {
        pwr_btn++;
    }
    else if( pwr_btn > 0 )
    {
        pwr_btn = 0;
    }

    if( pwr_btn >= ON_OFF_Delay   )
    {
        
        
            HAL_GPIO_WritePin( SYS_PWR_ON_GPIO_Port, SYS_PWR_ON_Pin, GPIO_PIN_RESET );
            LED1(UP);
            LED2(UP);
            LED3(UP);
            LED4(UP);
            LED5(UP);
        while(true){
            READY_OFF;
        };
    }
    
    
	tick >= 250 ? tick = 0: tick++;
	
	if( HAL_GetTick() >= greenlight + 1800 && !green )
	{
		LED1(UP);
		green = true;
        if(Params.write_reg & BATTERY_LOW)
        {
            greenlight -= 50;
            fast_green++;
        }
		//HAL_GPIO_TogglePin( GPIOA, GPIO_PIN_1 );
		greenlight = HAL_GetTick();
		
		if( alarm == TRESH )
		{
		
			ZUMMER( UP );
		
		}
		else if( alarm == 0 )
		{
			
			ZUMMER( DOWN );
		
		}
		
		
		
		
	}
	else if( HAL_GetTick() >= greenlight + 200 && green )
	{
        if(!charge)
        {
			LED1( DOWN );
			green = false;
		//	HAL_GPIO_TogglePin( GPIOA, GPIO_PIN_1 );
			greenlight = HAL_GetTick();
            if(Params.write_reg & BATTERY_LOW)
            {
                if(fast_green <3)
                {
                    fast_green++;
                    greenlight -=1600;              
                }
                else
                {
                    fast_green = 0;
                }
            }
        }
			if( alarm == TRESH )
			{
		
				ZUMMER( DOWN );
		
			}
	}
}


void Charger( void )
{



        if( (HAL_GPIO_ReadPin(CHRG_nPOWER_GOOD_GPIO_Port ,CHRG_nPOWER_GOOD_Pin) == GPIO_PIN_RESET) && (HAL_GPIO_ReadPin(CHRG_nSTATUS_GPIO_Port ,CHRG_nSTATUS_Pin) == GPIO_PIN_RESET) )
        {
                Params.write_reg |= CHARGING;
         //       GHARGE_ON;
               charge = true;
        }
//         else        if( HAL_GPIO_ReadPin(CHRG_nSTATUS_GPIO_Port ,CHRG_nSTATUS_Pin) == GPIO_PIN_SET )
//        {
//            Params.Flags_write &= ~CHARGING;
//            Params.Flags_write |= CHARGE_ERR;
//       //     GHARGE_OFF;
//            charge = false;
//        }
        else       if((HAL_GPIO_ReadPin(CHRG_nPOWER_GOOD_GPIO_Port ,CHRG_nPOWER_GOOD_Pin) == GPIO_PIN_SET))
        {
            Params.write_reg &= ~CHARGING;
            Params.write_reg |= NO_POW_SOUCE;
         //   GHARGE_OFF;
            charge = false;
        }

            
        
    

    
};



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

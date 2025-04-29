/*
 * 3KW_PWR_tim.c
 *
 *  Created on: May 23, 2022
 *      Author: user
 */
#include "stm32f4xx.h"
#include "PWR_3KW_driver.h"
#include "main.h"

const void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) 
{
	if(htim->Instance == htim1.Instance) 
	{
		uint16_t temp = 0;

		Timer_Count++; 
		
		flag_20us();
		flag_2ms();
		flag_1ms();
		
		temp = (uint32_t)(Timer_Count % ADC_PERIOD); 

		if(0 == temp)
		{
			uint8_t f = 1U;

			if(f == gval.timer_flag_gval.prepare_pwr_cont_timer_flag)
			{
				gval.timer_gval.prepare_pwr_cont_timer++;
			}
			else
			{
				gval.timer_gval.prepare_pwr_cont_timer = 0;
			}
		}
		else
		{
			__NOP();
		}

		flag_100ms();
		flag_1s();
	}
}

void flag_20us()
{
	if(0 == gval.flag.tlc59482_tx_flag) // SCLK Clock Generation 40us
	{
		__NOP();
	}
	else
	{
		TLC59482_SCLKS();
	}
	
}

void flag_2ms()
{
	uint32_t temp = 0;

	temp = (uint32_t)(Timer_Count % DISPLAY_PERIOD); // 16개 : 40us 640us

	if(0 == temp)
	{
		gval.flag.panel_flag = 1U;  // 2ms 시행시간
	}
	else
	{
		__NOP();
	}
}

void flag_1ms()
{
	uint32_t temp = 0;

	temp = (uint32_t)(Timer_Count % FAN_CHK_PERIOD);
	if(0 == temp)
	{
		gval.flag.fan_flag = 1U; // 1ms 확인 시행시간 0.4us, 다음 파형 시간 1.15ms

		uint8_t a = 1U;
		if(a == gval.flag.low_cont_state)
		{
			gval.timer_gval.low_load_cont_timer++;
		}
		else
		{
			gval.timer_gval.low_load_cont_timer = 0;
		}

	}
	else
	{
		__NOP();
	}
}

void flag_10ms()
{
	uint32_t temp = 0;

	temp = (uint32_t)(Timer_Count % ADC_PERIOD); // 10ms timer
	if(0 == temp)
	{
		uint8_t f = 1U;

		if(f == gval.timer_flag_gval.prepare_pwr_cont_timer_flag)
		{
			gval.timer_gval.prepare_pwr_cont_timer++;
		}
		else
		{
			gval.timer_gval.prepare_pwr_cont_timer = 0;
		}
	}
	else
	{
		__NOP();
	}
}

void flag_100ms()
{
	uint32_t temp = 0;

	temp = (uint32_t)(Timer_Count % CAN_PERIOD);
	if(0 == temp)
	{
		uint8_t e = 1U;
		uint8_t d = 1U;

		toggle_Test_LED_1();
		toggle_Test_LED_2();

		gval.flag.can_flag      = 1U; // 100ms 주기
		gval.flag.errCheck_flag = 1U; // 100ms 주기

		if(e == gval.flag.low_load_counter_on_timer)
		{
			Low_Load_Count_On_Timer++;
		}
		else
		{
			Low_Load_Count_On_Timer = 0;
		}

		if(e == gval.timer_flag_gval.wait_timer_flag)
		{
			gval.timer_gval.wait_timer++;
		}
		else
		{
			gval.timer_gval.wait_timer = 0;
		}


		//
		if(d == gval.timer_flag_gval.start_timer_flag)
		{
			uint8_t c = 20U;

			gval.timer_gval.start_timer++;
			if(c <= gval.timer_gval.start_timer)
			{
				gval.timer_gval.start_timer = 20U;
			}
			else
			{
				__NOP();
			}
		}
		else
		{
			gval.timer_gval.start_timer = 0;
		}		

		//
		if(d == gval.timer_flag_gval.Pulse_timer_flag)
		{
			uint8_t c = 50U;

			gval.timer_gval.triger_timer++;
			if(c <= gval.timer_gval.triger_timer)
			{
				gval.timer_gval.triger_timer = 50U;
			}
			else
			{
				__NOP();
			}
		}
		else
		{
			gval.timer_gval.triger_timer = 0;
		}
	}
	else
	{
		__NOP();
	}
}

void flag_1s()
{
	uint32_t temp = 0;

	temp = (uint32_t)(Timer_Count % ONE_SEC_PERIOD);

	if(0 == temp)
	{
		uint16_t b = 0;
		uint8_t d = 1U;

		gval.flag.one_sec = 1U; // 1s
		gval.test_sec++;
		fan_l++;
		fan_r++;
		if(d == gval.gpio_read.Pwr_Sw_OnOff)
		{
			Sec_Time_Count++;
		}
		else
		{
			Sec_Time_Count = 0;
		}

		if(d == gval.flag.wait_volt_time_flag)
		{
			uint8_t g = 5U;

			Wait_Volt_Time++;
			if(g < Wait_Volt_Time)
			{
				Wait_Volt_Time = 5U;
			}
			else
			{
				__NOP();
			}
		}
		else
		{
			Wait_Volt_Time = 0;
		}

		if(ONE_DAY < Sec_Time_Count)
		{
			Sec_Time_Count = 0;
		}

		if(d == gval.flag.three_min_start_flag)
		{
			Three_min_Time_Count++;
		}
		else
		{
			Three_min_Time_Count = 0;
		}

		if(d == gval.flag.low_load_counter_off_timer)
		{
			Low_Load_Count_Off_Timer++;
		}
		else
		{
			Low_Load_Count_Off_Timer = 0;
		}

		if(d == gval.flag.wait_current_time)
		{
			wait_current_time++;
		}
		else
		{
			wait_current_time = 0;
		}

		if(d == gval.flag.pre_overcurrent_flag)
		{
			gval.pre_overcurrent_time++;
		}
		else
		{
			gval.pre_overcurrent_time = 0;
		}

		b = 50000U;
		if(b == Timer_Count)
		{
			Timer_Count = 0;
		}
		else
		{
			__NOP();
		}

	}
	else
	{
		__NOP();
	}


}


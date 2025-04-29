/*
 * 3KW_PWR_core.c
 *
 *  Created on: May 23, 2022
 *      Author: user
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "PWR_3KW_driver.h"
#include "math.h"

uint32_t capture_1[2] = {0 , 0};
uint32_t capture_2[2] = {0 , 0};

// 초기화 동작작
void System_Init()
{
	PWR_Parameter_Init();	// PWR 파라메터 초기화
	Self_Test_3kw_PWR();	// 셀프 테스트 8초
}

// 무한루프 운용동작 
void System_core()
{
	// 리셋 동작중에는 다른 동작을 하지 않는다.
	if(gval.flag.HwReset_flag == ON)
	{
		// 셀프 테스트 종료 이후 리셋한다.
		// HW 에러 상태로 안켜지는 증상 개선 관련 추가 코드
		PWR_Bias_Reset();

		// 리셋 완료시에는 완료 플래그를 올림
		gval.flag.HwReset_ResetComplete_flag = ON;
		gval.flag.HwReset_flag = OFF;
	}
	else
	{
		PWR_Operation();	// 전면 스위치 입력에 따른 ON/OFF 제어
		ADC_Calc();			// ADC 폴링으로 읽는 과정 (2ms주기)
		Error_Check();		// OVP, OCP, OTP 에러 체크
		Low_load();			// 전류 상태별 전원 모드 제어
	}

	LED_Display();			// LED 제어
	Display_LED_Control();	// Display 명령어 삽입
	CAN_Communication();	// CAN 데이터 송신 (100ms주기)

}

void PWR_Parameter_Init()
{
	uint16_t b = 0;

	// timer start
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	HAL_TIM_Base_Start_IT(&htim1);

	InitRingBuffer();
	HAL_UART_Receive_IT(&huart1, &gval.communication.usart.debug.RXD[0], 1);

	Init_CAN(&hcan1);

	gval.flag.Sout_flag = 0;
	gval.control_data.read_sout = 0;
	gval.control_data.rd = 0;
	gval.flag.tlc59482_tx_flag = 1U;

	gval.control_data.tlc59482_tx_data = init_val_32;
	gval.flag.fcwrten_flag = 1U;
	gval.control_data.tlc59482_cnt = 0;

	gval.can_cbit.fw_version.version_number = (uint32_t)FW_3KW_Version;
	gval.can_cbit.fw_version.version_char = (uint8_t)FW_3KW_Version_Type;

	gval.flag.error_temp_flag = 0;

	gval.error_counter.over_voltage_counter1 = 0;
	gval.error_counter.over_voltage_counter2 = 0;
	gval.error_counter.over_voltage_counter3 = 0;

	gval.error_counter.non_over_load_counter = 0;
	gval.error_counter.over_load_counter = 0;

	gval.can_cbit.fw_lamp_status.PulseMode = OFF;
	gval.can_cbit.fw_lamp_status.temporyDataModeData = 0;

	Gpio_Init();

	gval.flag.HwReset_ResetComplete_flag = OFF;
	gval.flag.HwReset_flag = 0;

	gval.flag.test_flag = 1U;
	gval.flag.one_sec = 0;

	// 2022.08.14.
	// 추후 최초 자체 점검 중에는 LED 점멸 기능 추가
	// 점검 후에는 AC 입력 녹색 점등
	b = 16U;
	for(uint16_t a = 0; a < b; a++)
	{
		gval.tlc59482_data.tlc59482_data_32[a] = 0;
	}

	b = AD_CH;
	for(uint16_t i = 0; i < b; i++ )
	{
		AD_Filter_d[i] = 0.0f;

	}
	NTC_TABLE_CREATER();
}
// 저항 4.7k로 계산
// -40 ~ 25도 y = 5.1787x3 - 27.212x2 + 66.6x - 51.419
// 25 ~ 125도 y = 11.124x4 - 150.71x3 + 764.88x2 - 1695.8x + 1406.7
void Gpio_Init()
{
	SN74LVC4245APW_Init();

	OPERATION_L_FAN(OFF, 0);
	OPERATION_R_FAN(OFF, 0);

	PWR_Bias_Init();

	Heavy_Count_Init();

	EX_Load_Cont_Init();
}

void PWR_Bias_Init()
{
	PWR_Module1_Bias_OnOff(ON);
	PWR_Module2_Bias_OnOff(ON);
	PWR_Module3_Bias_OnOff(ON);

}

void PWR_Bias_Reset()
{
	PWR_Module1_Bias_OnOff(OFF);
	PWR_Module2_Bias_OnOff(OFF);
	PWR_Module3_Bias_OnOff(OFF);

	// 잠시 통신 두절
	HAL_Delay(1000);

	PWR_Module1_Bias_OnOff(ON);
	PWR_Module2_Bias_OnOff(ON);
	PWR_Module3_Bias_OnOff(ON);

	gval.gpio_read.Pwr_Pw_Bias = OFF;
}

void Heavy_Count_Init()
{
	Heavy1_Count_ONOFF(OFF);
	Heavy2_Count_ONOFF(OFF);
	Heavy3_Count_ONOFF(OFF);
}

void EX_Load_Cont_Init()
{
	EX1_Load_Cont_ONOFF(ON);
	EX2_Load_Cont_ONOFF(ON);
	EX3_Load_Cont_ONOFF(ON);
}

void NTC_TABLE_CREATER()
{
	uint32_t i = 1U;
	f32 j = 0;

	for(i = 0; i <= NTC_STD_MV_LOW; i++)
	{
		j = (f32)i / 1000.0f;
		NTC_mVolt_Table_DB[i] = (f32)((5.1787F * (j * j * j)) + (-27.212F * (j * j)) + (66.6F * j) - 51.419F);
	}

	for(i = NTC_STD_MV_LOW; i < NTC_mVolt_Table_index ; i++)
	{
		j = (f32)i / 1000.0f;
		NTC_mVolt_Table_DB[i] = (f32)((11.124F * (j * j * j * j)) + (-150.71F * (j * j * j)) + (764.88F * (j * j)) + (-1695.8F * j) + 1406.7F);
	}
}

void Self_Test_3kw_PWR()
{
	uint8_t Ready_time = 0;
	uint8_t b = 0;
	uint8_t c = 0;

	c = INIT_TIME;
	while(c >= Ready_time)
	{
		uint8_t e = 0;
		ADC_Calc();

		e = 1U;
		if(e == gval.flag.one_sec)
		{
			Ready_time++;
			gval.flag.one_sec = 0;
		}

		OPERATION_L_FAN(ON, 50U);
		OPERATION_R_FAN(ON, 50U);

		b = (uint8_t)(gval.test_sec % 2U);
		if(0 == b)
		{
			PANEL_LED_VALUE(AC_G_ON);
		}
		else
		{
			PANEL_LED_VALUE(AC_G_OFF);
		}

		LED_Display();
		Display_LED_Control();

		// 초기에 살릴지 여부 확인인
		ADC_Calc();				// ADC 폴링으로 읽는 과정 (2ms주기)
		CAN_Communication();	// CAN 데이터 송신 (100ms주기)

	} // While loop

	// 초기화 완료 후 AC 점등
	PANEL_LED_VALUE(AC_G_ON);

	Pwr_Self_Test_end();

	// 초기화 이후 한번 리셋동작함.
	gval.flag.HwReset_flag = ON;
}

void Pwr_Self_Test_end()
{
	OPERATION_L_FAN(OFF, 0);
	OPERATION_R_FAN(OFF, 0);

	EX1_Load_Cont_ONOFF(OFF);
	EX2_Load_Cont_ONOFF(OFF);
	EX3_Load_Cont_ONOFF(OFF);

	gval.flag.one_sec = 0;
	gval.flag.test_flag = 0;

	gval.flag.p1_status = 1u;
	gval.flag.p2_status = 1u;
	gval.flag.p3_status = 1u;
	gval.flag.system_error = 0;

	Sec_Time_Count = 0;
}

void PWR_Operation()
{
	uint8_t a = 0;

	gval.test_sec = 0;
	PWR_SW_Read();

	a = 1U;
	if(a == gval.flag.system_error)
	{
		Pwr_error();

		// System 에러 발생시 전면판 스위치로 리셋
		a = 1U;
		if(OFF == gval.gpio_read.Pwr_Sw_OnOff)
		{
			// 모든 에러를 초기화 한다.
			gval.flag.error_Over_Volt = 0;
			gval.flag.error_dc1_volt = 0;
			gval.flag.error_dc2_volt = 0;
			gval.flag.error_dc3_volt = 0;

			gval.flag.error_Total_current = 0;
			gval.flag.error_temp_flag = 0;

			gval.flag.system_error = 0;

			gval.error_counter.over_voltage_counter1 = 0;	
			gval.error_counter.over_voltage_counter2 = 0;
			gval.error_counter.over_voltage_counter3 = 0;

			gval.error_counter.non_over_load_counter = 0;
			gval.error_counter.over_load_counter = 0;

			gval.error_counter.over_temp_Counter = 0;
			gval.error_counter.non_over_temp_Counter = 0;

			gval.timer_flag_gval.Pulse_timer_flag = 0;
			gval.can_cbit.fw_lamp_status.PulseMode = OFF;

		}
	}
	else
	{
		a = 1U;
		if(a == gval.gpio_read.Pwr_Sw_OnOff)
		{
			gval.can_cbit.fw_lamp_status.pow_on_off_lamp = 1U;  // UI SW TX & Operation SW Info
			
			gval.gpio_read.Pwr_Pw_Bias = ON;

			PWR_DC_ON();

			if(OFF == gval.flag.before_pwr_sw_status)
			{
				gval.flag.low_load_counter_on_timer = 1U;
			}
			else
			{
				__NOP();
			}

		}
		else
		{
			uint16_t b = 0;
			a = 1U;
			if(a == gval.flag.before_pwr_sw_status)
			{
				gval.flag.three_min_start_flag = 1U;
				Pwr_Cooling_start();
			}

			b = THREE_MIN;
			if (b <= Three_min_Time_Count)
			{
				gval.flag.three_min_start_flag = 0;
				Pwr_Cooling_end();
			}

			gval.can_cbit.fw_lamp_status.pow_on_off_lamp = 0;
			gval.flag.wait_volt_time_flag = 0;
			PWR_DC_OFF();

			if(gval.gpio_read.Pwr_Pw_Bias == ON)
			{
				gval.flag.HwReset_flag = ON;
				gval.gpio_read.Pwr_Pw_Bias = OFF;
			}

		}
	}
}

void Pwr_error()
{
	OPERATION_L_FAN(OFF, 0);
	OPERATION_R_FAN(OFF, 0);
	PANEL_LED_VALUE(DC_G_OFF);
	PANEL_LED_VALUE(DC_R_ON);

	PWR_1_ON(OFF);
	PWR_2_ON(OFF);
	PWR_3_ON(OFF);

	PANEL_LED_VALUE(P1_G_OFF);
	PANEL_LED_VALUE(P2_G_OFF);
	PANEL_LED_VALUE(P3_G_OFF);
}

void Pwr_Cooling_start()
{
	OPERATION_L_FAN(ON, 100U);
	OPERATION_R_FAN(ON, 100U);

	EX1_Load_Cont_ONOFF(ON);
	EX2_Load_Cont_ONOFF(ON);
	EX3_Load_Cont_ONOFF(ON);
}

void Pwr_Cooling_end()
{
	OPERATION_L_FAN(OFF, 0);
	OPERATION_R_FAN(OFF, 0);
	Three_min_Time_Count = 0;

	EX1_Load_Cont_ONOFF(OFF);
	EX2_Load_Cont_ONOFF(OFF);
	EX3_Load_Cont_ONOFF(OFF);
}

// 전체 pclk2에서 div4하고 sample time을 56cycle로 하면 368us이다
// 56sycle에서 더 빠르게 변경 
void ADC_Calc() 
{
	uint8_t a = 0;
	ADC_ChannelConfTypeDef sConfig;
	uint32_t  adc_ch_change = 0;

	a = 1U;
	if (a == gval.flag.panel_flag)
	{
		// 2ms 주기 - 폴링방식으로 한번에 12CH 정보를 다 읽어오도록함.
		a = 13U;
		for(uint8_t i = 0; i < a; i++ )
		{
			// 채널 변경후 
			adc_ch_change = ADC_channel_switch(i);
			sConfig.Channel = adc_ch_change;
			sConfig.Rank = 1U;
			HAL_ADC_ConfigChannel(&hadc1, &sConfig);
		
			// 폴링 방식으로 읽어오도록 변경경
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, 100);
			gval.adVal_1.ad[i] = (uint16_t)HAL_ADC_GetValue(&hadc1);
			HAL_ADC_Stop(&hadc1);

			// 한번만 동작할것 여기서 시간 많이 소모하면 다른곳에 영향있음.
			if(i==12U)
			{
				// ADC 데이터 필터처리 
				ADC_Conversion_1();
				Filter_ADC_1();
			}
		}
	}
}

const uint32_t ADC_channel_switch(const uint8_t ach)
{
	uint32_t channel = (uint32_t)9999U;

	switch(ach)
	{
		case 0 :
			channel = ADC_CHANNEL_0;
			break;
		case 1U :
			channel = ADC_CHANNEL_1;
			break;
		case 2U :
			channel = ADC_CHANNEL_2;
			break;
		case 3U :
			channel = ADC_CHANNEL_3;
			break;
		case 4U :
			channel = ADC_CHANNEL_4;
			break;
		case 5U :
			channel = ADC_CHANNEL_5;
			break;
		case 6U :
			channel = ADC_CHANNEL_6;
			break;
		case 7U :
			channel = ADC_CHANNEL_7;
			break;
		case 8U :
			channel = ADC_CHANNEL_8;
			break;
		case 9U :
			channel = ADC_CHANNEL_9;
			break;
		case 10U :
			channel = ADC_CHANNEL_10;
			break;
		case 11U :
			channel = ADC_CHANNEL_11;
			break;
		case 12U :
		default :
			channel = ADC_CHANNEL_12;
			break;
	}

	return (uint32_t)channel;
}

void Error_Check()
{
	uint8_t a = 1U;

	// 전원이 켜저있을때 검사한다.
	if(a == gval.gpio_read.Pwr_Sw_OnOff)
	{
		// 100ms 주기로 검사한다.
		if(a == gval.flag.errCheck_flag)
		{
			a = 0;
			if(a == gval.flag.low_load_flag)
			{
				if(a == gval.flag.before_pwr_sw_status)
				{
					gval.flag.wait_volt_time_flag = 1U;
				}
				else
				{
					__NOP();
				}

				a = 5U;
				if(a <= Wait_Volt_Time)
				{
					Check_Volt();
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
			Check_Current();
			Check_Temper();

			gval.flag.errCheck_flag = 0;
		}
	}
	else
	{
		__NOP();
	}
}

void CAN_Communication()
{
	uint8_t a = 0;

	Error_CAN_TX();
	Load_CAN_TX();

	a = 1U;
	if(a == gval.flag.can_flag)
	{
		SEND_TO_UI();
		gval.flag.can_flag = 0;
	}
}

void Display_LED_Control()
{
	memcpy(&gval.led_status, &gval.can_cbit.fw_lamp_status, sizeof(S_FW_LAMP_STATUE));

	uint8_t a = 1U;
	if(a == gval.flag.panel_flag) // 최대 6us ,
	{
		uint8_t b = 2U;
		uint8_t c = 3U;
		if(0 == gval.control_data.Led_Control_Seq) // 1.FC Data Write enable
		{
			gval.flag.tlc59482_tx_flag = 1U;
			gval.flag.fcwrten_flag = 1U;
			gval.control_data.tlc59482_tx_data = init_val_32;
			gval.control_data.Led_Control_Seq ++;
		}
		else if(a == gval.control_data.Led_Control_Seq) // 2.FC Data Write
		{
			gval.flag.tlc59482_tx_flag = 1U;
			gval.flag.wrtfc_flag = 1U;
			gval.control_data.tlc59482_tx_data = init_val_32;
			gval.control_data.Led_Control_Seq ++;
		}
		else if(b == gval.control_data.Led_Control_Seq) // 3.GS 1ST LATCH DATA Write
		{
			gval.flag.tlc59482_tx_flag = 1U;
			gval.flag.wrtgs_flag = 1U;
		}
		else if(c == gval.control_data.Led_Control_Seq) // 4.GS 1ST LATCH DATA Copy to 2,3 LATCH DATA
		{
			LED_Latch_OnOff(LOW);
			gval.flag.tlc59482_tx_flag = 1U;
			gval.flag.latgs_flag = 1U;
			gval.control_data.tlc59482_tx_data = 0xFFFFFFFFU;
			gval.control_data.Led_Control_Seq ++;
		}
		else
		{
			gval.control_data.Led_Control_Seq = 0;
		}
		gval.flag.panel_flag = 0;
	}

	memcpy(&gval.before_led_status, &gval.led_status, sizeof(S_FW_LAMP_STATUE));
}

void OPERATION_L_FAN(uint8_t mode, uint16_t Temp)
{
	uint8_t Fan_onoff = 0;
	uint16_t Fan_Ratio = 0;
	uint8_t a = 1U;

	Fan_onoff = mode;
	Fan_Ratio = Temp;

	Fan_L_status();

	if((OFF == gval.gpio_read.Fan_L_status) && (ON == gval.flag.fan_l_status_before))
	{
		gval.fan_l_freqency++;
	}

	if(a <= fan_l)
	{
		Fan_error_check();
	}

	if(OFF == gval.flag.test_flag)
	{
		Fan_error_check_1(Fan_onoff);
	}
	else
	{
		__NOP();
	}

	FAN_L_OnOff(Fan_onoff);

	if(a == gval.fan_test.fan_test_mode )  // CAN 통신... 명령..
	{
		gval.fan_test.fan_left_pwm = (uint16_t)(gval.fan_test.fan_test_left_pwm * 11.2f);
		gval.fan_test.fan_right_pwm = (uint16_t)(gval.fan_test.fan_test_right_pwm * 11.2f);
	}
	else
	{
		Fan_speed(Fan_Ratio);
	}

	PWM_FAN_L_Period((uint32_t)gval.fan_test.fan_left_pwm);
}

void Fan_speed(uint16_t Fan_Ratio)
{
	if(0 >= Fan_Ratio)
	{
		gval.fan_test.fan_left_pwm = 0; // 0퍼센트
	}
	else if( (0 < Fan_Ratio) && (30U > Fan_Ratio) )
	{
		gval.fan_test.fan_left_pwm = 672U; // 60퍼센트
	}
	else if((30U <= Fan_Ratio) && (40U > Fan_Ratio))
	{
		gval.fan_test.fan_left_pwm = 784U; // 70퍼센트
	}
	else if((40U <= Fan_Ratio) && (50U > Fan_Ratio))
	{
		gval.fan_test.fan_left_pwm = 894U; // 80퍼센트
	}
	else if((50U <= Fan_Ratio) && (60U > Fan_Ratio))
	{
		gval.fan_test.fan_left_pwm = 1008U; // 90퍼센트
	}
	else
	{
		gval.fan_test.fan_left_pwm = 1100U; // 99퍼센트
	}
}

void Fan_error_check()
{
	uint16_t b = 100U;

	if(b <= gval.fan_l_freqency)
	{
		gval.flag.fan_l_error = 1U;
		gval.fan_l_freqency = 100U;
	}
	else
	{
		gval.flag.fan_l_error = 0;
	}
	fan_l = 0;
}

void Fan_error_check_1(uint8_t Fan_onoff)
{
	uint8_t a = 1U;

	if (0 == gval.flag.fan_l_error)  // FAN Error
	{
		if(a == Fan_onoff)
		{
			PANEL_LED_VALUE(FAN_L_R_ON);
			PANEL_LED_VALUE(FAN_L_G_OFF);
		}
		else
		{
			PANEL_LED_VALUE(FAN_L_R_OFF);
			PANEL_LED_VALUE(FAN_L_G_OFF);
		}
	}
	else   // FAN Normal
	{
		if(a == Fan_onoff)
		{
			PANEL_LED_VALUE(FAN_L_G_ON);
			PANEL_LED_VALUE(FAN_L_R_OFF);
		}
		else
		{
			PANEL_LED_VALUE(FAN_L_R_OFF);
			PANEL_LED_VALUE(FAN_L_G_OFF);
		}
	}
}

void OPERATION_R_FAN(uint8_t mode, uint16_t Temp) // pwm은 pos duty 퍼센트이다
{
	uint8_t Fan_onoff = 0;
	uint16_t Fan_Ratio = 0;
	uint8_t a = 1U;

	Fan_onoff = mode;
	Fan_Ratio = Temp;

	Fan_R_status();

	if((OFF == gval.gpio_read.Fan_R_status) && (ON == gval.flag.fan_r_status_before))
	{
		gval.fan_r_freqency++;
	}

	if(a <= fan_r)
	{
		uint16_t b = 100U;

		if(b <= gval.fan_r_freqency)
		{
			gval.flag.fan_r_error = 1U;
			gval.fan_r_freqency = 100U;
		}
		else
		{
			gval.flag.fan_r_error = 0;
		}
		fan_r = 0;
	}

	if(OFF == gval.flag.test_flag)
	{
		if(0 == gval.flag.fan_r_error) // FAN Error
		{
			if(a == Fan_onoff)
			{
				PANEL_LED_VALUE(FAN_R_R_ON);
				PANEL_LED_VALUE(FAN_R_G_OFF);
			}
			else
			{
				PANEL_LED_VALUE(FAN_R_R_OFF);
				PANEL_LED_VALUE(FAN_R_G_OFF);
			}
		}
		else   // FAN Normal
		{
			if(a == Fan_onoff)
			{
				PANEL_LED_VALUE(FAN_R_R_OFF);
				PANEL_LED_VALUE(FAN_R_G_ON);
			}
			else
			{
				PANEL_LED_VALUE(FAN_R_R_OFF);
				PANEL_LED_VALUE(FAN_R_G_OFF);
			}
		}
	}
	else
	{
		__NOP();
	}

	FAN_R_OnOff(Fan_onoff);

	if(0 >= Fan_Ratio)
	{
		gval.fan_test.fan_right_pwm = 0; // 0퍼센트
	}
	else if( (0 <= Fan_Ratio) && (30U > Fan_Ratio) )
	{
		gval.fan_test.fan_right_pwm = 672U; // 60퍼센트
	}
	else if((30U <= Fan_Ratio) && (40U > Fan_Ratio))
	{
		gval.fan_test.fan_right_pwm = 784U; // 70퍼센트
	}
	else if((40U <= Fan_Ratio) && (50U > Fan_Ratio))
	{
		gval.fan_test.fan_right_pwm = 894U; // 80퍼센트
	}
	else if((50U <= Fan_Ratio) && (60U > Fan_Ratio))
	{
		gval.fan_test.fan_right_pwm = 1008U; // 90퍼센트
	}
	else
	{
		gval.fan_test.fan_right_pwm = 1100U; // 99퍼센트
	}

	PWM_FAN_R_Period((uint32_t)gval.fan_test.fan_right_pwm);
}


// 25khz, 0~999
void PWM_FAN_L_Period(uint32_t pwm)
{
	uint32_t pwm_left = pwm;

	if(MAX_PWM_DUTY < pwm_left)
	{
		pwm_left = MAX_PWM_DUTY;
	}
	else if(MIN_PWM_DUTY > pwm_left)
	{
		pwm_left = MIN_PWM_DUTY;
	}
	else
	{
		__NOP();
	}

	TIM1->CCR3 = pwm_left;
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
}

// 25khz, 0~999
void PWM_FAN_R_Period(uint32_t pwm)
{
	uint32_t pwm_right = pwm;

	if(MAX_PWM_DUTY < pwm_right)
	{
		pwm_right = MAX_PWM_DUTY;
	}
	else if(MIN_PWM_DUTY > pwm_right)
	{
		pwm_right = MIN_PWM_DUTY;
	}
	else
	{
		__NOP();
	}

	TIM1->CCR4 = pwm_right;
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

void SN74LVC4245APW_Init()
{
	Sig_Out_En();
	EER_Chk_En();
	Sig_In_En();
	SPI_EN();

	Load_Control_ONOFF(0);
}

void Check_Volt()
{
	uint8_t a = 0;

	a = 1U;
	if(a == gval.flag.CAN_Over_chk_test)
	{
		if(a == gval.flag.CAN_Overdcvoltage_test)
		{
			PWR_1_ON(OFF);
			PWR_2_ON(OFF);
			PWR_3_ON(OFF);
			PANEL_LED_VALUE(DC_G_OFF);
			PANEL_LED_VALUE(DC_R_ON);
			PANEL_LED_VALUE(P1_G_OFF);
			PANEL_LED_VALUE(P2_G_OFF);
			PANEL_LED_VALUE(P3_G_OFF);
			
			gval.flag.error_Over_Volt = 1U;
			gval.flag.system_error = 1U;
			
		}
		else
		{
			__NOP();
			
		}
	}
	else
	{
		if(OFF == gval.flag.CAN_PWR_err_test)  // Normal status Check
		{
			Input_AC_CHECK(gval.adVal_1.real_1.PWR_AC);

			uint16_t d = (uint16_t)OUT_DCV_H_LIMIT;
			uint16_t b = 0;
			uint16_t c = (uint16_t)gval.low_voltage;

			b = (uint16_t)gval.adVal_1.real_1.PWR_1_28V;
			if( (d >= b) && (c <= b))
			{
				// 정상 범위 전압
				if(a != gval.flag.error_dc1_volt)
				{
					PANEL_LED_VALUE(P1_G_ON);
					PANEL_LED_VALUE(P1_R_OFF);
					gval.flag.p1_status = 1U;
					gval.flag.error_dc1_volt = 0;
				}
				else
				{
					PANEL_LED_VALUE(P1_G_OFF);
					PANEL_LED_VALUE(P1_R_ON);
					gval.flag.p1_status = 0;
					gval.flag.error_dc1_volt = 1U;
				}

				if(gval.error_counter.over_voltage_counter1>0)gval.error_counter.over_voltage_counter1--;
			}
			else
			{
				// 비정상 범위 전압
				gval.error_counter.over_voltage_counter1++;
				if(gval.error_counter.over_voltage_counter1 > ERR_SET_COUNT)
				{
					PANEL_LED_VALUE(P1_G_OFF);
					PANEL_LED_VALUE(P1_R_ON);
					gval.flag.p1_status = 0;
					gval.flag.error_dc1_volt = 1U;
					PWR_1_ON(OFF);

					gval.error_counter.over_voltage_counter1 = 0;
				}			
			}

			b = (uint16_t)gval.adVal_1.real_1.PWR_2_28V;
			if( (d >= b) && (c <= b))
			{
				// 정상 범위 전압
				if(a != gval.flag.error_dc2_volt)
				{
					PANEL_LED_VALUE(P2_G_ON);
					PANEL_LED_VALUE(P2_R_OFF);
					gval.flag.p2_status = 1U;
					gval.flag.error_dc2_volt = 0;
				}
				else
				{
					PANEL_LED_VALUE(P2_G_OFF);
					PANEL_LED_VALUE(P2_R_ON);
					gval.flag.p2_status = 0;
					gval.flag.error_dc2_volt = 1U;
				}

				if(gval.error_counter.over_voltage_counter2>0)gval.error_counter.over_voltage_counter2--;
			}
			else
			{
				// 비정상 범위 전압
				gval.error_counter.over_voltage_counter2++;
				if(gval.error_counter.over_voltage_counter2 > ERR_SET_COUNT)
				{
					PANEL_LED_VALUE(P2_G_OFF);
					PANEL_LED_VALUE(P2_R_ON);
					gval.flag.p2_status = 0;
					gval.flag.error_dc2_volt = 1U;
					PWR_2_ON(OFF);

					gval.error_counter.over_voltage_counter2 = 0;
				}
			}

			b = (uint16_t)gval.adVal_1.real_1.PWR_3_28V;
			if( (d >= b) && (c <= b))
			{
				// 정상 범위 전압
				if(a != gval.flag.error_dc3_volt)
				{
					PANEL_LED_VALUE(P3_G_ON);
					PANEL_LED_VALUE(P3_R_OFF);
					gval.flag.p3_status = 1U;
					gval.flag.error_dc3_volt = 0;
				}
				else
				{
					PANEL_LED_VALUE(P3_G_OFF);
					PANEL_LED_VALUE(P3_R_ON);
					gval.flag.p3_status = 0;
					gval.flag.error_dc3_volt = 1U;
				}

				if(gval.error_counter.over_voltage_counter3>0)gval.error_counter.over_voltage_counter3--;
			}
			else
			{
				// 비비정상 범위 전압
				gval.error_counter.over_voltage_counter3++;
				if(gval.error_counter.over_voltage_counter3 > ERR_SET_COUNT)
				{
					PANEL_LED_VALUE(P3_G_OFF);
					PANEL_LED_VALUE(P3_R_ON);
					gval.flag.p3_status = 0;
					gval.flag.error_dc3_volt = 1U;
					PWR_3_ON(OFF);

					gval.error_counter.over_voltage_counter3 = 0;
				}
			}

			if( (ON == gval.flag.error_dc1_volt) && (ON == gval.flag.error_dc2_volt) && (ON == gval.flag.error_dc3_volt) )
			{
				gval.flag.error_Over_Volt = 1U;
				PANEL_LED_VALUE(DC_G_OFF);
				PANEL_LED_VALUE(DC_R_ON);
				gval.flag.system_error = 1U;
				PANEL_LED_VALUE(P1_G_OFF);
				PANEL_LED_VALUE(P1_R_ON);
				PANEL_LED_VALUE(P2_G_OFF);
				PANEL_LED_VALUE(P2_R_ON);
				PANEL_LED_VALUE(P3_G_OFF);
				PANEL_LED_VALUE(P3_R_ON);
				PWR_1_ON(OFF);
				PWR_2_ON(OFF);
				PWR_3_ON(OFF);
			}
			else
			{
				PANEL_LED_VALUE(DC_G_ON);
				PANEL_LED_VALUE(DC_R_OFF);
			}
		}
		else
		{
			if(a == gval.flag.CAN_pwr_1_err_test)
			{
				PANEL_LED_VALUE(P1_G_OFF);
				PANEL_LED_VALUE(P1_R_ON);
				gval.flag.p1_status = 0;
				gval.flag.error_dc1_volt = 1U;
				PWR_1_ON(OFF);
			}
			else
			{
				PANEL_LED_VALUE(P1_G_ON);
				PANEL_LED_VALUE(P1_R_OFF);
				gval.flag.p1_status = 1U;
				gval.flag.error_dc1_volt = 0;
			}

			if(a == gval.flag.CAN_pwr_2_err_test)
			{
				PANEL_LED_VALUE(P2_G_OFF);
				PANEL_LED_VALUE(P2_R_ON);
				gval.flag.p2_status = 0;
				gval.flag.error_dc2_volt = 1U;
				PWR_2_ON(OFF);
			}
			else
			{
				PANEL_LED_VALUE(P2_G_ON);
				PANEL_LED_VALUE(P2_R_OFF);
				gval.flag.p2_status = 1U;
				gval.flag.error_dc2_volt = 0;
			}

			if(a == gval.flag.CAN_pwr_3_err_test)
			{
				PANEL_LED_VALUE(P3_G_OFF);
				PANEL_LED_VALUE(P3_R_ON);
				gval.flag.p3_status = 0;
				gval.flag.error_dc3_volt = 1U;
				PWR_3_ON(OFF);
			}
			else
			{
				PANEL_LED_VALUE(P3_G_ON);
				PANEL_LED_VALUE(P3_R_OFF);
				gval.flag.p3_status = 1U;
				gval.flag.error_dc3_volt = 0;
			}

			if( (a == gval.flag.error_dc1_volt) && (a == gval.flag.error_dc2_volt) && (a == gval.flag.error_dc3_volt) )
			{
				PANEL_LED_VALUE(DC_G_OFF);
				PANEL_LED_VALUE(DC_R_ON);
				gval.flag.error_Over_Volt = 1U;
				gval.flag.system_error = 1U;
				PWR_1_ON(OFF);
				PWR_2_ON(OFF);
				PWR_3_ON(OFF);
			}
			else
			{
				gval.flag.error_Over_Volt = 0;
				PANEL_LED_VALUE(DC_G_ON);
				PANEL_LED_VALUE(DC_R_OFF);
			}
		}
	}
}

void Check_Current()
{
	uint8_t b = 0;

	gval.Total_DCA = gval.adVal_1.real_1.PWR_1_40A;		// PEAK치로 변경 
	gval.can_cbit.fw_pow_temp.Total_Current = gval.Total_DCA;

	b = 1U;
	if(b == gval.flag.low_load_flag)
	{
		gval.Average_DCV = ((gval.adVal_1.real_1.PWR_1_28V + gval.adVal_1.real_1.PWR_2_28V + gval.adVal_1.real_1.PWR_3_28V) / 3);
	}
	else
	{
		gval.DC_POWER = gval.Total_DCA * gval.Average_DCV;
	}

	gval.can_cbit.fw_pow_temp.power = gval.DC_POWER;

	if(OFF == gval.flag.CAN_Over_chk_test)
	{
		uint16_t a = 0;
		uint16_t c = 0;

		uint16_t d = TOTAL_ZERO_LOAD;
		uint16_t i = TOTAL_OVERLOAD;

		a = (uint16_t)gval.Total_DCA;
		if(d >= a) // 무부하 및 경부하
		{
			gval.flag.pre_overcurrent_flag = 0;

			gval.flag.low_load_counter_on_timer = 1U;
			PANEL_LED_VALUE(LOAD_25_G_OFF);
			PANEL_LED_VALUE(LOAD_50_G_OFF);
			PANEL_LED_VALUE(LOAD_75_G_OFF);
			PANEL_LED_VALUE(LOAD_100_G_OFF);
			PANEL_LED_VALUE(LOAD_25_R_OFF);
			PANEL_LED_VALUE(LOAD_50_R_OFF);
			PANEL_LED_VALUE(LOAD_75_R_OFF);
			PANEL_LED_VALUE(LOAD_100_R_OFF);
			gval.error_counter.non_over_load_counter++; // 노이즈로 인한 추가
		}
		else if((d < a) && (i >= a))
		{
			Non_over_load_led_state();
		}
		else if(i < a)
		{
			PANEL_LED_VALUE(LOAD_25_G_OFF);
			PANEL_LED_VALUE(LOAD_50_G_OFF);
			PANEL_LED_VALUE(LOAD_75_G_OFF);
			PANEL_LED_VALUE(LOAD_100_G_OFF);
			PANEL_LED_VALUE(LOAD_25_R_ON);
			PANEL_LED_VALUE(LOAD_50_R_ON);
			PANEL_LED_VALUE(LOAD_75_R_ON);
			PANEL_LED_VALUE(LOAD_100_R_ON);
			PANEL_LED_VALUE(DC_G_OFF);
			PANEL_LED_VALUE(DC_R_ON); // 추후 애러까지 살릴것

			gval.error_counter.over_load_counter++; // 노이즈로 인한 추가
		}

		if((ERR_SET_COUNT <= gval.error_counter.over_load_counter)||(ON == gval.flag.error_Total_current)) 
		{
			PANEL_LED_VALUE(LOAD_25_G_OFF);
			PANEL_LED_VALUE(LOAD_50_G_OFF);
			PANEL_LED_VALUE(LOAD_75_G_OFF);
			PANEL_LED_VALUE(LOAD_100_G_OFF);
			PANEL_LED_VALUE(LOAD_25_R_ON);
			PANEL_LED_VALUE(LOAD_50_R_ON);
			PANEL_LED_VALUE(LOAD_75_R_ON);
			PANEL_LED_VALUE(LOAD_100_R_ON);
			PANEL_LED_VALUE(DC_G_OFF);
			PANEL_LED_VALUE(DC_R_ON); // 추후 애러까지 살릴것

			gval.flag.error_Total_current = 1U;
			gval.flag.system_error = 1U;

			gval.error_counter.non_over_load_counter = 0;
			gval.error_counter.over_load_counter = 0;
		}

		c = ERR_SET_COUNT;
		if(c <= gval.error_counter.non_over_load_counter)
		{
			gval.error_counter.non_over_load_counter = 0;
			gval.error_counter.over_load_counter = 0;
		}

	}
	else
	{
		Can_Load_Test();
	}
}

void Load_led_toggle()
{
	uint16_t j = 1U;

	gval.flag.pre_overcurrent_flag = 1U;
	if(j == gval.pre_overcurrent_time)
	{
		PANEL_LED_VALUE(LOAD_25_G_ON);
		PANEL_LED_VALUE(LOAD_50_G_ON);
		PANEL_LED_VALUE(LOAD_75_G_ON);
		PANEL_LED_VALUE(LOAD_100_G_ON);
	}
	else if(0 == gval.pre_overcurrent_time)
	{
		PANEL_LED_VALUE(LOAD_25_G_OFF);
		PANEL_LED_VALUE(LOAD_50_G_OFF);
		PANEL_LED_VALUE(LOAD_75_G_OFF);
		PANEL_LED_VALUE(LOAD_100_G_OFF);
	}
	else
	{
		gval.pre_overcurrent_time = 0;
	}
	// 110프로 이하일때 추가
	PANEL_LED_VALUE(LOAD_25_R_OFF);
	PANEL_LED_VALUE(LOAD_50_R_OFF);
	PANEL_LED_VALUE(LOAD_75_R_OFF);
	PANEL_LED_VALUE(LOAD_100_R_OFF);
	gval.error_counter.non_over_load_counter++; // 노이즈로 인한 추가
}

void Non_over_load_led_state()
{
	uint16_t a = 0;
	uint16_t d = TOTAL_ZERO_LOAD;
	uint16_t e = TOTAL_QUAD_LOAD;
	uint16_t f = TOTAL_HALF_LOAD;
	uint16_t g = TOTAL_THREE_QUAD_LOAD;
	uint16_t h = TOTAL_FULL_LOAD;
	uint16_t i = TOTAL_OVERLOAD;

	a = (uint16_t)gval.Total_DCA;

	if( (d < a) && (e >= a) ) // 25% 이하
	{
		gval.flag.pre_overcurrent_flag = 0;

		PANEL_LED_VALUE(LOAD_25_G_ON);
		PANEL_LED_VALUE(LOAD_50_G_OFF);
		PANEL_LED_VALUE(LOAD_75_G_OFF);
		PANEL_LED_VALUE(LOAD_100_G_OFF);
		PANEL_LED_VALUE(LOAD_25_R_OFF);
		PANEL_LED_VALUE(LOAD_50_R_OFF);
		PANEL_LED_VALUE(LOAD_75_R_OFF);
		PANEL_LED_VALUE(LOAD_100_R_OFF);
		gval.error_counter.non_over_load_counter++; // 노이즈로 인한 추가
	}
	else if( (e < a) && (f >= a) ) // 50% 이하
	{
		gval.flag.pre_overcurrent_flag = 0;

		PANEL_LED_VALUE(LOAD_25_G_ON);
		PANEL_LED_VALUE(LOAD_50_G_ON);
		PANEL_LED_VALUE(LOAD_75_G_OFF);
		PANEL_LED_VALUE(LOAD_100_G_OFF);
		PANEL_LED_VALUE(LOAD_25_R_OFF);
		PANEL_LED_VALUE(LOAD_50_R_OFF);
		PANEL_LED_VALUE(LOAD_75_R_OFF);
		PANEL_LED_VALUE(LOAD_100_R_OFF);
		gval.error_counter.non_over_load_counter++; // 노이즈로 인한 추가
	}
	else if( (f < a) && (g >= a) ) // 75% 이하
	{
		gval.flag.pre_overcurrent_flag = 0;

		PANEL_LED_VALUE(LOAD_25_G_ON);
		PANEL_LED_VALUE(LOAD_50_G_ON);
		PANEL_LED_VALUE(LOAD_75_G_ON);
		PANEL_LED_VALUE(LOAD_100_G_OFF);
		PANEL_LED_VALUE(LOAD_25_R_OFF);
		PANEL_LED_VALUE(LOAD_50_R_OFF);
		PANEL_LED_VALUE(LOAD_75_R_OFF);
		PANEL_LED_VALUE(LOAD_100_R_OFF);
		gval.error_counter.non_over_load_counter++; // 노이즈로 인한 추가
	}
	else if( (g < a) && (h >= a) ) // 100% 이하
	{
		gval.flag.pre_overcurrent_flag = 0;

		PANEL_LED_VALUE(LOAD_25_G_ON);
		PANEL_LED_VALUE(LOAD_50_G_ON);
		PANEL_LED_VALUE(LOAD_75_G_ON);
		PANEL_LED_VALUE(LOAD_100_G_ON);
		PANEL_LED_VALUE(LOAD_25_R_OFF);
		PANEL_LED_VALUE(LOAD_50_R_OFF);
		PANEL_LED_VALUE(LOAD_75_R_OFF);
		PANEL_LED_VALUE(LOAD_100_R_OFF);
		gval.error_counter.non_over_load_counter++; // 노이즈로 인한 추가
	}
	else if( (h < a) && (i >= a) ) // 110% 이하
	{
		Load_led_toggle();
	}
}

void Can_Load_Test()
{
	uint8_t b = 0;

	if(b == gval.flag.CAN_OverLoad_test)
	{
		
		PANEL_LED_VALUE(LOAD_25_G_OFF);
		PANEL_LED_VALUE(LOAD_50_G_OFF);
		PANEL_LED_VALUE(LOAD_75_G_OFF);
		PANEL_LED_VALUE(LOAD_100_G_OFF);
		PANEL_LED_VALUE(LOAD_25_R_OFF);
		PANEL_LED_VALUE(LOAD_50_R_OFF);
		PANEL_LED_VALUE(LOAD_75_R_OFF);
		PANEL_LED_VALUE(LOAD_100_R_OFF);
		
	}
	else
	{
		gval.flag.error_Total_current = 1U;
		PWR_1_ON(OFF);
		PWR_2_ON(OFF);
		PWR_3_ON(OFF);
		gval.flag.system_error = 1U;

		PANEL_LED_VALUE(DC_G_OFF);
		PANEL_LED_VALUE(DC_R_ON);

		PANEL_LED_VALUE(P1_G_OFF);
		PANEL_LED_VALUE(P2_G_OFF);
		PANEL_LED_VALUE(P3_G_OFF);

		PANEL_LED_VALUE(LOAD_25_G_OFF);
		PANEL_LED_VALUE(LOAD_50_G_OFF);
		PANEL_LED_VALUE(LOAD_75_G_OFF);
		PANEL_LED_VALUE(LOAD_100_G_OFF);
		PANEL_LED_VALUE(LOAD_25_R_ON);
		PANEL_LED_VALUE(LOAD_50_R_ON);
		PANEL_LED_VALUE(LOAD_75_R_ON);
		PANEL_LED_VALUE(LOAD_100_R_ON);
	}
}

void Check_Temper()
{
	f32 pwr1_temp = 0.0f;
	f32 pwr2_temp = 0.0f;
	f32 pwr3_temp = 0.0f;
	f32 high_temp = 0.0f;

	int16_t b = 0;

	pwr1_temp = Peak_Search(0, 0, gval.adVal_1.real_1.PWR_1_TEMP3, gval.adVal_1.real_1.PWR_1_TEMP4);
	pwr2_temp = Peak_Search(0, 0, gval.adVal_1.real_1.PWR_2_TEMP3, gval.adVal_1.real_1.PWR_2_TEMP4);
	pwr3_temp = Peak_Search(0, 0, gval.adVal_1.real_1.PWR_3_TEMP3, gval.adVal_1.real_1.PWR_3_TEMP4);

	high_temp = Peak_Search(pwr1_temp, pwr2_temp, pwr3_temp, 0);

	b = (int16_t)high_temp;

	if(0 >= b)
	{
		gval.low_voltage = 5.0f;
	}
	else
	{
		gval.low_voltage = 18.0f;	
	}

	if(OFF == gval.flag.CAN_Over_chk_test)
	{
		uint16_t a = 0;
		int16_t d = TEMPERATURE_NORMAL;
		int16_t e = TEMPERATURE_HOT;
		int16_t f = TEMPERATURE_LIMIT;

		if( (d < b) && (e > b))
		{
			temp_fan(b);
			gval.error_counter.non_over_temp_Counter++;
		}
		else if( (e <= b) && (f >= b))
		{
			temp_fan(b);
			gval.error_counter.non_over_temp_Counter++;
		}
		else if(f < b)
		{
			temp_fan(b);
			gval.error_counter.over_temp_Counter++;
		}
		else
		{
			PANEL_LED_VALUE(TEMP_G_ON);
			PANEL_LED_VALUE(TEMP_E_R_OFF);
			OPERATION_L_FAN(OFF, 0);
			OPERATION_R_FAN(OFF, 0);

			gval.error_counter.non_over_temp_Counter++;
		}

		a = ERR_SET_COUNT;
		if((a <= gval.error_counter.over_temp_Counter) || (ON == gval.flag.error_temp_flag)) // (500 <= gval.error_counter.over_temp_Counter)
		{
			PANEL_LED_VALUE(TEMP_G_OFF);
			PANEL_LED_VALUE(TEMP_E_R_ON);

			gval.flag.error_temp_flag = 1U;
			gval.flag.system_error = 1U;

			gval.error_counter.over_temp_Counter = 0;
			gval.error_counter.non_over_temp_Counter = 0;
		}
		else
		{
			__NOP();
		}

		if(a <= gval.error_counter.non_over_temp_Counter)
		{
			gval.error_counter.over_temp_Counter = 0;
			gval.error_counter.non_over_temp_Counter = 0;
		}
		else
		{
			__NOP();
		}

	}
	else
	{
		uint8_t c = 1U;

		if(c == gval.flag.CAN_Overtemp_test)
		{
			gval.flag.error_temp_flag = 1U;
			gval.flag.system_error = 1U;
			
			PWR_1_ON(OFF);
			PWR_2_ON(OFF);
			PWR_3_ON(OFF);

			PANEL_LED_VALUE(P1_G_OFF);
			PANEL_LED_VALUE(P2_G_OFF);
			PANEL_LED_VALUE(P3_G_OFF);

			PANEL_LED_VALUE(TEMP_G_OFF);
			PANEL_LED_VALUE(TEMP_E_R_ON);
			
		}
	}
}

void temp_fan(uint16_t pwm)
{
	uint8_t a = 1U;

	if(a == gval.flag.system_error)
	{
		PANEL_LED_VALUE(TEMP_G_OFF);
		PANEL_LED_VALUE(TEMP_E_R_ON);

		OPERATION_L_FAN(OFF, 0);
		OPERATION_R_FAN(OFF, 0);
	}
	else
	{
		PANEL_LED_VALUE(TEMP_G_ON);
		PANEL_LED_VALUE(TEMP_E_R_OFF);

		OPERATION_L_FAN(ON, pwm);
		OPERATION_R_FAN(ON, pwm);
	}
}

f32 Peak_Search(f32 temp_1, f32 temp_2, f32 temp_3, f32 temp_4)
{
	f32 high_temp = 0;
	f32 mid_temp_1 = 0;
	f32 mid_temp_2 = 0;

	if(0.0f < (temp_1 - temp_2))
	{
		mid_temp_1 = temp_1;
	}
	else
	{
		mid_temp_1 = temp_2;
	}

	if(0.0f < (temp_3 - temp_4))
	{
		mid_temp_2 = temp_3;
	}
	else
	{
		mid_temp_2 = temp_4;
	}

	if(0.0f < (mid_temp_1 - mid_temp_2))
	{
		high_temp = mid_temp_1;
	}
	else
	{
		high_temp = mid_temp_2;
	}

	return high_temp;

}

void Check_Board()
{
	uint8_t Err_flag = 0;
	uint8_t a = 1U;

	Err_flag = Power_Error_P1();
	if(a == Err_flag)
	{
		PWR_1_ON(OFF);
		gval.flag.p1_status = 0;
	}

	Err_flag = Power_Error_P2();
	if(a == Err_flag)
	{
		PWR_2_ON(OFF);
		gval.flag.p2_status = 0;
	}

	Err_flag = Power_Error_P3();
	if(a == Err_flag)
	{
		PWR_3_ON(OFF);
		gval.flag.p3_status = 0;
	}

}
void Error_CAN_TX()
{
	uint8_t a = 1U;

	if((a == gval.flag.error_Over_Volt) || (a == gval.flag.error_Total_current) || (a == gval.flag.error_temp_flag))
	{
		if(a == gval.flag.error_Over_Volt)
		{
			gval.can_cbit.fw_lamp_status.over_volt_alert_lamp = 1U;
		}
		else
		{
			gval.can_cbit.fw_lamp_status.over_volt_alert_lamp = 0;
		}

		if(a == gval.flag.error_Total_current)
		{
			gval.can_cbit.fw_lamp_status.over_load_alert_lamp = 1U;
		}
		else
		{
			gval.can_cbit.fw_lamp_status.over_load_alert_lamp = 0;
		}

		if(a == gval.flag.error_temp_flag)
		{
			gval.can_cbit.fw_lamp_status.over_temp_alert_lamp = 1U;
		}
		else
		{
			gval.can_cbit.fw_lamp_status.over_temp_alert_lamp = 0;
		}
		gval.flag.system_error = a;
	}
	else
	{
		if(a == gval.flag.error_Over_Volt)
		{
			gval.can_cbit.fw_lamp_status.over_volt_alert_lamp = 1U;
		}
		else
		{
			gval.can_cbit.fw_lamp_status.over_volt_alert_lamp = 0;
		}

		if(a == gval.flag.error_Total_current)
		{
			gval.can_cbit.fw_lamp_status.over_load_alert_lamp = 1U;
		}
		else
		{
			gval.can_cbit.fw_lamp_status.over_load_alert_lamp = 0;
		}

		if(a == gval.flag.error_temp_flag)
		{
			gval.can_cbit.fw_lamp_status.over_temp_alert_lamp = 1U;
		}
		else
		{
			gval.can_cbit.fw_lamp_status.over_temp_alert_lamp = 0;
		}
	}
}


void Load_CAN_TX()
{
	f32 Watt = 0.0f;
	f32 Load_percent = 0.0f;

	Watt =  gval.Average_DCV * gval.Total_DCA;

	Load_percent = (Watt / 3000.0f) * 100.0f;

	gval.can_cbit.fw_load.load = Load_percent;
	gval.can_cbit.fw_pow_temp.power = Watt;
}

void PWR_DC_ON()
{
	if(OFF == gval.flag.CAN_manual_on_off_test) // 정상모드에서 전력변환보드 제어 ON 한다.
	{
		///
		PWR_1_ON(gval.flag.p1_status);
		PWR_2_ON(gval.flag.p2_status);
		PWR_3_ON(gval.flag.p3_status);
	}
	else  // 시험모드 제어
	{
		PWR_1_ON(gval.flag.pwr1_on_off);
		PWR_2_ON(gval.flag.pwr2_on_off);
		PWR_3_ON(gval.flag.pwr3_on_off);
	}

	gval.flag.HwReset_ResetComplete_flag = OFF;
}

void PWR_DC_OFF()
{
	if(OFF == gval.gpio_read.Pwr_Sw_OnOff)
	{
		PWR_1_ON(OFF);
		PWR_2_ON(OFF);
		PWR_3_ON(OFF);

		// POWER OFF시 전압제어 IC 전원(12V)를 리셋함.
		if(gval.flag.HwReset_ResetComplete_flag == OFF) gval.flag.HwReset_flag = ON;
	}
	else
	{
		__NOP();
	}

	gval.Total_DCA = gval.adVal_1.real_1.PWR_1_40A + gval.adVal_1.real_1.PWR_2_40A + gval.adVal_1.real_1.PWR_3_40A;
	gval.can_cbit.fw_pow_temp.Total_Current = gval.Total_DCA;

	PANEL_LED_VALUE(DC_G_OFF);
	PANEL_LED_VALUE(DC_R_OFF);

	PANEL_LED_VALUE(P1_G_OFF);
	PANEL_LED_VALUE(P1_R_OFF);
	PANEL_LED_VALUE(P2_G_OFF);
	PANEL_LED_VALUE(P2_R_OFF);
	PANEL_LED_VALUE(P3_G_OFF);
	PANEL_LED_VALUE(P3_R_OFF);

	PANEL_LED_VALUE(LOAD_25_G_OFF);
	PANEL_LED_VALUE(LOAD_25_R_OFF);
	PANEL_LED_VALUE(LOAD_50_G_OFF);
	PANEL_LED_VALUE(LOAD_50_R_OFF);
	PANEL_LED_VALUE(LOAD_75_G_OFF);
	PANEL_LED_VALUE(LOAD_75_R_OFF);
	PANEL_LED_VALUE(LOAD_100_G_OFF);
	PANEL_LED_VALUE(LOAD_100_R_OFF);

	PANEL_LED_VALUE(TEMP_G_OFF);
	PANEL_LED_VALUE(TEMP_R_OFF);
	PANEL_LED_VALUE(TEMP_E_G_OFF);
	PANEL_LED_VALUE(TEMP_E_R_OFF);
}

void Low_load()
{
	uint8_t c = 0;
	uint16_t d = 0;

	c = 1U;
	if(c == gval.gpio_read.Pwr_Sw_OnOff)
	{
		gval.timer_flag_gval.start_timer_flag = 1U;
	}
	else
	{
		gval.timer_flag_gval.start_timer_flag = 0;
	}

	// 초기 2초간 동작 
	d = 20U;
	if(d > gval.timer_gval.start_timer)
	{
		gval.flag.low_start_flag = 0;
	}
	else
	{
		gval.flag.low_start_flag = 1U;
	}

	// Tiriger Mode Timer 새로 생성
	d = 50U;
	if(d > gval.timer_gval.triger_timer)
	{
		gval.flag.Triger_start_flag = 0;
	}
	else
	{
		gval.flag.Triger_start_flag = 1U;
	}

	Low_load_Voltage_Control();

}

// 트리거모드 과부하 펄스 모드
void PulseMode()
{
	if(gval.can_cbit.fw_lamp_status.PulseMode == ON)
	{
		gval.timer_flag_gval.Pulse_timer_flag =  1U;

		if(0 == gval.flag.Triger_start_flag)
		{
			if(gval.timer_gval.triger_timer < 42)
			{
				// 타임어 진행 중 실행
				Zero_load_Voltage_220();
				gval.flag.wait_volt_time_flag = 0;
				// 검사전에 초기화화
				gval.error_counter.PulseMode_Counter = 0;
			}
			if(gval.timer_gval.triger_timer >= 42)
			{
				Heavy_load_Voltage_220();
				gval.flag.wait_volt_time_flag = 1U;

				// 500ms 이후 300ms 시간동안 펄스모드 진입 여부 확인 
				// 펄스 모드 빠져나오는 동작 확인.
				if((gval.timer_gval.triger_timer >= 47) && (gval.timer_gval.triger_timer <= 49))
				{
					// 20A 이하로 측정시 하드웨어 트립상태로 판단한다.
					if(gval.adVal_1.real_1.PWR_1_40A < 20.0)
					{
						// 500ms 이후 전류가 없으면 하드웨어 트립으로 감지한다.
						// 이때 무부하 조건은 무시한다.
						if(gval.error_counter.PulseMode_Counter<500)gval.error_counter.PulseMode_Counter++;
					}
					else
					{
						if(gval.error_counter.PulseMode_Counter>0)gval.error_counter.PulseMode_Counter--;
					}
				}
			}
		}
		else
		{
			// 하드웨어 트립 없으면 펄스모드 빠져나온다
			if(gval.error_counter.PulseMode_Counter < 200)
			{
				gval.error_counter.PulseMode_Counter = 0;
				gval.can_cbit.fw_lamp_status.PulseMode = OFF;
			}

			Heavy_load_Voltage_220();
			gval.flag.wait_volt_time_flag = 1U;

			// timer 끝나면 실행
			gval.timer_flag_gval.Pulse_timer_flag =  0;
		}
	}
	else
	{
		// 정상부하에 진입하여 과전류가 하단 이하로 진입하는지 확인한다.
		Heavy_load_Voltage_220();
		gval.flag.wait_volt_time_flag = 1U;
	}
}

// 과부하 펄스모드 진입 조건 체크
void PulseModeCheck()
{
	// 피크전류가 현재값+50A보다 크면 펄스모드 진입
	// 초기 전류변화에 잘못 진입 하는 경우가 있어 조건 강화함 
	if(((gval.adVal_1.real_1.PWR_1_40A_PEAK > 50.0)
	    && ((gval.adVal_1.real_1.PWR_1_40A_PEAK - gval.adVal_1.real_1.PWR_1_40A) > 50.0))
		&& (gval.adVal_1.real_1.PWR_1_40A < 20))
	{
		// 하드웨어 트립이 감지될시
		if(gval.error_counter.PulseMode_Counter<5000)gval.error_counter.PulseMode_Counter++;

		if(gval.error_counter.PulseMode_Counter > 1000)
		{
			gval.error_counter.PulseMode_Counter = 0;

			// 펄스 모드 진입
			gval.can_cbit.fw_lamp_status.PulseMode = ON;

			gval.timer_flag_gval.Pulse_timer_flag = 0;
			gval.flag.wait_volt_time_flag = 0;
		}
	}
	else
	{
		if(gval.error_counter.PulseMode_Counter>0)gval.error_counter.PulseMode_Counter--;
	}
}

// 정상 범위 내에 전류 모드
void NormalMode()
{
	if(0 == gval.flag.system_error)
	{
		if(3.0f > gval.adVal_1.real_1.PWR_1_40A_PEAK)
		{
			Zero_load_Voltage_220();
			gval.flag.wait_volt_time_flag = 0;
			gval.can_cbit.fw_lamp_status.temporyDataModeData = 1;

		}
		else if((3.0f <= gval.adVal_1.real_1.PWR_1_40A_PEAK) && (10.0f > gval.adVal_1.real_1.PWR_1_40A_PEAK))
		{
			Light_load_Voltage_220();
			gval.flag.wait_volt_time_flag = 0;
			gval.can_cbit.fw_lamp_status.temporyDataModeData = 2;
		}
		else
		{
			Heavy_load_Voltage_220();
			gval.flag.wait_volt_time_flag = 1U;
			gval.can_cbit.fw_lamp_status.temporyDataModeData = 3;
		}
	}
}

// 전류 제한 119 -> 150으로 변경 플리커 모드에서 전류트립 발생하여 임시조치
// 하드웨어 트립 아닌구간 < > 하드웨어 트립구간 구분하는 방법 생각할것
// 저전류에서 동작 시퀀스 생각해둘것.
// ON/OFF시에 트리거모드로 진입 못함
void Low_load_Voltage_Control()
{
	if(0 == gval.flag.system_error)
	{
		if(1U == gval.flag.low_start_flag)
		{
			if(gval.can_cbit.fw_lamp_status.PulseMode==ON) 
			{
				// 과전류시에 펄스모드 진입입
				PulseMode();
			}
			else 
			{
				// 정상 상태에서 동작
				NormalMode();
			}
		}
		else
		{
			// 700ms 이후 펄스모드 진입 여부 확인
			if(gval.timer_gval.start_timer > 7)
			{
				PulseModeCheck();
			}

			Heavy_load_Voltage_220();

			EX1_Load_Cont_ONOFF(0);
			EX2_Load_Cont_ONOFF(0);
			EX3_Load_Cont_ONOFF(0);
		}
	}
}

void Zero_load_Voltage_220()
{
	if((0 == gval.flag.error_dc1_volt) || (0 == gval.flag.error_dc2_volt) || (0 == gval.flag.error_dc3_volt))
	{
		PANEL_LED_VALUE(DC_G_ON);
		PANEL_LED_VALUE(DC_R_OFF);

		Zero_Load_Check_Pwr_State();
	}
	else
	{
		PANEL_LED_VALUE(DC_G_OFF);
		PANEL_LED_VALUE(DC_R_ON);
	}

	EX1_Load_Cont_ONOFF(1U);
	EX2_Load_Cont_ONOFF(1U);
	EX3_Load_Cont_ONOFF(1U);

	CAL_Pwr_OnOff_time();
}

void Zero_Load_Check_Pwr_State()
{
	if(0 == gval.flag.error_dc1_volt)
	{
		PANEL_LED_VALUE(P1_G_ON);
		PANEL_LED_VALUE(P1_R_OFF);
	}
	else
	{
		PANEL_LED_VALUE(P1_G_OFF);
		PANEL_LED_VALUE(P1_R_ON);
	}

	if(0 == gval.flag.error_dc2_volt)
	{
		PANEL_LED_VALUE(P2_G_ON);
		PANEL_LED_VALUE(P2_R_OFF);
	}
	else
	{
		PANEL_LED_VALUE(P2_G_OFF);
		PANEL_LED_VALUE(P2_R_ON);
	}

	if(0 == gval.flag.error_dc3_volt)
	{
		PANEL_LED_VALUE(P3_G_ON);
		PANEL_LED_VALUE(P3_R_OFF);
	}
	else
	{
		PANEL_LED_VALUE(P3_G_OFF);
		PANEL_LED_VALUE(P3_R_ON);
	}
}


// OFF 타임을 줄일수록 전압이 작아지는듯 하다
void CAL_Pwr_OnOff_time()
{
	uint16_t on_time = 0;
	uint16_t off_time = 0;

	uint16_t total_time = 0;
	uint16_t pre_timer = 0;

	if(1.0f > gval.adVal_1.real_1.PWR_1_40A)
	{
		on_time = 25U;
		off_time = 155U;

		EX1_Load_Cont_ONOFF(1U);
		EX2_Load_Cont_ONOFF(1U);
		EX3_Load_Cont_ONOFF(1U);
	}
	else if((1.0f <= gval.adVal_1.real_1.PWR_1_40A)&&(2.0f > gval.adVal_1.real_1.PWR_1_40A))
	{
		on_time = 20U;
		off_time = 160U;

		EX1_Load_Cont_ONOFF(1U);
		EX2_Load_Cont_ONOFF(1U);
		EX3_Load_Cont_ONOFF(1U);
	}
	else if((2.0f <= gval.adVal_1.real_1.PWR_1_40A)&&(3.0f > gval.adVal_1.real_1.PWR_1_40A))
	{
		on_time = 20U;
		off_time = 165U;
	}
	else if((3.0f <= gval.adVal_1.real_1.PWR_1_40A)&&(4.0f > gval.adVal_1.real_1.PWR_1_40A))
	{
		on_time = 20U;
		off_time = 170U;
	}
	else if((4.0f <= gval.adVal_1.real_1.PWR_1_40A)&&(4.5f > gval.adVal_1.real_1.PWR_1_40A))
	{
		on_time = 20U;
		off_time = 175U;
	}
	else
	{
		on_time = 20U;
		off_time = 175U;
	}

	// 펄스모드에서는 듀티를 고정한다.
	if(gval.can_cbit.fw_lamp_status.PulseMode == ON)
	{
		on_time = 15U;		// OFF 타임
		off_time = 140U;	// ON  타임 하드웨어 꺼지는 증상으로 하향함.
							// ON 타임을 너무 길게하면 HW트립 발생
							// ON 타임을 너무 짧게하면 턴온 불가
							// 적절한 조건에서 설정할 것.
	}
	
	gval.flag.low_load_flag = 1U;

	gval.flag.low_cont_state = 1U;

	gval.flag.wait_time_flag = 1U;

	Wait_Timer_control();

	total_time = on_time + off_time;

	pre_timer = (uint16_t)(gval.timer_gval.low_load_cont_timer % total_time);

	gval.timer_gval.low_load_cont_timer = pre_timer;

	OnOff_Control(on_time);
}

void Wait_Timer_control()
{
	uint16_t a = 12U;

	if(a < gval.timer_gval.wait_timer)
	{
		gval.flag.wait_time = 1U;

		gval.timer_flag_gval.wait_timer_flag = 0;
	}
	else
	{
		gval.flag.wait_time = 0;

		gval.timer_flag_gval.wait_timer_flag = 1U;
	}
}

void OnOff_Control(uint16_t on_time)
{
	if(on_time <= gval.timer_gval.low_load_cont_timer)
	{
		if(0 == gval.flag.error_dc1_volt)
		{
			// PULSE 모드에서는 POWER ALL ON
			if(gval.can_cbit.fw_lamp_status.PulseMode == ON)
			{
				gval.flag.p1_status = 1U;
				gval.flag.p2_status = 1U;
				gval.flag.p3_status = 1U;
			}
			else
			{
				gval.flag.p1_status = 1U;
				gval.flag.p2_status = 0;
				gval.flag.p3_status = 0;
			}
		}
		else
		{
			Pwr_1_error_State();
		}
	}
	else
	{
		gval.flag.p1_status = 0;
		gval.flag.p2_status = 0;
		gval.flag.p3_status = 0;
	}
}

void Pwr_1_error_State()
{
	if(0 == gval.flag.error_dc2_volt)
	{
		if(gval.can_cbit.fw_lamp_status.PulseMode == ON)
		{
			gval.flag.p1_status = 1U;
			gval.flag.p2_status = 1U;
			gval.flag.p3_status = 1U;
		}
		else
		{
			gval.flag.p1_status = 0;
			gval.flag.p2_status = 1U;
			gval.flag.p3_status = 0;			
		}
	}
	else
	{
		if(0 == gval.flag.error_dc3_volt)
		{
			gval.flag.p1_status = 0;
			gval.flag.p2_status = 0;
			gval.flag.p3_status = 1U;
		}
		else
		{
			__NOP();
		}
	}
}

void Light_load_Voltage_220()
{
	PANEL_LED_VALUE(DC_G_ON);
	PANEL_LED_VALUE(DC_R_OFF);

	gval.flag.low_load_flag = 1U;
	if(0 == gval.flag.error_dc1_volt)
	{
		PANEL_LED_VALUE(P1_G_ON);
		PANEL_LED_VALUE(P1_R_OFF);
	}
	else
	{
		PANEL_LED_VALUE(P1_G_OFF);
		PANEL_LED_VALUE(P1_R_ON);
	}

	if(0 == gval.flag.error_dc2_volt)
	{
		PANEL_LED_VALUE(P2_G_ON);
		PANEL_LED_VALUE(P2_R_OFF);
	}
	else
	{
		PANEL_LED_VALUE(P2_G_OFF);
		PANEL_LED_VALUE(P2_R_ON);
	}

	if(0 == gval.flag.error_dc3_volt)
	{
		PANEL_LED_VALUE(P3_G_ON);
		PANEL_LED_VALUE(P3_R_OFF);
	}
	else
	{
		PANEL_LED_VALUE(P3_G_OFF);
		PANEL_LED_VALUE(P3_R_ON);
	}

	// 더미저항 P1모드에서도 동작 변경
	EX1_Load_Cont_ONOFF(ON);
	EX2_Load_Cont_ONOFF(ON);
	EX3_Load_Cont_ONOFF(ON);

	if(0 == gval.flag.error_dc1_volt)
	{
		gval.flag.p1_status = 1U;
		gval.flag.p2_status = 0;
		gval.flag.p3_status = 0;
	}
	else
	{
		if(0 == gval.flag.error_dc2_volt)
		{
			gval.flag.p1_status = 0;
			gval.flag.p2_status = 1U;
			gval.flag.p3_status = 0;
		}
		else
		{
			if(0 == gval.flag.error_dc3_volt)
			{
				gval.flag.p1_status = 0;
				gval.flag.p2_status = 0;
				gval.flag.p3_status = 1U;
			}
			else
			{
				__NOP();
			}
		}
	}
}

void Heavy_load_Voltage_220()
{
	gval.flag.low_load_flag = 0;
	gval.flag.low_cont_state = 0;
	EX1_Load_Cont_ONOFF(0);
	EX2_Load_Cont_ONOFF(0);
	EX3_Load_Cont_ONOFF(0);

	if(0 == gval.flag.error_dc1_volt)
	{
		gval.flag.p1_status = 1U;
	}
	else
	{
		gval.flag.p1_status = 0;
	}

	if(0 == gval.flag.error_dc2_volt)
	{
		gval.flag.p2_status = 1U;
	}
	else
	{
		gval.flag.p2_status = 0;
	}

	if(0 == gval.flag.error_dc3_volt)
	{
		gval.flag.p3_status = 1U;
	}
	else
	{
		gval.flag.p3_status = 0;
	}
}

void LED_Display()
{
	uint8_t b = 16U;

	for(uint16_t a = 0; a < b; a++)
	{
		gval.tlc59482_data.tlc59482_data_32_real[a] = gval.tlc59482_data.tlc59482_data_32[a] ;
	}
}

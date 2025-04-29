/*
 * PWR_3KW_adc.c
 *
 *  Created on: May 25, 2022
 *      Author: user
 */

#include "main.h"
#include "PWR_3KW_driver.h"
#include "math.h"

// PEAK치 검출 배열 사이즈 주기 100ms 예상됨.
// 적절한 배열 크기로 선정해야함.
//#define SIZE 30  
#define SIZE 50  

uint8_t data[SIZE] =  { 0.0, }; // 배열 초기화
uint8_t max = 0;
uint16_t count = 0;

uint8_t adc_ch_1 = 0;
uint8_t ad_ok_1 = 0;

// 최대값을 찾는 함수
uint8_t findMax() {
	max = 0;
    for (int i = 1; i < SIZE; i++) {
        if (data[i] > max) {
            max = data[i];  // 더 큰 값이 발견되면 갱신
        }
    }
    return max;
}

const void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(HAL_OK == hadc->ErrorCode)
	{
		if(hadc->Instance == hadc1.Instance)
		{
			;
		}
	}
}

void ADC_Conversion_1()
{
	f32 ftemp = 0.0f;
	f32 ftemp_Lim = 0.0f;
	uint16_t ftemp_l_lim = (uint16_t)(IN_AC_L_LIMIT * SENSOR_LIM_RANGE_L);
	uint16_t ftemp_h_lim = (uint16_t)(IN_AC_H_LIMIT * SENSOR_LIM_RANGE_H);

	ftemp = gval.adVal_1.ad[eAD_PWR_AC] * CAL_CONSTANT_12BIT_VOLTAGE;

	ftemp_Lim = ftemp * CAL_CONSTANT_AC_VOLTAGE;

	if(ftemp_l_lim > (uint16_t)ftemp_Lim )
	{
		ftemp_Lim = IN_AC_L_LIMIT * SENSOR_LIM_RANGE_L;
	}
	else if(ftemp_h_lim < (uint16_t)ftemp_Lim )
	{
		ftemp_Lim = IN_AC_H_LIMIT * SENSOR_LIM_RANGE_H;
	}
	else
	{
		__NOP();
	}
	gval.adVal_1.real_1.PWR_AC = ftemp_Lim; // channel 15
	AD_Filter_d[eAD_PWR_AC] = gval.adVal_1.real_1.PWR_AC;
	ftemp = 0.0f;
	ftemp_Lim = 0.0f;

	//	 Digital AD * Mili Volt Coeff * 0.078674
	AD_Filter_d[eAD_PWR_1_28V] = DC_V_CALC(gval.adVal_1.ad[eAD_PWR_1_28V]); // channel 3 p1 voltage

	AD_Filter_d[eAD_PWR_2_28V] = DC_V_CALC(gval.adVal_1.ad[eAD_PWR_2_28V]); // channel 4 p2 voltage

	AD_Filter_d[eAD_PWR_3_28V] = DC_V_CALC(gval.adVal_1.ad[eAD_PWR_3_28V]); // channel 5 p3 voltage

	AD_Filter_d[eAD_PWR_1_40A] = DC_A_CALC(gval.adVal_1.ad[eAD_PWR_1_40A]); // channel 0 null

	AD_Filter_d[eAD_PWR_2_40A] = DC_A_CALC(gval.adVal_1.ad[eAD_PWR_2_40A]); // channel 1 total_Current

	AD_Filter_d[eAD_PWR_3_40A] = DC_A_CALC(gval.adVal_1.ad[eAD_PWR_3_40A]); // channel 2 null

	// Out Temp Sensor(NTC) Volt = (Digital AD * Volt Coeff) * 4.0 [V]
	// -40 ~ 25.0 = y = -3.349x4 + 22.88x3 - 58.281x2 + 87.01x - 55.355
	// 25.0 ~125  = y = 10.462x4 - 140.56x3 + 707.22x2 - 1551.9x + 1273.6
	AD_Filter_d[eAD_PWR_1_TEMP3] = NTC_CALC(gval.adVal_1.ad[eAD_PWR_1_TEMP3]); // channel 6 pwr1_temp3

	AD_Filter_d[eAD_PWR_2_TEMP3] = NTC_CALC(gval.adVal_1.ad[eAD_PWR_2_TEMP3]); // channel 7 pwr2_temp3

	AD_Filter_d[eAD_PWR_3_TEMP3] = NTC_CALC(gval.adVal_1.ad[eAD_PWR_3_TEMP3]); // channel 8 pwr3_temp3

	AD_Filter_d[eAD_PWR_1_TEMP4] = NTC_CALC(gval.adVal_1.ad[eAD_PWR_1_TEMP4]); // channel 9 pwr1_temp4

	AD_Filter_d[eAD_PWR_2_TEMP4] = NTC_CALC(gval.adVal_1.ad[eAD_PWR_2_TEMP4]); // channel 10 pwr2_temp4

	AD_Filter_d[eAD_PWR_3_TEMP4] = NTC_CALC(gval.adVal_1.ad[eAD_PWR_3_TEMP4]); // channel 11 pwr3_temp4
}

// ADC 채널데이터를 평균 필터링 한다.
void Filter_ADC_1()
{
	uint16_t filter_num = FILTER_NUM;

	for(uint32_t i = 0; i < eMax_Ad_Channel_1; i++)
	{
		gval.adVal_1.filter[i].sum -= gval.adVal_1.filter[i].val[gval.adVal_1.filter[i].pos];
		gval.adVal_1.filter[i].val[gval.adVal_1.filter[i].pos] = AD_Filter_d[i];
		gval.adVal_1.filter[i].sum += gval.adVal_1.filter[i].val[gval.adVal_1.filter[i].pos];
		++gval.adVal_1.filter[i].pos;

		if(filter_num <= gval.adVal_1.filter[i].pos)
		{
			gval.adVal_1.filter[i].pos = 0;
		}
		gval.adVal_1.filter[i].avr = gval.adVal_1.filter[i].sum/FILTER_NUM;
	}

	// 전류는 필터없이 그대로 가져올것 필터 적용시 결과에 영향이 있음.
	data[count++] = (uint8_t)AD_Filter_d[eAD_PWR_1_40A];
	if(count >= SIZE) {count = 0;}

	// 최대치 추출 전류값
	gval.adVal_1.real_1.PWR_1_40A_PEAK = findMax(data, SIZE);

	// gval 변수에 저장
	gval.adVal_1.real_1.PWR_1_40A = gval.adVal_1.filter[0].avr;
	gval.adVal_1.real_1.PWR_2_40A = gval.adVal_1.filter[1].avr;
	gval.adVal_1.real_1.PWR_3_40A = gval.adVal_1.filter[2].avr;
	gval.adVal_1.real_1.PWR_1_28V = gval.adVal_1.filter[3].avr;
	gval.adVal_1.real_1.PWR_2_28V = gval.adVal_1.filter[4].avr;
	gval.adVal_1.real_1.PWR_3_28V = gval.adVal_1.filter[5].avr;
	gval.adVal_1.real_1.PWR_1_TEMP3 = gval.adVal_1.filter[6].avr;
	gval.adVal_1.real_1.PWR_2_TEMP3 = gval.adVal_1.filter[7].avr;
	gval.adVal_1.real_1.PWR_3_TEMP3 = gval.adVal_1.filter[8].avr;
	gval.adVal_1.real_1.PWR_1_TEMP4 = gval.adVal_1.filter[9].avr;
	gval.adVal_1.real_1.PWR_2_TEMP4 = gval.adVal_1.filter[10].avr;
	gval.adVal_1.real_1.PWR_3_TEMP4 = gval.adVal_1.filter[11].avr;
	gval.adVal_1.real_1.PWR_AC = gval.adVal_1.filter[12].avr;

	// CAN 구조체로 데이터 전달
	gval.can_cbit.fw_volt_current_1.voltage = gval.adVal_1.real_1.PWR_1_28V;
	gval.can_cbit.fw_volt_current_2.voltage = gval.adVal_1.real_1.PWR_2_28V;
	gval.can_cbit.fw_volt_current_3.voltage = gval.adVal_1.real_1.PWR_3_28V;

	gval.can_cbit.fw_volt_current_1.current = 0;
	gval.can_cbit.fw_volt_current_2.current = gval.adVal_1.filter[0].avr;
	gval.can_cbit.fw_volt_current_3.current = 0;

	gval.can_cbit.fw_temp_temp_1_2.temperature_3 = gval.adVal_1.real_1.PWR_1_TEMP3;
	gval.can_cbit.fw_temp_temp_1_2.temperature_4 = gval.adVal_1.real_1.PWR_1_TEMP4;

	gval.can_cbit.fw_temp_temp_2_2.temperature_3 = gval.adVal_1.real_1.PWR_2_TEMP3;
	gval.can_cbit.fw_temp_temp_2_2.temperature_4 = gval.adVal_1.real_1.PWR_2_TEMP4;

	gval.can_cbit.fw_temp_temp_3_2.temperature_3 = gval.adVal_1.real_1.PWR_3_TEMP3;
	gval.can_cbit.fw_temp_temp_3_2.temperature_4 = gval.adVal_1.real_1.PWR_3_TEMP4;

	gval.can_cbit.fw_lamp_status.temporyDataPeakCurrent = (uint8_t)gval.adVal_1.real_1.PWR_1_40A_PEAK;
	gval.can_cbit.fw_lamp_status.temporyDataCurrent  = (uint8_t)gval.adVal_1.real_1.PWR_1_40A;
}

void Input_AC_CHECK(f32 in_AC)
{
	uint16_t Comp_ac = 0;
	const uint16_t a = IN_AC_L_LIMIT;
	const uint16_t b = IN_AC_H_LIMIT;

	Comp_ac = (uint16_t)in_AC;

	if((a <= Comp_ac) && (b >= Comp_ac))
	{
		PANEL_LED_VALUE(AC_G_ON);
		PANEL_LED_VALUE(AC_R_OFF);
	}
	else
	{
		__NOP();
	}
}


f32 DC_V_CALC(uint16_t ad)
{
	f32 ftemp = 0.000f;
	f32 ftemp_Lim = 0.000f;
	uint16_t a = (uint16_t)(OUT_DCV_L_LIMIT * SENSOR_LIM_RANGE_L);
	uint16_t b = (uint16_t)(OUT_DCV_H_LIMIT * SENSOR_LIM_RANGE_H);
	uint16_t c = 0;

	ftemp = (f32)ad * CAL_CONSTANT_12BIT_VOLTAGE;

	ftemp_Lim = ftemp * CAL_CONSTANT_PWR_28V;
	c = (uint16_t)ftemp_Lim;

	if( a > c )
	{
		ftemp_Lim = OUT_DCV_L_LIMIT * SENSOR_LIM_RANGE_L;
	}
	else if( b < c )
	{
		ftemp_Lim = OUT_DCV_H_LIMIT * SENSOR_LIM_RANGE_H;
	}
	else
	{
		__NOP();
	}

	return ftemp_Lim;
}

f32 DC_A_CALC(uint16_t ad)
{
	f32 ftemp = 0.0f;
	f32 ftemp_Lim = 0.0f;

	ftemp = (f32)ad * CAL_CONSTANT_12BIT_MILI_VOLT;
	ftemp_Lim = ftemp * 0.067f;

	return ftemp_Lim;
}

f32 NTC_CALC(uint16_t ad) // 볼트로 환산 후에 계산 할 것
{
	f32 mv = 0;
	uint16_t mv_2 = 0;
	f32 ftemp = 0;
	const uint16_t ftemp_2 = 0;
	uint16_t c = 0;

	const int16_t a = (int16_t)(NTC_L_LIMIT * SENSOR_LIM_RANGE_L);
	const uint16_t b = (uint16_t)(NTC_H_LIMIT * SENSOR_LIM_RANGE_H);

	mv = ((f32)ad * TEMP_COMP);// - 400; // 400MV 보정
	mv_2 = (uint16_t)mv;

	c = 3876U;
	if(c >=  mv_2)
	{
		c = 400U;
		if(mv_2 < c)
		{
			__NOP();
		}
		else
		{
			mv_2 = mv_2 - 400;
		}
	}
	else
	{
		mv_2 = mv_2 - 300;
	}

	ftemp = NTC_mVolt_Table_DB[(uint16_t)mv_2];

	if( a > ftemp_2 )
	{
		ftemp = NTC_L_LIMIT * SENSOR_LIM_RANGE_L;
	}
	else if( b < ftemp_2 )
	{
		ftemp = NTC_H_LIMIT * SENSOR_LIM_RANGE_H;
	}
	else
	{
		__NOP();
	}
	return ftemp;
}


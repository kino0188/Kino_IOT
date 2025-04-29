/*
 * PWR_3KW_driver.h
 *
 *  Created on: 2022. 5. 24.
 *      Author: user
 */

#ifndef INC_PWR_3KW_DRIVER_H_
#define INC_PWR_3KW_DRIVER_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "PWR_3KW_define.h"

#pragma pack(push, 1)
typedef struct
{
	uint32_t			BD_SerialNumber;
	f32				Average_DCV;
	f32				Total_DCA;
	f32				DC_POWER;
	uint16_t 			fan_l_freqency;
	uint16_t 			fan_r_freqency;
	uint8_t				test_sec;
	f32				before_dca[3];
	uint16_t			pre_overcurrent_time;

	GPIO_READ			gpio_read;
	GPIO_OUTPUT			gpio_output;
	COMMUNICATION		communication;
	S_AD_VAL_1			adVal_1;
	S_EEPROM_PARAM		eeprom;
	S_FLAG				flag;
	S_TLC59482_DATA		tlc59482_data;
	S_TLC59482_Control	control_data;
	S_RPM_CAL			rpm;
	S_CAN_CBIT			can_cbit;
	S_FAN_TEST			fan_test;
	S_ADC_LPF			adc_lpf;

	S_FW_LAMP_STATUE	before_led_status;
	S_FW_LAMP_STATUE	led_status;

	S_ERRROR_COUNTER	error_counter;

	S_TIMER_GVAL		timer_gval;
	S_TIMER_FLAG_GVAL	timer_flag_gval;

	S_AC_CAL_VAL		ac_cal_val;
	f32					low_voltage;
}gVal;
#pragma pack(pop)

extern gVal gval;

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc3;
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc3;

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim11;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

extern uint16_t wait_current_time;
extern uint16_t Wait_Volt_Time;
extern uint32_t Timer_Count;
extern uint32_t Sec_Time_Count;
extern uint16_t Three_min_Time_Count;

extern uint8_t CAN_Seq;
extern uint8_t FAN_L_Alt;
extern uint8_t FAN_R_Alt;

extern uint16_t Low_Load_Count_On_Timer;
extern uint16_t Low_Load_Count_Off_Timer;

extern uint8_t Digit;
extern uint32_t temp;
extern uint32_t Sin;
extern uint8_t	WRTGS_Count;

extern uint16_t AD_DATA[AD_CH];
extern float AC_In[100];

extern uint16_t AD_Inedex_1;
extern uint16_t AD_Inedex_3;
extern float AD_Filter_d[AD_CH];
extern float NTC_mVolt_Table_DB[NTC_mVolt_Table_index];
extern uint8_t adc_ch_1;
extern uint8_t adc_ch_3;
extern uint8_t ad_ok_1;
extern uint8_t ad_ok_3;

extern uint8_t fan_l;
extern uint8_t fan_r;

extern uint16_t read_count;
extern uint16_t read_count_zero;


//core

/*void SystemClock_Config();
static void MX_GPIO_Init();
static void MX_ADC1_Init();
static void MX_ADC3_Init();
static void MX_CAN1_Init();
static void MX_SPI3_Init();
static void MX_TIM1_Init();
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);*/

/*
void MX_TIM3_Init();
void MX_TIM4_Init();
void MX_TIM5_Init();
void MX_TIM6_Init();
void MX_TIM11_Init();
*/


void PWR_Parameter_Init(void);
void NTC_TABLE_CREATER();
void Self_Test_3kw_PWR();

void Input_AC_CHECK(float);

void PWR_Operation();
void ADC_Calc();
void Error_Check();
void Display_Panel();
void CAN_Communication();

void SN74LVC4245APW_Init(void);

void OPERATION_L_FAN(uint8_t mode, uint16_t Temp);
void OPERATION_R_FAN(uint8_t mode, uint16_t Temp);
void FAN_L_OnOff(uint8_t state);
void FAN_R_OnOff(uint8_t state);
void PWM_FAN_R_Period(uint32_t pwm);
void PWM_FAN_L_Period(uint32_t pwm);
uint8_t Fan_L_status(void);
uint8_t Fan_R_status(void);

void PANEL_LED_VALUE(E_LED_CONTROL Selection);

void PWR_DC_ON();
void PWR_DC_OFF();

void Check_Volt();
void Check_Current();
void Check_Temper();
void Check_Board();

float Peak_Search(float temp_1, float temp_2, float temp_3, float temp_4);

void Filter_Coeff();
void Low_Load_Counter();
uint8_t UPDATA_LED_STATUS(S_FW_LAMP_STATUE before_status, S_FW_LAMP_STATUE status);
void Heavy_load();
void Low_load();
void LED_Display();
void Low_load_1();
void NormalMode();
void PulseModeCheck();
void PulseMode();
void Low_load_Voltage_Control();
void Zero_load_Voltage_220();
void Light_load_Voltage_220();
void Heavy_load_Voltage_220();
void LED_Display();
void temp_fan(uint16_t pwm);

//gpio
void toggle_Test_LED_1(void);
void toggle_Test_LED_2(void);
uint8_t PWR_SW_Read(void);

void Sig_Out_En(void);
void EER_Chk_En(void);
void Sig_In_En(void);
void SPI_EN(void);

void Test_LED_1(uint8_t state);
void Test_LED_2(uint8_t state);

// TEST_START
void LED_Latch_OnOff(uint8_t state);
void Power1_Error_OnOff(uint8_t state);
void Power2_Error_OnOff(uint8_t state);
void Power3_Error_OnOff(uint8_t state);
void Out28_V_Over(uint8_t state);
void Out28_C_Over(uint8_t state);
void Over_Heat(uint8_t state);
void In_AC_status(uint8_t state);
void Out_DC_status(uint8_t state);

void PWR_Module1_Bias_OnOff(uint8_t state);
void PWR_Module2_Bias_OnOff(uint8_t state);
void PWR_Module3_Bias_OnOff(uint8_t state);

void PWR_Out1_Cont(uint8_t state);
void PWR_Out2_Cont(uint8_t state);
void PWR_Out3_Cont(uint8_t state);
void PWR_1_ON(uint8_t state);
void PWR_2_ON(uint8_t state);
void PWR_3_ON(uint8_t state);
void External_Start_Cmd(void);
uint8_t Pwr1_C_Err(void);
uint8_t Pwr2_C_Err(void);
uint8_t Pwr3_C_Err(void);
void NCS_EEPROM(uint8_t state);
void EEPROM_WP(uint8_t state);
void EEPROM_HOLD(uint8_t state);
uint8_t LED_SOUT();

uint8_t Power_Error_P1(void);
uint8_t Power_Error_P2(void);
uint8_t Power_Error_P3(void);

void Heavy1_Count_ONOFF(uint8_t state);
void Heavy2_Count_ONOFF(uint8_t state);
void Heavy3_Count_ONOFF(uint8_t state);
void Load_Control_ONOFF(uint8_t state);
void EX1_Load_Cont_ONOFF(uint8_t state);
void EX2_Load_Cont_ONOFF(uint8_t state);
void EX3_Load_Cont_ONOFF(uint8_t state);
void Gpio_Init();
void PWR_Bias_Init();
void PWR_Bias_Reset();
void Heavy_Count_Init();
void EX_Load_Cont_Init();
void System_Init();
void System_core();
void PulseModeCheck();
void Gpio_Init();
void PWR_Bias_Init();
void Heavy_Count_Init();
void EX_Load_Cont_Init();
void Pwr_Self_Test_end();
void Cal_Pwr_state(uint8_t Ready_time);
void Pwr_error();
void Pwr_Cooling_start();
void Pwr_Cooling_end();
void Fan_speed(uint16_t Fan_Ratio);
void Fan_error_check();
void Fan_error_check_1(uint8_t Fan_onoff);
void Can_Load_Test();
void Zero_Load_Check_Pwr_State();
void CAL_Pwr_OnOff_time();
void Wait_Timer_control();
void OnOff_Control(uint16_t on_time);
void Pwr_1_error_State();
void Non_over_load_led_state();



// SPI_TLC59482
void CLOCK(uint16_t state);
void SIN(uint16_t state);
void INPUT_DATA();
void SCLK_MODE_SELECT();
void TLC59482_Data_Recevie();

//uart
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void InitRingBuffer(void);
void Parsing_Debug(void);
void Parsing_Debug_Cmd(uint8_t* bf);
int32_t CheckSum_Check(uint8_t* data, uint32_t datanum);
uint16_t GetCheckSum(uint8_t data[], uint32_t datanum);
void Write_Debug_INPUT_STATE(void);
void Write_Debug_OUTPUT_STATE(void);
void Boot_Mode_In(void);
void FLASH_If_Init(void);
void Check_Gpio_1(uint8_t a);
void Check_Gpio_2(uint8_t a);
void Check_Gpio_3(uint8_t a);

//timer
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void LED_PWM(uint32_t pwm);

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void flag_20us();
void flag_2ms();
void flag_1ms();
void flag_10ms();
void flag_100ms();
void flag_1s();

//can
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* p_hcan);
void Init_CAN(CAN_HandleTypeDef* p_hcan);
void SEND_TO_UI();
void Send_CAN_1(uint32_t id, uint8_t* sd, uint32_t num);
void Send_CAN_2(uint32_t id, uint8_t* sd, uint32_t num);

void Error_CAN_TX();
void Load_CAN_TX();

void CAN_cmd(void);
void manual_on_off_cmd(void);
void pwr_err_test_cmd(void);
void pwr_over_test_cmd(void);

void fan_pwm_test_cmd();

//adc
void Read_Polling_ADC_1(void);
void Read_Polling_ADC_3(void);
/*void ADC_Conversion(void);
void Filter_ADC(void);*/
void ADC_Conversion_1(void);
void Filter_ADC_1(void);
void ADC_Conversion_3(void);
void Filter_ADC_3(void);

uint32_t ADC_channel_switch(const uint8_t ach);
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef* hadc);

float NTC_CALC(uint16_t ad);
float DC_V_CALC(uint16_t ad);
//float DC_A_CALC(uint16_t ad);
float DC_A_CALC(uint16_t ad);
void LPF_H(uint16_t Input, uint16_t *Output, uint16_t *PastInput, uint16_t *PastOutput, float limit);

// util
void DELAY_US(uint32_t time_us);
void DELAY_MS(uint32_t time_ms);

//TLC59482.C
void OUTPUT_DATA(uint16_t data, uint8_t edge);
void Display_LED_SCLK_Toggle();
void MULTI_GSCLK_Toggle();
void MULTI_GSCLK(uint16_t state);
void Display_LED_Control();
void TLC59482_SCLKS();
const void U1_LED_ON(it c);
const void U2_LED_ON(it c);
const void U1_LED_OFF(it c);
const void U2_LED_OFF(it c);
const void U1_LED_OFF_1(it c);
const void U1_LED_OFF_2(it c);
const void U1_LED_OFF_3(it c);
const void U2_LED_OFF_1(it c);
const void U2_LED_OFF_2(it c);


#ifdef __cplusplus
#endif /* __cplusplus */

#endif /* INC_PWR_3KW_DRIVER_H_ */

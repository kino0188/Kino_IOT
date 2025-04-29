/*
 * PWR_3KW_define.h
 *
 *  Created on: 2022. 5. 24.
 *      Author: user
 */

#ifndef INC_PWR_3KW_DEFINE_H_
#define INC_PWR_3KW_DEFINE_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "ring_buffer.h"

#define FW_3KW_Version						230305
#define FW_3KW_Version_Type					'a'

//adc
#define ADC_TIME_OUT_MS							1U
#define SAMPLE_FREQ								1000
#define FC_H									10.0F
#define FC_L									75.0F
#define FC_D									0.01

//uart,can
#define DEBUG_PACKET_SIZE 						130
#define RING_BUFFER_SIZE						2048U
#define CAN_DATA_FIELD_BYTE_SIZE_MAX			8U
#define PACKET_0_DEBUG_HEADER					0xffU
#define PACKET_1_DEBUG_UI23kw_INPUT_STATE		0x11U
#define PACKET_1_DEBUG_UI23kw_OUTPUT_STATE		0x00U
#define PACKET_1_DEBUG_UI23kw_EEPROM_REQUEST	0XE1U
#define PACKET_1_DEBUG_UI23kw_EEPROM_WRITE		0XE2U
#define PACKET_1_DEBUG_UI23kw_AD5270_WRITE		0XA2U
#define PACKET_1_DEBUG_UI23kw_TLC59482			0X59U
#define UART_LIMIT_TIME							100U
#define BOOTMODE_FLAG_FLASH_ADDR				(uint32_t)(0x08020000U)
#define BOOTMODE_FLAG_FLASH_VALUE				0x9999B007U
#define VECT_TAB_OFFSET							0x4000U

//timer , pwm
#define MAX_PWM_DUTY							1119U // cnt( Counter Period 값이 1120이 최대이다 )
#define MIN_PWM_DUTY							672U

//TLC59482 ON/OFF
#define init_val_32								0x03F003F0U // U1,U2 LED BC및 CONTORL BIT 설정(BC값은 최대)

//TLC59482 ON/OFF 값

#define TLC59482_LED_OFF						0X0000U
#define TLC59482_LED_ON_U1						0X1000U
#define TLC59482_LED_ON_U2						0x1500U

#define TLC59482_LED_ON_32_0					0X15001000U
#define TLC59482_LED_ON_32_1					0X00001000U 	// 첫번째 LED 킴
#define TLC59482_LED_ON_32_2					0X15000000U 	// 두번째 LED 킴
#define TLC59482_LED_OFF_32						0X00000000U 	// 두개 LED OFF

#define INIT_TIME								8U 				// 초기화 시간 8초

#define INIT_TIME_HALF							10U

#define NTC_mVolt_Table_index					5000U

//adc
#define AD_CH									22U

#define FILTER_NUM								10U
#define CAL_CONSTANT_12BIT_VOLTAGE				0.00080f // 3.3 / 4096 [V]
#define CAL_CONSTANT_12BIT_MILI_VOLT			0.8056640625f // 3300 / 4096 [mV]

//#define CAL_CONSTANT_AC_VOLTAGE					0.002817f // 1 / 0.002817 = 354.9875f
#define CAL_CONSTANT_AC_VOLTAGE						354.9875f
 // 계산 값 : Sensor Out_Current = (Input AC * 1.414) / (100 * 1000) * 2.5,  Out_Current * 120(Shunt Resistor) = Outplut Volt 0.004242f
 // 실제 측정 : 0.004242f로 하면 피크값을 두고 계산하므로 0.00260883f 8프로 => 0.0002087064
 // 0.0028175364f

/*
#define CAL_CONSTANT_PWR_28V					0.08338744f // 계산 측정 : 0.078674f//1%당 0.00078674 를 078674f에서 더하면 됨 //
// Out_Volt = Digital AD * Mili Volt Coeff * (1.5k/271.5k = 0.005524) * 8.0 * 1.78 =  Digital AD * Mili Volt Coeff * 0.005524 * 14.24
// Out_Volt = Digital AD * Mili Volt Coeff * 0.005524 * 14.24 = Digital AD * Mili Volt Coeff * 0.078674
*/
//#define CAL_CONSTANT_PWR_28V					0.05f // 1 / 0.05 = 20
#define CAL_CONSTANT_PWR_28V					20.0f
// Out_Volt = Digital AD * Mill Volt Coeff * (10k/200k = 0.05f) = Digital AD * Mill Volt Coeff * 0.05f

#define CAL_CONSTANT_PWR_40A					0.08f
// Out_Current = ((Digital AD * Mili Volt Coeff) - 2500) * 80(Coeff), [mA] ((Digital AD * Mili Volt Coeff) - 2500) * 0.08(Coeff), [A]


#define CAL_CONSTANT_NTC_VOLT					4.3f // 입력저항 분배비 // 실측이후 4.0f에서 4.3f으로 변경
// ( 저항을 4.7K로 단독으로 봤을 경우 )
// Out Temp Sensor(NTC) Volt = (Digital AD * Volt Coeff) * 4.0 [V]
// -40 ~ 25.0 = y = -3.349x4 + 22.88x3 - 58.281x2 + 87.01x - 55.355
// 25.0 ~125  = y = 10.462x4 - 140.56x3 + 707.22x2 - 1551.9x + 1273.6
// ( 합성저항으로 2.16K로 봤을 경우)
// - 40 ~ 25도  y = 25.778x3 - 85.345x2 + 118.23x - 48.694
// 25 ~ 125도 y = 4.3299x3 - 33.072x2 + 105.43x - 77.8
#define TEMP_COMP								(CAL_CONSTANT_12BIT_MILI_VOLT * CAL_CONSTANT_NTC_VOLT)

#define CAL_CONSTANT_24V						13.0f // 13 * 3300 / 4096
// Out_Volt = (Digital AD * Mili Volt Coeff) * 13.0

#define NTC_OFFSET								5U
#define CAL_CONSTANT_5V_3V						2.0f // 2 * 3300 / 4096
 // Out_Volt = (Digital AD * Mili Volt Coeff) * 2.0f
#define ADC_3_INDEX								15U

// 20us 
#define DISPLAY_PERIOD 							200U
#define FAN_CHK_PERIOD 							120U
#define ADC_PERIOD 								1002U 
#define CAN_PERIOD 								10002U
#define ONE_SEC_PERIOD							100000U

#define THREE_MIN 								5U 		// TEST로서 10초 원래는 180
#define THREE_SEC								12U
#define ONE_DAY									345600L

#define LOW_LOAD_POWER							300.0f

#define Init_AD_cnt								100U

#define SENSOR_LIM_RANGE_L						0.9f
#define SENSOR_LIM_RANGE_H						1.1f

#define ERR_SET_COUNT							30U		// ERR SET COUNT 3초

#define IN_AC_H_LIMIT							265U
#define IN_AC_L_LIMIT							79U

#define OUT_DCV_H_LIMIT							32.0f 	// OVP 
#define OUT_DCV_L_LIMIT							0.0f

#define OUT_DCA_H_LIMIT							40.0f
#define OUT_DCA_L_LIMIT							0.0f
#define OUT_DCA_SENSOR							2500.0f

#define NTC_H_LIMIT								105.0f
#define NTC_L_LIMIT								-35.0f

#define NTC_STD_LOW								2.500f
#define NTC_STD_MV_LOW							2500U 	

#define TOTAL_3DCA_OVER_LIMIT					117U
#define TOTAL_3DCA_H_LIMIT						107U

#define TOTAL_2DCA_OVER_LIMIT					86U
#define TOTAL_2DCA_H_LIMIT						71U

#define TOTAL_1DCA_OVER_LIMIT					43U
#define TOTAL_1DCA_H_LIMIT						36U

#define TOTAL_ZERO_LOAD							3U		

#define	TOTAL_QUAD_LOAD							27U
#define TOTAL_HALF_LOAD							54U
#define TOTAL_THREE_QUAD_LOAD					81U
#define TOTAL_FULL_LOAD							107U

#define TOTAL_OVERLOAD							123U	// 26V * 123A = 3128W 약 3200W 설정정

#define TEMPERATURE_NORMAL						30
#define TEMPERATURE_HOT							85
#define TEMPERATURE_LIMIT						105

typedef float f32;
typedef unsigned int ui;
typedef char cha;
typedef int it;

typedef enum
{
	NEG = 0,
	POS
}E_POS_NEG;

typedef enum
{
	FALSE = 0,
	TRUE
} E_TRUE_FALSE;

typedef enum
{
	OFF = 0,
	ON
} E_ON_OFF;

typedef enum
{
	LOW = 0,
	HIGH
} E_HIGH_LOW;

typedef enum
{
	eAD_PWR_1_40A,				// 0, ADC1_IN0 PWR1_40A
	eAD_PWR_2_40A,				// 1, ADC1_IN1 PWR2_40A
	eAD_PWR_3_40A,				// 2, ADC1_IN2 PWR3_40A
	eAD_PWR_1_28V,				// 3, ADC1_IN3 PWR1_28V
	eAD_PWR_2_28V,				// 4, ADC1_IN4 PWR2_28V
	eAD_PWR_3_28V,				// 5, ADC1_IN5 PWR3_28V
	eAD_PWR_1_TEMP3,			// 6, ADC1_IN6 PWR1_TEMP3
	eAD_PWR_2_TEMP3,			// 7, ADC1_IN7 PWR2_TEMP3
	eAD_PWR_3_TEMP3,			// 8, ADC1_IN8 PWR3_TEMP3
	eAD_PWR_1_TEMP4,			// 9, ADC1_IN9 PWR1_TEMP4
	eAD_PWR_2_TEMP4,			// 10, ADC1_IN10 PWR2_TEMP4
	eAD_PWR_3_TEMP4,			// 11, ADC1_IN11 PWR3_TEMP4
	eAD_PWR_AC,					// 12, ADC1_IN12 PWR_AC_IN_MCU
	eMax_Ad_Channel_1
} E_AD_Channel_1;

typedef enum
{
	WRTGS	= 1,
	LATGS	= 3,
	READFC	= 5,
	WRTFC	= 11,
	TMGRST	= 13,
	FCWRTEN	= 15
}E_TLC_COMMAND;

typedef enum
{
	// ON Command
	FAN_R_R_ON 		= 0,
	FAN_R_G_ON,
	FAN_L_R_ON,
	FAN_L_G_ON,
	DC_R_ON,
	DC_G_ON 		= 5,
	AC_R_ON,
	AC_G_ON,
	LOAD_100_R_ON,
	LOAD_100_G_ON,
	LOAD_75_R_ON 	= 10,
	LOAD_75_G_ON,
	LOAD_50_R_ON,
	LOAD_50_G_ON,
	LOAD_25_R_ON,
	LOAD_25_G_ON	= 15,
	CAN_R_ON,
	CAN_G_ON,
	P3_R_ON,
	P3_G_ON,
	P2_R_ON			= 20,
	P2_G_ON,
	P1_R_ON,
	P1_G_ON,
	TEMP_E_R_ON,
	TEMP_E_G_ON		= 25,
	TEMP_R_ON,
	TEMP_G_ON,

	// OFF Command
	FAN_R_R_OFF,
	FAN_R_G_OFF,
	FAN_L_R_OFF		= 30,
	FAN_L_G_OFF,
	DC_R_OFF,
	DC_G_OFF,
	AC_R_OFF,
	AC_G_OFF		= 35,
	LOAD_100_R_OFF,
	LOAD_100_G_OFF,
	LOAD_75_R_OFF,
	LOAD_75_G_OFF,
	LOAD_50_R_OFF	= 40,
	LOAD_50_G_OFF,
	LOAD_25_R_OFF,
	LOAD_25_G_OFF,
	CAN_R_OFF,
	CAN_G_OFF		= 45,
	P3_R_OFF,
	P3_G_OFF,
	P2_R_OFF,
	P2_G_OFF,
	P1_R_OFF		= 50,
	P1_G_OFF,
	TEMP_E_R_OFF,
	TEMP_E_G_OFF,
	TEMP_R_OFF,
	TEMP_G_OFF		= 55,
	FAN_R_R_Toggle
} E_LED_CONTROL;

#pragma pack(push, 1)
typedef struct
{
	uint8_t Pwr_Sw_OnOff : 1;
	uint8_t External_Start_Cmd : 1;
	uint8_t Pwr1_C_Err : 1;
	uint8_t Pwr2_C_Err : 1;
	uint8_t Pwr3_C_Err : 1;
	uint8_t Fan_L_status : 1;
	uint8_t Fan_R_status : 1;
	uint8_t Pwr_Pw_Bias : 1;

	uint8_t P1_Err : 1;
	uint8_t P2_Err : 1;
	uint8_t P3_Err : 1;

} GPIO_READ; // 2byte

typedef struct
{
	uint8_t LED_Latch_OnOff : 1;
	uint8_t FAN_L_ON_OnOff : 1;
	uint8_t FAN_R_ON_OnOff : 1;
	uint8_t SPI_EN_OnOff : 1;
	uint8_t Pwr1_C_Err_OnOff : 1;
	uint8_t Pwr2_C_Err_OnOff : 1;
	uint8_t Pwr3_C_Err_OnOff : 1;
	uint8_t Out28_V_Over : 1;

	uint8_t Out28_C_Over : 1;
	uint8_t Over_Heat : 1;
	uint8_t In_AC_status : 1;
	uint8_t Out_DC_status : 1;
	uint8_t PWR1_ON : 1;
	uint8_t PWR2_ON : 1;
	uint8_t PWR3_ON : 1;
	uint8_t PWR_Out1_Cont : 1;

	uint8_t PWR_Out2_Cont : 1;
	uint8_t PWR_Out3_Cont : 1;
	uint8_t PWR_1_ON : 1;
	uint8_t PWR_2_ON : 1;
	uint8_t PWR_3_ON : 1;
	uint8_t ncs_eeprom : 1;
	uint8_t eeprom_wp : 1;
	uint8_t eeprom_hold : 1;

	uint8_t ex1_load_cont : 1;
	uint8_t ex2_load_cont : 1;
	uint8_t ex3_load_cont : 1;
} GPIO_OUTPUT; // 3byte

typedef struct
{
	uint32_t				PortNum;
	uint32_t 				Baud;
	uint8_t					TXD[DEBUG_PACKET_SIZE];
	uint8_t 				RXD[2];
	uint8_t 				ring_buffer1[RING_BUFFER_SIZE];
	ring_buffer				ring;
} S_UART_CONTROL;

typedef struct
{
	S_UART_CONTROL 			debug;
} USART;

typedef struct
{
	CAN_TxHeaderTypeDef   	TxHeader;
	CAN_RxHeaderTypeDef   	RxHeader;
	uint8_t               	TxData[CAN_DATA_FIELD_BYTE_SIZE_MAX];
	uint8_t               	RxData[CAN_DATA_FIELD_BYTE_SIZE_MAX];
	uint32_t              	TxMailbox;
} CAN;

typedef struct
{
	USART		usart;
	CAN			can;
	CAN			can_2;
} COMMUNICATION;


typedef struct
{
	f32 					PWR_1_40A;				// ADC1_IN0
	f32 					PWR_1_40A_PEAK;			// ADC1_IN0
	f32 					PWR_1_40A_PEAKtoPEAK;	// ADC1_IN0
	f32 					PWR_1_40A_PEAKGAP;		// ADC1_IN0
	f32						PWR_1_40A_PEAKBEFORE;
	f32 					PWR_2_40A;				// ADC1_IN1
	f32 					PWR_3_40A;				// ADC1_IN2
	f32 					PWR_1_28V;				// ADC1_IN3
	f32 					PWR_2_28V;				// ADC1_IN4
	f32 					PWR_3_28V;				// ADC1_IN5
	f32 					PWR_1_TEMP3;			// ADC1_IN6
	f32 					PWR_2_TEMP3;			// ADC1_IN7
	f32 					PWR_3_TEMP3;			// ADC1_IN8
	f32 					PWR_1_TEMP4;			// ADC1_IN9
	f32 					PWR_2_TEMP4;			// ADC1_IN10
	f32						PWR_3_TEMP4;			// ADC1_IN11
	f32						PWR_AC;					// ADC1_IN12
} S_ANALOG_VAL_1;

typedef struct
{
	uint16_t				pos;
	f32				 		val[FILTER_NUM];
	f32						sum;
	f32						avr;
} S_AD_MEDIAN;

typedef struct
{
	uint16_t				ad[eMax_Ad_Channel_1];			// 12bit
	S_AD_MEDIAN				filter[eMax_Ad_Channel_1];
	S_ANALOG_VAL_1			real_1;							// AD_1 각 채널의 입력값을 실제값으로 환산한 값
} S_AD_VAL_1;

typedef struct
{
	uint32_t 				req_param_save;
	uint32_t 				req_bb_save;
	uint32_t				req_round_save;
} S_EEPROM_CONTROL;

typedef struct
{
	// board serial number
	uint32_t				BD_SerialNumber;		// sn, BD S/N 8자리 자연수 ex) 12345678

	S_EEPROM_CONTROL control;
} S_EEPROM_PARAM;

typedef struct
{
	uint8_t	adc_1_flag : 1;
	uint8_t	adc_2_flag : 1;
	uint8_t Sout_flag : 1;
	uint8_t tlc59482_tx_flag : 1;
	uint8_t wrtgs_flag : 1;
	uint8_t panel_flag : 1;
	uint8_t gsclk_flag : 1;
	uint8_t fcwrten_flag : 1;
	uint8_t fan_flag : 1;
	uint8_t can_flag : 1;
	uint8_t errCheck_flag : 1;

	uint8_t wrtfc_flag : 1;
	uint8_t latgs_flag : 1;
	uint8_t tlc59482_data_end_flag : 1;
	uint8_t wait_time_flag : 1;
	uint8_t icp_1 : 1;
	uint8_t icp_2 : 1;
	uint8_t one_sec : 1;

	uint8_t test_flag : 1;
	uint8_t HwReset_flag : 1;
	uint8_t HwReset_ResetComplete_flag : 1;
	uint8_t fan_l_error : 1;
	uint8_t fan_r_error : 1;
	uint8_t ms10 : 1;
	uint8_t CAN_manual_on_off_test : 1;
	uint8_t pwr1_on_off : 1;
	uint8_t pwr2_on_off : 1;
	uint8_t pwr3_on_off : 1;

	uint8_t CAN_PWR_err_test : 1;
	uint8_t CAN_pwr_1_err_test : 1;
	uint8_t CAN_pwr_2_err_test : 1;
	uint8_t CAN_pwr_3_err_test : 1;
	uint8_t CAN_Over_chk_test : 1;
	uint8_t CAN_Overtemp_test : 1;
	uint8_t CAN_OverLoad_test : 1;
	uint8_t CAN_Overdcvoltage_test : 1;

	uint8_t error_dc1_volt : 1;
	uint8_t error_dc2_volt : 1;
	uint8_t error_dc3_volt : 1;
	uint8_t error_Over_Volt : 1;
	uint8_t error_dc1_current : 1;
	uint8_t error_dc2_current : 1;
	uint8_t error_dc3_current : 1;
	uint8_t error_Total_current : 1;
	uint8_t error_temp_flag : 1;
	uint8_t system_error : 1;

	uint8_t p1_status : 1;
	uint8_t p2_status : 1;
	uint8_t p3_status : 1;

	uint8_t DC_Out_Start_flag : 1;
	uint8_t DC_Out_Stop_flag : 1;
	uint8_t three_min_start_flag : 1;
	uint8_t before_pwr_sw_status : 1;
	uint8_t low_load_counter_on_timer : 1;
	uint8_t low_load_counter_off_timer : 1;
	uint8_t low_load_flag : 1;
	uint8_t wait_volt_time_flag : 1;

	uint8_t wait_current_time : 1;
	uint8_t pwr_1_low : 1;
	uint8_t pwr_2_low : 1;
	uint8_t pwr_3_low : 1;
	uint8_t pwr_1_high : 1;
	uint8_t pwr_2_high : 1;
	uint8_t pwr_3_high : 1;
	uint8_t fan_l_status_before : 1;

	uint8_t fan_r_status_before : 1;
	uint8_t real_pwr_sw_onoff : 1;
	uint8_t heavy1_cont_state : 1;
	uint8_t heavy2_cont_state : 1;
	uint8_t heavy3_cont_state : 1;
	uint8_t load_control : 1;
	uint8_t pre_overcurrent_flag : 1;
	uint8_t low_cont_state : 1;

	uint8_t wait_time : 1;
	uint8_t low_current_flag : 1;
	uint8_t low_start_flag : 1;
	uint8_t Triger_start_flag : 1;
} S_FLAG;

typedef struct
{
	uint32_t		tlc59482_data_32[16];
	uint32_t		tlc59482_data_32_real[16];
	/* -- LED PWM 들어가는 변수 --
	gval->tlc59482_data.tlc59482_data_32[0] = TLC59482_LED_OFF_32; // FAN_R_R , CAN_R
	gval->tlc59482_data.tlc59482_data_32[1] = TLC59482_LED_ON_32_0; // FAN_R_G , CAN_G
	gval->tlc59482_data.tlc59482_data_32[2] = TLC59482_LED_ON_32_0; // FAN_L_R , P3_R
	gval->tlc59482_data.tlc59482_data_32[3] = TLC59482_LED_OFF_32; // FAN_L_G , P3_G
	gval->tlc59482_data.tlc59482_data_32[4] = TLC59482_LED_OFF_32; // DC_R , P2_R
	gval->tlc59482_data.tlc59482_data_32[5] = TLC59482_LED_OFF_32; // DC_G , P2_G
	gval->tlc59482_data.tlc59482_data_32[6] = TLC59482_LED_OFF_32; // AC_R , P1_R
	gval->tlc59482_data.tlc59482_data_32[7] = TLC59482_LED_OFF_32; // AC_G , P1_G
	gval->tlc59482_data.tlc59482_data_32[8] = TLC59482_LED_OFF_32; // LOAD_100_R , X
	gval->tlc59482_data.tlc59482_data_32[9] = TLC59482_LED_OFF_32; // LOAD_100_G , X
	gval->tlc59482_data.tlc59482_data_32[10] = TLC59482_LED_OFF_32; // LOAD_75_R , X
	gval->tlc59482_data.tlc59482_data_32[11] = TLC59482_LED_OFF_32; // LOAD_75_G , X
	gval->tlc59482_data.tlc59482_data_32[12] = TLC59482_LED_OFF_32;// LOAD_50_R , TEMP_E_R
	gval->tlc59482_data.tlc59482_data_32[13] = TLC59482_LED_OFF_32; // LOAD_50_G , TEMP_E_G
	gval->tlc59482_data.tlc59482_data_32[14] = TLC59482_LED_OFF_32; // LOAD_25_R , TEMP_R
	gval->tlc59482_data.tlc59482_data_32[15] = TLC59482_LED_OFF_32; // LOAD_25_G , TEMP_G
	*/
} S_TLC59482_DATA;

typedef struct
{
	uint8_t				period_flag;
	uint32_t 			period[2];
	uint32_t 			frequency_1;
	uint32_t 			frequency_2;
	uint32_t 			RPM_R;
	uint32_t 			RPM_L;
} S_RPM_CAL;

typedef struct
{
	uint16_t			read_sout;
	uint16_t			rd;
	uint32_t			tlc59482_tx_data;
	uint16_t			tlc59482_edge;
	uint8_t				high;
	uint8_t				tlc59482_cnt;
	uint16_t			wrtgs_data_count;
	uint16_t			Count_Edge;
	uint8_t				Led_Control_Seq;
} S_TLC59482_Control;

typedef struct
{
	uint32_t version_number : 32;
	uint8_t	version_char : 8;
	uint32_t comm_count : 24;
} S_FW_VERSION;

typedef struct
{
	f32					voltage;
	f32					current;
} S_FW_VOLT_CURRENT;

typedef struct
{
	f32					power;
	f32					Total_Current;
} S_FW_POW_TEMP;

typedef struct
{
	f32					temperature_1;
	f32					temperature_2;
} S_FW_TEMP_TEMP;

typedef struct
{
	f32					temperature_3;
	f32					temperature_4;
} S_FW_TEMP_TEMP_1;

typedef struct
{
	f32				load;
	f32				average_volt;
} S_FW_LOAD;

typedef struct
{
	uint8_t				pow_on_off_lamp : 2;
	uint8_t				ac_input_lamp : 2;
	uint8_t				dc_output_lamp : 2;
	uint8_t				left_fan_lamp : 2;

	uint8_t				right_fan_lamp : 2;
	uint8_t				temp_ok_lamp : 2;
	uint8_t				temp_fail_lamp : 2;
	uint8_t				power_module_1_lamp : 2;

	uint8_t				power_module_2_lamp : 2;
	uint8_t				power_module_3_lamp : 2;
	uint8_t				over_volt_alert_lamp : 1;
	uint8_t				over_load_alert_lamp : 1;
	uint8_t				over_temp_alert_lamp : 1;
	uint8_t				reserverd			 : 1;

	uint8_t				load_level_1_lamp	 : 2;
	uint8_t				load_level_2_lamp	 : 2;
	uint8_t				load_level_3_lamp	 : 2;
	uint8_t				load_level_4_lamp	 : 2;

	uint8_t				temporyDataPeakCurrent : 8;
	uint8_t				temporyDataCurrent : 8;
	uint8_t				temporyDataModeData : 8;
	uint8_t				PulseMode : 8;

} S_FW_LAMP_STATUE;

typedef struct
{
	S_FW_VERSION		fw_version;
	S_FW_VOLT_CURRENT	fw_volt_current_1;
	S_FW_VOLT_CURRENT	fw_volt_current_2;
	S_FW_VOLT_CURRENT	fw_volt_current_3;
	S_FW_POW_TEMP		fw_pow_temp;
	S_FW_TEMP_TEMP		fw_temp_temp_1_1;
	S_FW_TEMP_TEMP_1	fw_temp_temp_1_2;
	S_FW_TEMP_TEMP		fw_temp_temp_2_1;
	S_FW_TEMP_TEMP_1	fw_temp_temp_2_2;
	S_FW_TEMP_TEMP		fw_temp_temp_3_1;
	S_FW_TEMP_TEMP_1	fw_temp_temp_3_2;
	S_FW_LOAD			fw_load;
	S_FW_LAMP_STATUE	fw_lamp_status;
} S_CAN_CBIT;

typedef struct
{
	uint16_t	c_status;
	uint16_t	l_status;

	uint16_t	ptn_counter;
	uint16_t	ntp_counter;
	uint16_t	ac_frequency;

	f32			peak_ac_volt;
	f32	 		peak_ac_volt_ad;
}S_AC_CAL_VAL;

typedef struct
{
	uint8_t		fan_test_mode;
	uint8_t		fan_test_left_pwm;
	uint8_t		fan_test_right_pwm;
	uint16_t	fan_left_pwm;
	uint16_t	fan_right_pwm;
}S_FAN_TEST;

typedef struct
{
	float Omega_H;
	float Omega_L;
	float Omega_D;

	float Alpha_H;
	float Alpha_L;
	float Alpha_D;

	float Betha_H;
	float Betha_L;
	float Betha_D;
}S_ADC_LPF;

typedef struct
{
	uint16_t over_load_counter;
	uint16_t non_over_load_counter;

	uint16_t over_voltage_counter1;
	uint16_t over_voltage_counter2;
	uint16_t over_voltage_counter3;

	uint16_t over_temp_Counter;
	uint16_t non_over_temp_Counter;

	uint16_t PulseMode_Counter;
}S_ERRROR_COUNTER;

typedef struct
{
	uint16_t low_load_cont_timer;
	uint16_t prepare_pwr_cont_timer;
	uint16_t wait_timer;
	uint16_t start_timer;
	uint16_t triger_timer;
}S_TIMER_GVAL;

typedef struct
{
	uint8_t prepare_pwr_cont_timer_flag : 1;
	uint8_t wait_timer_flag : 1;
	uint8_t start_timer_flag : 1;
	uint8_t Pulse_timer_flag : 1;
}S_TIMER_FLAG_GVAL;

#pragma pack(pop)

#ifdef __cplusplus
#endif /* __cplusplus */

#endif /* INC_PWR_3KW_DEFINE_H_ */

/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
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
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "ESP_Command.h"
#include "esp8266.h"
#include <string.h>  

#define RING_BUF_SIZE 2048
    
unsigned int unIPD_id;

u08 test_string[]                 = "AT+CWMODE=3\r\n";
u08 test_string_cwlap[]           = "AT+CWLAP\r\n";
u08 test_string_cwjap[]           = "AT+CWJAP=\"Kino0188\",\"72487248aa\"\r\n";
u08 test_string_cifsr[]           = "AT+CIFSR\r\n";
u08 test_string_send_stop[]       = "AT+CIPCLOSE=1\r\n";
u08 test_string_send_cipstart[]   = "AT+CIPSTART=1,\"TCP\",\"192.168.0.14\",80\r\n";
u08 test_string_cmd_webserver1[]  = "GET /reset HTTP/1.1\r\n";
u08 test_string_cmd_webserver2[]  = "Host: cse.dmu.ac.uk\r\n\r\n";
u08 test_string_send[]            = "AT+CIPSEND=1,115\r\n";
u08 test_string_send1[]           = "AUX-1O\r\n";
u08 test_string_send2[]           = "AUX-1X\r\n";
u08 test_string_atclpmux[]        = "AT+CIPMUX=1\r\n";
u08 test_string_clpserver[]       = "AT+CIPSERVER=1,80\r\n";
u08 hellowmsg2[]                  = "HTTP/1.1 200 OK\r\n";
u08 hellowmsg[4096];

u08 buf;
u08 buf2;

u08 ucbuf;

u08 ucTemps;

u08 ch_Ack = 0;
u08 ch_WaitAck = 0;
u08 Aux_Status = 0;
u08 *ucTemp;
u08 ucDebugString[12];

u08 msgkey = 0;

u08 switchkey = 0;
u08 switchkey2 = 0;

u08 switchkey_timeout = 0;

u08 Test_Sting[30] = "";
u08 ucLight_State = 'X';

u08 ucLink_State = 0;

u08 ring_buf[RING_BUF_SIZE]; // ring buffer
u08 uart_buffer[512][128];
u08 uart_buffer2 [128];

u08 chTcp_Cmd[254];
u08 nCmd_Count=0;
u08 nCmd_Count_tail=0;

u08 ucTemp2;

u16 ring_buf_len = 0; // ring buffer length
u16 ring_buf_lp = 0; // load pointer
u16 ring_buf_cp = 0; // consume pointer

uint8_t uart_buffer_temp[5];

uint8_t RxBuffer[100]; 

u16 nTask_WatchDog = 0;
u16 nTask_WatchDog_Count = 0;
u16 nReceive_Count = 0;
u16 nReceive_Count_tail = 0;
u16 nReceive_CountL = 0;

u16 nInput_Data = 0;

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

HAL_StatusTypeDef RcsStat;

osThreadId defaultTaskHandle;
osThreadId Tcpip_ControlTask;
osThreadId Uart_ControlTask;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/


void ring_buffer_init(void);
u16 ring_buffer_length(void);
void ring_buffer_push(u08 data);
u08 ring_buffer_pop(void);

int na;

void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_NVIC_Init(void);

void Eliminate(u08 *str, u08 ch);

void TCP_ReceiceData();
void Send_WebPage();
void make_msg(u08 *msg);
void send_msg(u08 *msg);
void send_msg2(u08 *msg);

void StartDefaultTask(void const * argument);
void TcpControlTask(void const * argument);
void UartControlTask(void const * argument);

int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_NVIC_Init();
  
  ring_buffer_init();

  HAL_UART_Transmit(&huart1," TCP IP Wireless TEST Program.. \r\n",100, 1000); 
 
  HAL_UART_Transmit(&huart2,"AT+RST", 10, 100); 
  
  HAL_Delay(100);
  
  HAL_UART_Transmit(&huart2,test_string, sizeof(test_string), 100); 
  
  HAL_Delay(100);

  HAL_UART_Transmit(&huart2,test_string_cifsr, sizeof(test_string_cifsr), 100); 
  
  HAL_Delay(100);

  HAL_UART_Transmit(&huart2,test_string_atclpmux, sizeof(test_string_atclpmux), 100); 
  
  HAL_Delay(100);

  HAL_UART_Transmit(&huart2,test_string_clpserver, sizeof(test_string_clpserver), 100); 
  
  HAL_Delay(100);
  
  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);
  
  osThreadDef(Tcpip_ControlTask, TcpControlTask, osPriorityNormal, 0, 128);
  Tcpip_ControlTask = osThreadCreate(osThread(Tcpip_ControlTask), NULL);
  
  osThreadDef(Uart_ControlTask, UartControlTask, osPriorityNormal, 0, 128);
  Uart_ControlTask = osThreadCreate(osThread(Uart_ControlTask), NULL);

  HAL_UART_Receive_IT(&huart2 , (uint8_t *)&buf,1); 
  HAL_UART_Receive_IT(&huart1 , (uint8_t *)&buf2,1); 
  
  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    ;
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
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}


/** NVIC Configuration
*/
static void MX_NVIC_Init(void)
{
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);
  
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)            //After Uart RX interrupt this function will be automatically called
{
      
    if(huart->Instance == USART1)                                  //Using USART1 check.
    {
      
       HAL_UART_Receive_IT(&huart1 , (uint8_t *) &buf2,1); 
        
       if(buf2 == 'A')
       {
         HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_1);
       }
       if(buf2 == 'B')
       {
         HAL_UART_Transmit(&huart2,"AT+RST\r\n", 8, 100); 
       }
       if(buf2 == 'C')
       {
         HAL_UART_Transmit(&huart2,test_string_cwjap, sizeof(test_string_cwjap), 100); 
       }
       if(buf2 == 'D')
       {
         HAL_UART_Transmit(&huart2,test_string_cifsr, sizeof(test_string_cifsr), 100); 
       }
       if(buf2 == 'E')
       {
         HAL_UART_Transmit(&huart2,test_string_atclpmux, sizeof(test_string_atclpmux), 100); 
       }
       if(buf2 == 'F')
       {
         HAL_UART_Transmit(&huart2,test_string_clpserver, sizeof(test_string_clpserver), 100); 
       }
       if(buf2 == '1')
       {
           sprintf(ucDebugString, "Lenth : %d \n\r", nReceive_Count);
           send_msg(ucDebugString);
       }
       if(buf2 == '2')
       {
          make_msg(hellowmsg);
          espsend(Test_Sting, strlen(hellowmsg)+1);
          send_msg2(Test_Sting);
       }
       if(buf2 == '3')
       {
          send_msg2(hellowmsg);
       }
       if(buf2 == '4')
       {
          espstop(Test_Sting);
          send_msg2(Test_Sting);
       }
    }
    
    if(huart->Instance == USART2)                                  //Using USART2 check.
    {   
      HAL_UART_Receive_IT(&huart2 , (uint8_t *)&buf,1);   
      HAL_UART_Transmit_IT(&huart1 , (uint8_t *)&buf,1);  
      
      ring_buffer_push(buf);
      
    }
}

void make_msg(u08 *msg)
{
      memset(msg, 0, sizeof(msg)); 
      make_html(msg, "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n<!DOCTYPE HTML>\r\n<html>");
      make_html(msg, "<head><title>ESP8266 LED Control</title></head>");
      make_html(msg, "<body>");
      if(ucLight_State == 'O')
      {
        make_html(msg, "<h>ESP8266 LED Control - ON</h></br></br>");
        make_html(msg, "<img alt=\"555.jpg\" src=\"https://i.imgur.com/BXM82qg.jpg\" width=\"200\" height=\"300\" />");
      }
      if(ucLight_State == 'X')
      {
        make_html(msg, "<h>ESP8266 LED Control - OFF</h></br></br>");
        make_html(msg, "<img alt=\"555.jpg\" src=\"https://i.imgur.com/F3mjhIz.jpg\" width=\"200\" height=\"300\"/>");
      }
      
      make_html(msg, "<br><input type=\"button\" name=\"b1\" value=\"Turn LED ON\" onclick=\"location.href=',LON'\">");
      make_html(msg, "<input type=\"button\" name=\"b1\" value=\"Turn LED OFF\" onclick=\"location.href=',LOFF'\">");
      make_html(msg, "</body>");
      make_html(msg, "</html>\n");
}

void Eliminate(u08 *str, u08 ch)
{
    for (; *str != '\0'; str++)//종료 문자를 만날 때까지 반복
    {
        if (*str == ch)//ch와 같은 문자일 때
        {
            strcpy(str, str + 1);
            str--;
        }
    }
}

void make_html(u08 *msg, u08 *data)
{
      sprintf(msg, "%s%s", msg, data);
}

void send_msg(u08 *msg)
{
      //Eliminate(msg, '\0'); 
      HAL_UART_Transmit(&huart1, msg, strlen(msg)+1, 500);
      HAL_UART_Receive_IT(&huart1 , (uint8_t *)&buf2,1);  
      memset(msg, 0, sizeof(msg)); 
}

void send_msg2(u08 *msg)
{
     // Eliminate(msg, '\0');
      HAL_UART_Transmit(&huart2, msg, strlen(msg)+1, 500);
      HAL_UART_Receive_IT(&huart2 , (uint8_t *)&buf,1);   
      memset(msg, 0, sizeof(msg)); 
}

void Send_WebPage()
{
  
  if(switchkey==1)
  {
        if(switchkey2==0)
        {
          if(ucLink_State == 1)
          {
            osDelay(500); // 기다려줄것 
            make_msg(hellowmsg);
            espsend(Test_Sting, strlen(hellowmsg)+1);
            send_msg("Sen PIs... \n\r");
            send_msg2(Test_Sting);
            switchkey2=1;
          }
        }
        
        if(switchkey2==2)
        {
          osDelay(30);
          send_msg2(hellowmsg);
          send_msg("Sen PIs2... \n\r");
          switchkey2 = 3;
        }
        
        if(switchkey2==4)
        {
          osDelay(30);
          espstop(Test_Sting);
          send_msg2(Test_Sting);
          send_msg("Sen PIs3... \n\r");
          switchkey2 = 0;
          switchkey = 0;
        }
        
        if(switchkey_timeout > 500)
        {
          send_msg("Web Page TimeOut... \n\r");
          switchkey2 = 0;
          switchkey = 0;
        }
        osDelay(1);
        switchkey_timeout++;        
  }
  
  if(switchkey==11)
  {
          send_msg("Reset ESP System \r\n");
          memset(uart_buffer, 0, sizeof(uart_buffer)); 
          memset(uart_buffer2, 0, sizeof(uart_buffer2)); 
          nReceive_Count = 0; nReceive_Count_tail=0;
          osDelay(500);
          send_msg2(test_string_atclpmux);
          osDelay(500);
          send_msg2(test_string_clpserver);          
          osDelay(500);
          switchkey = 0;
  }
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    
    HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_0);
    osDelay(1000);
    
    osDelay(1);
  }
  /* USER CODE END 5 */ 
}

void TcpControlTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  { 
    TCP_ReceiceData();
    Send_WebPage();

    osDelay(1);
  }
  /* USER CODE END 5 */ 
}

void UartControlTask(void const * argument)
{
  
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
 
  
  for(;;)
  {
    if (0 < ring_buf_len) 
    {
        ucTemps = ring_buffer_pop();
        if(ucTemps != NULL)
        {
          uart_buffer2[nReceive_CountL++] = ucTemps;
        }
        
        if(NULL != strstr(uart_buffer2, "\r\n") || NULL != strstr(uart_buffer2, ">") || NULL != strstr(uart_buffer2, "System Ready"))
        {
          if(nReceive_Count > 511)
          {
            memset(uart_buffer, 0, sizeof(uart_buffer)); 
            nReceive_Count = 0; nReceive_Count_tail=0;
          }
          
          memcpy(uart_buffer[nReceive_Count++], uart_buffer2, sizeof(uart_buffer2));
          memset(uart_buffer2, 0, sizeof(uart_buffer2)); 
          nReceive_Count++;
          nReceive_CountL = 0;
        }
    }
    else
    {
      if(nReceive_Count > nReceive_Count_tail)
      {
        if( NULL != strstr(uart_buffer[nReceive_Count_tail], "+IPD"))
        {
          ucTemp = strtok(uart_buffer[nReceive_Count_tail], ",");
          
          while(1)
          {
            if(NULL != ucTemp)
            {
              if( NULL != strstr(ucTemp, "+IPD"))
              {
                  ucTemp = strtok(NULL, ",");
                  unIPD_id = atoi(ucTemp);
              }
              else if( NULL != strstr(ucTemp, "GET"))
              {
                  ucTemp = strtok(NULL, ",");
              
                  if( NULL != strstr(ucTemp, "LON"))
                  {
                    chTcp_Cmd[nCmd_Count++] = 'e';
                  }
                  else if( NULL != strstr(ucTemp, "LOFF"))
                  {
                    chTcp_Cmd[nCmd_Count++] = 'f';
                  }
                  if(nCmd_Count > 200) { nCmd_Count = 0; }
              }
             
            }
            else
            {
              break;
            }
            ucTemp = strtok(NULL, ",");
            
          }
        }
        
        if(NULL != strstr(uart_buffer[nReceive_Count_tail], "link is not"))
        {
            if(nCmd_Count > 200) { nCmd_Count = 0; nCmd_Count_tail=0;}      
            chTcp_Cmd[nCmd_Count++] = 'h';
        }
        if(NULL != strstr(uart_buffer[nReceive_Count_tail], "busy inet"))
        {
            if(nCmd_Count > 200) { nCmd_Count = 0; nCmd_Count_tail=0;}      
            chTcp_Cmd[nCmd_Count++] = 'y';
        }
        if(NULL != strstr(uart_buffer[nReceive_Count_tail], "Accept-Language"))
        {
            if(nCmd_Count > 200) { nCmd_Count = 0; nCmd_Count_tail=0;}      
            chTcp_Cmd[nCmd_Count++] = 'd';           
        }
        if(NULL != strstr(uart_buffer[nReceive_Count_tail], "Link\r\n"))
        {
            if(nCmd_Count > 200) { nCmd_Count = 0; nCmd_Count_tail=0;}      
            chTcp_Cmd[nCmd_Count++] = 'l';           
        }
        if(NULL != strstr(uart_buffer[nReceive_Count_tail], "Unlink\r\n"))
        {
            if(nCmd_Count > 200) { nCmd_Count = 0; nCmd_Count_tail=0;}      
            chTcp_Cmd[nCmd_Count++] = 'n';           
        }
        if(NULL != strstr(uart_buffer[nReceive_Count_tail], ">"))
        {
            if(nCmd_Count > 200) { nCmd_Count = 0; nCmd_Count_tail=0;}      
            chTcp_Cmd[nCmd_Count++] = 'g';
        }
        if(NULL != strstr(uart_buffer[nReceive_Count_tail], "SEND OK"))
        {
            if(nCmd_Count > 200) { nCmd_Count = 0; nCmd_Count_tail=0;}      
            chTcp_Cmd[nCmd_Count++] = 'j';
        }
        if(NULL != strstr(uart_buffer[nReceive_Count_tail], "System Ready"))
        {
            if(nCmd_Count > 200) { nCmd_Count = 0; nCmd_Count_tail=0;}      
            chTcp_Cmd[nCmd_Count++] = 'k';
        }

        nReceive_Count_tail++;
      }
    }
    
    osDelay(1);
  }
  /* USER CODE END 5 */ 
}

void TCP_ReceiceData()
{
    if(nCmd_Count_tail <= nCmd_Count)
    {
      if((chTcp_Cmd[nCmd_Count_tail] == 'd'))
      {
              nCmd_Count_tail++;
              if(switchkey==0) // 전송중에는 다시 재전송 하지 말것 
              {                
                switchkey = 1;
                switchkey2 = 0;   
                switchkey_timeout =0;
              }
      }  
      if((chTcp_Cmd[nCmd_Count_tail] == 'e'))
      {
              HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
              ucLight_State = 'O';
              nCmd_Count_tail++;
      }  
      if((chTcp_Cmd[nCmd_Count_tail] == 'f'))
      {
              HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);   
              ucLight_State = 'X';
              nCmd_Count_tail++;
      }  
      if((chTcp_Cmd[nCmd_Count_tail] == 'g'))
      {   
              switchkey2 = 2;
              
              nCmd_Count_tail++;
      }  
      if((chTcp_Cmd[nCmd_Count_tail] == 'j'))
      {
              switchkey2 = 4;
              nCmd_Count_tail++;
      }  
      if((chTcp_Cmd[nCmd_Count_tail] == 'h'))
      {
              switchkey2 = 0;
              switchkey  = 0;
              nCmd_Count_tail++;
      }  
      if((chTcp_Cmd[nCmd_Count_tail] == 'y'))
      {
              switchkey2 = 0;
              switchkey  = 0;
              nCmd_Count_tail++;
      } 
      if((chTcp_Cmd[nCmd_Count_tail] == 'k'))
      {
              switchkey  = 11;
              nCmd_Count_tail++;
      }  
      if((chTcp_Cmd[nCmd_Count_tail] == 'l'))
      {
              ucLink_State = 1;
              nCmd_Count_tail++;
      }  
      if((chTcp_Cmd[nCmd_Count_tail] == 'n'))
      {
              ucLink_State = 0;
              nCmd_Count_tail++;
      }  
    }
}

void ring_buffer_init(void)
{
    ring_buf_lp = 0;
    ring_buf_cp = 0;
    ring_buf_len = 0;
}

u16 ring_buffer_length(void)
{
    return ring_buf_len;
}

void ring_buffer_push(u08 data)
{
    ring_buf[ring_buf_lp++] = data;
    ring_buf_len++;

    if (ring_buf_len > RING_BUF_SIZE) {
        ring_buf_cp++;
        if (ring_buf_cp == RING_BUF_SIZE) {
            ring_buf_cp = 0;
        }
        ring_buf_len = RING_BUF_SIZE;
    }

    if (ring_buf_lp == RING_BUF_SIZE) {
        ring_buf_lp = 0;
    }
}

u08 ring_buffer_pop(void)
{
    u08 ret = 0;

    if (ring_buf_len > 0) {
        if (ring_buf_cp == RING_BUF_SIZE) {
            ring_buf_cp = 0;
        }
        ret = ring_buf[ring_buf_cp++];
        ring_buf_len--;
    }
    return ret;
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
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


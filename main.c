/**
  ******************************************************************************
  * @file    ADC/ADC1_DMA/main.c 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    16-May-2014
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  *   // NB,N fucking B - this only works with a sample time 96 or bigger in the regular channel configuration
  *   //this is something to do with timing i havent quite figured out
  *   
  *   NBNBNB using eclipse template you need to add a reference to the source file (.c) to the objects.mk file
  *   in the USR_OBJS option
  *   
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx.h"
#include "stm32l1xx_conf.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include "mini-printf.h"



/** @addtogroup STM32L1xx_StdPeriph_Examples
  * @{
  */

/** @addtogroup ADC1_DMA
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define ADC1_DR_ADDRESS    ((uint32_t)0x40012458)

// RX = PA10
// TX = PA9

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
ADC_InitTypeDef ADC_InitStructure;
DMA_InitTypeDef DMA_InitStructure;
GPIO_InitTypeDef GPIO_InitStructure;
USART_InitTypeDef usart_init;
USART_ClockInitTypeDef usart_clk_init;

uint16_t ADC_ConvertedValue[3];


/* Private function prototypes -----------------------------------------------*/
void ADC_DMA_Config(void);
void initUart(void);
void usart_write(uint8_t ch);
uint8_t usart_read(void);
uint8_t usart_available(void);
void usart_print( char *msg );
void delay(int a);


/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32l1xx_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32l1xx.c file
     */
  /* ADC1 channel18 configuration using DMA1 channel1 */
  ADC_DMA_Config();
  initUart();

  while (1)
  {

    // OSBM analog reading (old school bullshit method)
    // if(ADC_GetSoftwareStartConvStatus(ADC1) == RESET){

    // uint16_t adcValue = ADC_GetConversionValue(ADC1);

    // } 


	  //FILTERING? KALMAN FILTER?

	      char strDisp[20];

	      usart_print("X axis: ");
	      mini_snprintf( strDisp, 5,  "%d", ADC_ConvertedValue[0]);
	      usart_print(strDisp);

	      usart_print("	y axis: ");
	      	      mini_snprintf( strDisp, 5,  "%d", ADC_ConvertedValue[1]);
	      	      usart_print(strDisp);

	      usart_print("	z axis: ");
	      	      mini_snprintf( strDisp, 5,  "%d", ADC_ConvertedValue[2]);
	      	      usart_print(strDisp);

	      	      usart_print( "\r\n" );

	     delay(100);



    if ( usart_available() ) // data available
      {
        usart_print( "Data Available: " );
        uint8_t ch = usart_read();
        usart_print( "\r\n" );

        usart_print("before");

        char strDisp[20];
        mini_snprintf( strDisp, 5,  "%d", ADC_ConvertedValue[0]);
        usart_print(strDisp);

        usart_print("after");
               usart_print( "\r\n" );
      }

  }
}

/**
  * @brief  Configure the ADC1 channel18 using DMA channel1.
  * @param  None
  * @retval None
  */

void initUart(){

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

        // PA9 = Tx, PA10 = Rx
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

  USART_ClockStructInit(&usart_clk_init);
  USART_ClockInit(USART1, &usart_clk_init);

  usart_init.USART_BaudRate =            9600;
  usart_init.USART_WordLength =          USART_WordLength_8b;
  usart_init.USART_StopBits =            USART_StopBits_1;
  usart_init.USART_Parity =              USART_Parity_No ;
  usart_init.USART_Mode =                USART_Mode_Rx | USART_Mode_Tx;
  usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_Init(USART1, &usart_init);
  USART_Cmd(USART1,ENABLE);

  while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {}

}

void ADC_DMA_Config(void)
{
  /*------------------------ DMA1 configuration ------------------------------*/
  /* Enable DMA1 clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  /* DMA1 channel1 configuration */
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_ADDRESS;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADC_ConvertedValue[0];
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 3;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  
  /* Enable DMA1 channel1 */
  DMA_Cmd(DMA1_Channel1, ENABLE);

  /*----------------- ADC1 configuration with DMA enabled --------------------*/
  /* Enable the HSI oscillator */
  RCC_HSICmd(ENABLE);


  /* Enable GPIOA clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

   /* Configure PA01 + PA02 (ADC Channel1 + 2 respectively) in analog mode */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);


  /* Check that HSI oscillator is ready */
  while(RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET);

  /* Enable ADC1 clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  /* ADC1 configuration */
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 3;
  ADC_Init(ADC1, &ADC_InitStructure);


  /* ADC1 regular channel18 configuration */
  // NB,N fucking B - this only works with a sample time 96 or bigger
  ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_96Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 2, ADC_SampleTime_96Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 3, ADC_SampleTime_96Cycles);


  /* Enable the request after last transfer for DMA Circular mode */
  ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
  
  /* Enable ADC1 DMA */
  ADC_DMACmd(ADC1, ENABLE);
  
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);

  /* Wait until the ADC1 is ready */
  while(ADC_GetFlagStatus(ADC1, ADC_FLAG_ADONS) == RESET)
  {
  }

  /* Start ADC1 Software Conversion */ 
  ADC_SoftwareStartConv(ADC1);
}

void usart_write(uint8_t ch)
{
      USART_SendData(USART1, (uint8_t) ch);
      while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
      {
      }
}

uint8_t usart_read(void){
     while ( USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);
        return (uint8_t)USART_ReceiveData(USART1);
}

uint8_t usart_available(void)
{
  if ( USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET )
    return 1;

  return 0;   
}
 

void usart_print( char *msg )
{
  int len = strlen( msg );

  for ( int c = 0; c < len; c++ )
    usart_write( (uint8_t)*msg++ );
}

void delay( int a )
{
	volatile int i, j;

	for ( i = 0; i < a; i++ )
	{
		j++;
	}
}


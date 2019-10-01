#include <stm32f10x.h>
#include "misc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "delay.h"
#include "usart.h"
#include "twilcd.h"
#include "stdio.h"
#include "math.h"
#include "stdlib.h"








#define SLAVE_ADDRESS_LCD 0x4E
#define SYSCLK 72000000
#define PRESCALER 72
#define SAMPLE 50
#define PWM_PERIOD 50



int avgSum = 0;
char buffer[20];
char buffer2[20];
int TIM_Pulse;
double averaged;

volatile double R;
volatile double celcius ;


 

typedef enum
{
	TOUCH_ERR_NONE,
	TOUCH_ERR_UNKNOWN,
	TOUCH_ERR_HW_ERROR,
	TOUCH_ERR_WRONG_IOCTL_CMD,
}TouchScreenErrorCodes;


TouchScreenErrorCodes adc_init(void);
double getTemp(uint32_t val);







int main(void) {

    I2C1_init();
    lcd_init();
    clearlcd();
    USART_init();
    adc_init();
  
    
    GPIO_InitTypeDef GPIO_InitStructure;
    
    TIM_TimeBaseInitTypeDef timer;
    TIM_OCInitTypeDef timerPWM;

    // enable clock on APB2
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO,  ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_StructInit(&GPIO_InitStructure);

    // configure port A1 for driving an LED
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    // output push-pull mode
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   // highest speed
    GPIO_Init(GPIOB, &GPIO_InitStructure) ;     

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure) ;

    TIM_TimeBaseStructInit(&timer);
    timer.TIM_Prescaler = PRESCALER;
    timer.TIM_Period = SYSCLK / PRESCALER / PWM_PERIOD;
    timer.TIM_ClockDivision = 0;
    timer.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &timer);

    TIM_OCStructInit(&timerPWM);
    timerPWM.TIM_Pulse = 1000;
    timerPWM.TIM_OCMode = TIM_OCMode_PWM1;
    timerPWM.TIM_OutputState = TIM_OutputState_Enable;
    timerPWM.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC4Init(TIM2, &timerPWM);

    TIM_Cmd(TIM2, ENABLE);


            

    // main loop
    while(1) {

   

	 TIM_Pulse = timerPWM.TIM_Pulse;
	
	
    GPIOA->CRL &=0x0; //~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0);
    GPIOA->CRL |= 0x0000A000;//GPIO_CRL_MODE0;

    ADC1->SQR1 = 0x00000000;
    ADC1->SQR3 = (1<<0);
    _delay_ms(10);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);


  
    while ((ADC1->SR & ADC_SR_EOC) == 0){

    }
	
	ADC_SoftwareStartConvCmd(ADC1, DISABLE);
	
   for(int i=0; i<SAMPLE; i++){
     
	 avgSum += ADC1->DR ;

   }

    averaged = avgSum/SAMPLE;
   // temp = getTemp(averaged);
    
    
    R = (100000 / ((4096/averaged)-1));

    USART_SendString(USART1,"DEBUG: conversion started \n\r");

    
    
    celcius = R/100000;
    celcius = log(celcius);
    celcius /= 3950; // 1/B * ln(R/Ro)
    celcius += 1.0 / (25 + 273.15); // + (1/To)
    celcius = 1.0 / celcius; // Invert
    celcius -= 273.15; // convert to C
    
   USART_SendString(USART1,"DEBUG: conversion ended \n\r");


    
    

    
   
    sprintf(buffer,"%d.%ld \r\n",(int)celcius, (uint32_t)((celcius - (int)celcius) *1000000.0));
  
   






  

   USART_SendString(USART1,buffer);
    
    sprintf(buffer2, "adc: %d",avgSum/SAMPLE);
    avgSum =0;
    setpos(0,0);
    str_lcd(buffer);
    setpos(0,1);
    str_lcd(buffer2);
   // USART_SendString(USART1,buffer);
    
    
    

   

   

 

    //TIM2->CCR4=19000;


  
 USART_SendString(USART1,"DEBUG: end of main \n\r");
 


 /*GPIO_SetBits(GPIOB, GPIO_Pin_14);    // turn the LED on
_delay_ms(250);
GPIO_ResetBits(GPIOB, GPIO_Pin_14);  // turn the LED off
_delay_ms(250); */
    }
}


TouchScreenErrorCodes adc_init(void)
{
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
     GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);
    ADC_InitTypeDef ADC_InitStructure;
    RCC->APB2ENR |= 0x00000004; //RCC_APB2ENR_IOPAEN;
   

     /*GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);*/
	RCC_ADCCLKConfig (RCC_PCLK2_Div6);
	
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;	// we work in continuous sampling mode
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 2;
	//ADC_RegularChannelConfig(ADC1,ADC_Channel_1, 1,ADC_SampleTime_1Cycles5);// define regular conversion config
	//ADC_RegularChannelConfig(ADC1,ADC_Channel_2, 1,ADC_SampleTime_1Cycles5);// define regular conversion config
	 // define regular conversion config
	ADC_Init ( ADC1, &ADC_InitStructure);
	ADC_Cmd (ADC1,ENABLE);
	ADC_ResetCalibration(ADC1);	// Reset previous calibration
	while(ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);	// Start new calibration (ADC must be off at that time)
	while(ADC_GetCalibrationStatus(ADC1));
	ADC_Cmd (ADC1,ENABLE);	//enable ADC1
	
	 

  return TOUCH_ERR_NONE;
}



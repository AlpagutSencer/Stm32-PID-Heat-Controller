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
#define PWM_FREQ 1000




char buffer[20];
char buffer2[20];
int TIM_Pulse;
double rem;

double Input;
double Output;
double Setpoint=50;
double errorSum;
double lastErr=0;
double ITerm=0;
double DTerm=0;
double outMin=0;
double outMax=1000;
double kp=2.55;
double ki=0.00010;
double kd=2.3;



 

typedef enum
{
	TOUCH_ERR_NONE,
	TOUCH_ERR_UNKNOWN,
	TOUCH_ERR_HW_ERROR,
	TOUCH_ERR_WRONG_IOCTL_CMD,
}TouchScreenErrorCodes;


TouchScreenErrorCodes adc_init(void);
double getTemp(double alpha);
void tim_init(void);


void TIM4_IRQHandler(void)

{

if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET){ 
  TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
 

Input = getTemp(0.01);

double error = Setpoint - Input;
//errorSum+=error; //error accumulator

//ITerm += (ki * error);

if(Input<Setpoint){ki=0.0002;}
if(Input>Setpoint+0.6){ki=0.0020;}

if((error>-3)&&(error<3))
{
  
  ITerm += (ki * error);
}

DTerm = kd * ((error-lastErr)/1000);




if(ITerm>outMax){ITerm=outMax;}
else if(ITerm<outMin){ITerm=outMin;}

if(Output>outMax){Output=outMax;}
if(Output<outMin){Output=outMin;}

Output = kp*error+ITerm+DTerm;

lastErr = error;

TIM2->CCR4=Output;




}

  /*if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET){
        
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
		GPIOB->ODR ^= GPIO_Pin_14;
   }*/
}





int main(void) {

    I2C1_init();
    lcd_init();
    clearlcd();
    USART_init();
    adc_init();
    tim_init();

  
    
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
    timer.TIM_Prescaler = 71;
    timer.TIM_Period = 999;
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
    
    
    

    rem = getTemp(0.01);
   
    sprintf(buffer,"%d.%ld \r\n",(int)rem, (uint32_t)((rem - (int)rem) *1000000.0));

    sprintf(buffer2,"%d.%ld \r\n",(int)ITerm, (uint32_t)((ITerm - (int)ITerm) *1000000.0));
    USART_SendString(USART1,buffer2);
    USART_SendString(USART1,buffer);
    
    //USART_SendString(USART1,buffer);
    /*setpos(0,0);
    str_lcd(buffer);*/
    
   
    
    //TIM2->CCR4=19000;


  
 
 


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

void tim_init(void)
{

  TIM_TimeBaseInitTypeDef TIMER_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); 

  TIM_TimeBaseStructInit(&TIMER_InitStructure);
  TIMER_InitStructure.TIM_CounterMode = TIM_CounterMode_Up; 
  TIMER_InitStructure.TIM_Prescaler = 1; 
   
  TIMER_InitStructure.TIM_Period = 35999; 
  TIM_TimeBaseInit(TIM4, &TIMER_InitStructure);
  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE); 
  TIM_Cmd(TIM4, ENABLE);
  
  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

}



double getTemp(double alpha)
{

 double arithAveraged;
 int avgSum = 0;
 volatile double R;
 volatile double celcius ;
 int SAMPLE = 70;
 double lastExpAveraged=0;
 double exponentialAveraged=0;

 ADC_SoftwareStartConvCmd(ADC1, ENABLE);

 while ((ADC1->SR & ADC_SR_EOC) == 0){
 }
	
 ADC_SoftwareStartConvCmd(ADC1, DISABLE);
	
 for(int i=0; i<SAMPLE; i++){
     
  avgSum += ADC1->DR ;}

 arithAveraged = avgSum/SAMPLE;

 exponentialAveraged = (1 -alpha) * arithAveraged + alpha * lastExpAveraged;

 lastExpAveraged = exponentialAveraged; 
    
 R = (100000 / ((4096/exponentialAveraged)-1));


    
 celcius = R/100000;
 celcius = log(celcius);
 celcius /= 3950; // 1/B * ln(R/Ro)
 celcius += 1.0 / (25 + 273.15); // + (1/To)
 celcius = 1.0 / celcius; // Invert
 celcius -= 273.15; // convert to C


  return celcius;
}
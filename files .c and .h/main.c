
/**
 *    IAR project for USB HID Device
 *
 *    @author     Juan Domingo Jiménez
*    @email        juajimje@gmail.com
 */
/* Include core modules */
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "defines.h"
#include "stm32f4_usb_hid_device.h"
 #include "tm_stm32f4_delay.h"
#include "tm_stm32f4_disco.h"
#include <stdio.h>
#include <stdlib.h>

__IO uint8_t DemoEnterCondition = 0x00;                 
__IO uint8_t UserButtonPressed = 0x00;                  
                                                        
extern uint8_t tecla1;
extern uint8_t tecla2;

int echo=1;

/*Time*/
extern __IO uint32_t TM_Time; //incrementa en 1 cada ms
__IO uint32_t ms_decimales,cs_decimales, VALo,Tiempo_Actual=0; //variables para obtener el tiempo en decimas de ms

/* Vectores*/
#define MAX 256
uint32_t __IO Tiempos[MAX];
char event_type[MAX][4];
uint32_t event_count[MAX];
uint32_t next_slot=0;
int idx_time=0, idx_event=0; //indices
uint32_t np1i=0, np1f=0, np2i=0,np2f=0, nei=0,nef=0;  //contadores de eventos de cada tipo

char st1,st2,st3,st4,st5,st6;

//******* function prototypes USART *******************
void UART_Initialize(void);                        //**
void GPIOInitialize(void);                         //**
void NVICInitialize(void);                         //**
void USART_puts(   volatile char *s);              //**
                                                   //**
USART_InitTypeDef USART_InitStructure;             //**
GPIO_InitTypeDef GPIO_InitStructure;               //**
extern NVIC_InitTypeDef NVIC_InitStructure;        //**
//*****************************************************

//*****************************PULSOMETRO*****************************************************************
#include "arm_math.h" 
#define TEST_LENGTH_SAMPLES 2048 
static float32_t testOutput[TEST_LENGTH_SAMPLES/2]; 
 uint32_t fftSize = 1024,ifftFlag = 0, doBitReverse = 1, testIndex = 0; ; 
float32_t testInput_f32_10khz[2048],Input_f32_10khz[2048];
#define ADC3_DR_ADDRESS     ((uint32_t)0x4001224C)
__IO uint16_t ADC3ConvertedValue = 0;
__IO uint32_t VR = 0, VIR=0; //voltage
uint32_t tiempo1=0, tiempo2=0,tiempo3=0, ti=0, tf=0 ,Tcalculos=0;
uint8_t flag1=0;
float ppm=0;
void ADC3_CH12_DMA_Config(void);

extern int activar_p;


//********************************************************************************************************
uint32_t dms(void);

int main(void) {
  
/*Initialize LEDs and User_Button on STM32F4-Discovery */
    STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI);                                                            
    STM_EVAL_LEDInit(LED4); STM_EVAL_LEDInit(LED3); STM_EVAL_LEDInit(LED5); STM_EVAL_LEDInit(LED6);        


                
  /* ADC3 configuration *******************************************************/
  /*  - Enable peripheral clocks                                              */
  /*  - DMA2_Stream0 channel2 configuration                                   */
  /*  - Configure ADC Channel12 pin as analog input                           */
  /*  - Configure ADC3 Channel12                                              */
  ADC3_CH12_DMA_Config();
  /* Start ADC3 Software Conversion                                           */ 
  ADC_SoftwareStartConv(ADC3); 
  /****************************************************************************/
  /* GPIOD Periph clock enable */                                           /**/ 
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);                     /**/
  /* Configure PD12, PD13, PD14 and PD15 in output pushpull mode */         /**/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10 ;                    /**/
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;                             /**/
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                            /**/
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;                        /**/
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;                          /**/
  GPIO_Init(GPIOD, &GPIO_InitStructure);                                    /**/
/******************************************************************************/



    

//********************** USART1 ***********************
    GPIOInitialize();                              //**
    UART_Initialize();                             //**
    NVICInitialize();                              //**
//*****************************************************
       
    uint8_t already1 = 0, already2=0,flag = 0; //flags
       
    USB_HIDDEVICE_Keyboard_t Keyboard;  /* Set struct */
    SystemInit();   /* Initialize system */
    TM_DISCO_LedInit(); /* Initialize leds */
    TM_DISCO_ButtonInit(); /* Initialize button */
    
    /*Inicializamos boton externo*/       
    TM_GPIO_Init(GPIOD, GPIO_PIN_14, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_DISCO_BUTTON_PULL, TM_GPIO_Speed_Low);
    /*Inicializamos entrada estimulo*/     
    TM_GPIO_Init(GPIOE, GPIO_PIN_7, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_DISCO_BUTTON_PULL, TM_GPIO_Speed_Low);
    
    TM_DELAY_Init();  /* Initialize delay */
    USB_HIDDEVICE_Init();   /* Initialize USB HID Device */
    USB_HIDDEVICE_KeyboardStructInit(&Keyboard); /* Set default values for keyboard struct */

    USART_puts("introduce orden \n"); //Frase inicial en consola
    
    while (1) {            //==================================WHILE(1)=========================================/
        /* If we are connected and drivers are OK */
        if (USB_HIDDEVICE_GetStatus() == USB_HIDDEVICE_Status_Connected) {
            /* Turn on green LED */
            TM_DISCO_LedOn(LED_GREEN);           
            
/*Boton1*/ if ((TM_GPIO_GetInputPinValue(GPIOD, GPIO_PIN_14)!=0) && already1 == 0) { // Button1 on
                
                already1 = 1;
                Keyboard.Key1 = tecla1;                                           
                USB_HIDDEVICE_KeyboardSend(&Keyboard);// Send keyboard report 
                
                np1i++;//contador evento pulsar boton1
                Tiempos[next_slot]=dms(); 
                event_type[next_slot][0]='P'; 
                event_type[next_slot][1]='1'; 
                event_type[next_slot][2]='I'; 
                event_count[next_slot]=np1i; next_slot++;                
                Delay(20000); // 0.1s
                
            } else if (!TM_GPIO_GetInputPinValue(GPIOD, GPIO_PIN_14) && already1 == 1) { // Button1 release 
                already1 = 0;
               /* Release all buttons*/ 
                Keyboard.L_GUI = USB_HIDDEVICE_Button_Released;    // No button 
                Keyboard.Key1 = 0x00;                                 // No key 
                USB_HIDDEVICE_KeyboardSend(&Keyboard); // Send keyboard report 

                np1f++;
                Tiempos[next_slot]=dms();
                event_type[next_slot][0]='P'; 
                event_type[next_slot][1]='1'; 
                event_type[next_slot][2]='F'; 
                event_count[next_slot]=np1f; next_slot++;
            }
            
            
/*Boton2*/ if (TM_DISCO_ButtonPressed() && already2 == 0) { //Button2 on           
                already2 = 1;
                Keyboard.Key1 = tecla2;                                 
                HIDDEVICE_KeyboardSend(&Keyboard); // Send keyboard report 

                np2i++;
                Tiempos[next_slot]=dms(); 
                event_type[next_slot][0]='P'; 
                event_type[next_slot][1]='2'; 
                event_type[next_slot][2]='I'; 
                event_count[next_slot]=np2i; next_slot++;
                Delay(20000); //0.1s
                
            } else if (!TM_DISCO_ButtonPressed() && already2 == 1) { // Button2 release 
                already2 = 0;
                /* Release all buttons*/ 
                Keyboard.L_GUI = HIDDEVICE_Button_Released;    // No button 
                Keyboard.Key1 = 0x00;                                 // No key 
                USB_HIDDEVICE_KeyboardSend(&Keyboard); // Send keyboard report 
                
                np2f++;
                Tiempos[next_slot]=dms(); 
                event_type[next_slot][0]='P'; 
                event_type[next_slot][1]='2'; 
                event_type[next_slot][2]='F'; 
                event_count[next_slot]=np2f; next_slot++;
            }
/*Estimul*/if (TM_GPIO_GetInputPinValue(GPIOE, GPIO_PIN_7)!=0) { // estimulo activo 
                if(flag==0){
                  TM_DISCO_LedOn(LED_ORANGE);  
                  nei++;
                  Tiempos[next_slot]=dms(); 
                  event_type[next_slot][0]='E'; 
                  event_type[next_slot][1]='I'; 
                  event_count[next_slot]=nei; next_slot++;
                  Delay(5000); //25ms
                  flag=1;
                }
            }   
            else if ((TM_GPIO_GetInputPinValue(GPIOE, GPIO_PIN_7)==0)&& flag == 1){
                  
                  nef++;
                  Tiempos[next_slot]=dms(); 
                  event_type[next_slot][0]='E'; 
                  event_type[next_slot][1]='F'; 
                  event_count[next_slot]=nef; next_slot++;
                  TM_DISCO_LedOff(LED_ORANGE);
                  Delay(5000);//25ms
                  flag=0;
            }
            } else {
                  /* Turn off green LED */
                  TM_DISCO_LedOff(LED_GREEN);
            }
/*pulsomet*/if(activar_p==1){  /********************PULSOMETRO*******/
                   GPIO_SetBits(GPIOD, GPIO_Pin_9);
                  if(flag1==0){ 
                    tiempo1=TM_Time;  //solo la primera vez
                    while(TM_Time-tiempo1<100);  //quitar rebote sensor
                    //  while(TM_Time-tiempo1<3) /* convert the ADC value (from 0 to 0xFFF --4095 decimal) to a voltage value (from 0V to 3.3V)*/
                    for( int n=0; n<2048; n++){ 
                        testInput_f32_10khz[n] = ADC3ConvertedValue *3300/0xFFF;  //PC2 
                        tiempo2=TM_Time;
                        while(TM_Time-tiempo2<1);
                    }
                    flag1=1;
                  } 
                    ti=dms();//Tiempo inicio calculos
                    for(int j=0;j<2038;j++){ //Guardamos todas las muestras menos las 10 primeras antes de machacar el vector
                      Input_f32_10khz[j]=testInput_f32_10khz[j+10];
                    }
                  //  /* Disable ADC3 DMA */  ADC_DMACmd(ADC3, DISABLE);
                  //  /* Disable ADC3 */      ADC_Cmd(ADC3, DISABLE);                     
                    arm_status status; 
                    arm_cfft_radix4_instance_f32 S; 
                    float32_t maxValue; 
                    status = ARM_MATH_SUCCESS; 
                    /* Initialize the CFFT/CIFFT module */  
                    status = arm_cfft_radix4_init_f32(&S, fftSize, ifftFlag, doBitReverse); 
                    /* Process the data through the CFFT/CIFFT module */ 
                    arm_cfft_radix4_f32(&S, testInput_f32_10khz); 
                    /* Process the data through the Complex Magnitude Module for  
                    calculating the magnitude at each bin */ 
                    arm_cmplx_mag_f32(testInput_f32_10khz, testOutput, fftSize); 
                    testOutput[0]=0;
                    /* Calculates maxValue and returns corresponding BIN value */ 
                    arm_max_f32(testOutput, fftSize, &maxValue, &testIndex); 
                    
                    ppm = (100.0f/2048.0f)*(1024.0f-testIndex)*60.0f;
                                            
 
                    for(int q=0;q<2048;q++){ //recuperamos vector entero
                      testInput_f32_10khz[q]=Input_f32_10khz[q];
                    }
                   // /* Enable ADC3 DMA */  ADC_DMACmd(ADC3, ENABLE);
                   // /* Enable ADC3 */      ADC_Cmd(ADC3, ENABLE);
                    tf=dms();
                    Tcalculos= ti -tf;
                    for( int k=2038; k<2048; k++){   //añadimos 10 muestras actuales
                      testInput_f32_10khz[k] = ADC3ConvertedValue *3300/0xFFF;  //PC2 
                      tiempo2=TM_Time;
                      while(TM_Time-tiempo2<1);
                    }
              }
    } //End while(1)
} //End Main

/*Funcion que retorna decimas de milisegundo*/
uint32_t dms(void)
{
       VALo=SysTick->VAL;  //Valor actual del SysTick
       cs_decimales=(uint32_t)(VALo*99/168000);    //entre 0 y 99 decimas de s^-2
       ms_decimales=(TM_Time*100)+(cs_decimales);
       return ms_decimales;
}

//**********************************************FUNCIONES UART1****************************************************************************
void UART_Initialize(void)
{
 /* Enable peripheral clock for USART1 */
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
 
/* USART1 configured as follow:
 * BaudRate 9600 baud
 * Word Length 8 Bits
 * 1 Stop Bit
 * No parity
 * Hardware flow control disabled
 * Receive and transmit enabled
 */
 USART_InitStructure.USART_BaudRate = 9600;
 USART_InitStructure.USART_WordLength = USART_WordLength_8b;
 USART_InitStructure.USART_StopBits = USART_StopBits_1;
 USART_InitStructure.USART_Parity = USART_Parity_No;
 USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
 USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
 
USART_Init(USART1, &USART_InitStructure); // USART configuration
 USART_Cmd(USART1, ENABLE); // Enable USART
}
 
void GPIOInitialize(void)
{
 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); //Enable clock for GPIOB
 
/* USART1 Tx on PB6 | Rx on PB7 */
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
 
GPIO_Init(GPIOB, &GPIO_InitStructure);
 GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);//Connect PB6 to USART1_Tx
 GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);//Connect PB7 to USART1_Rx
}
 
void NVICInitialize(void)
{
 
 NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
 
NVIC_Init(&NVIC_InitStructure);
 USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);    //habilita las interrupciones USART
 
}


void USART_puts(volatile char *s){
  if(echo!=48){  // El 0 de la consola se detecta como 48
    // wait until data register is empty
    while(*s){
      while( !(USART1->SR & 0x00000040) );
      USART_SendData(USART1, *s);
      *s++;
    }
  } 
  return;
} //*****************************************************FIN FUNCIONES UART1***************************

/***********************************PULSOMETRO********************************************************/
void ADC3_CH12_DMA_Config(void)
{
  ADC_InitTypeDef       ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  DMA_InitTypeDef       DMA_InitStructure;
  GPIO_InitTypeDef      GPIO_InitStructure;

  /* Enable ADC3, DMA2 and GPIO clocks ****************************************/
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);

  /* DMA2 Stream0 channel0 configuration **************************************/
  DMA_InitStructure.DMA_Channel = DMA_Channel_2;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC3_DR_ADDRESS;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC3ConvertedValue;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = 1;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream0, &DMA_InitStructure);
  DMA_Cmd(DMA2_Stream0, ENABLE);

  /* Configure ADC3 Channel12 pin as analog input ******************************/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* ADC Common Init **********************************************************/
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);

  /* ADC3 Init ****************************************************************/
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC3, &ADC_InitStructure);

  /* ADC3 regular channel12 configuration *************************************/
  ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 1, ADC_SampleTime_3Cycles);

 /* Enable DMA request after last transfer (Single-ADC mode) */
  ADC_DMARequestAfterLastTransferCmd(ADC3, ENABLE);

  /* Enable ADC3 DMA */
  ADC_DMACmd(ADC3, ENABLE);

  /* Enable ADC3 */
  ADC_Cmd(ADC3, ENABLE);
}
/********************************************************************************************************************/ 

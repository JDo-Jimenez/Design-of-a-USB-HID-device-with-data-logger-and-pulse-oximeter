/*******************************************************************************
  * @file    stm32f4xx_it.c
  * @author  Juan Domingo Jiménez Jerez
  * @date    14-Mayo-2015
  * @brief   Main Interrupt Service Routines.
  *          This file provides all exceptions handler and peripherals interrupt
  *          service routine.
  *****************************************************************************/ 
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
//#include "main.h"
#include "usb_core.h"
#include "usbd_core.h"
#include "stm32f4_discovery.h"
#include "usbd_hid_core.h"   
#include "stm32f4xx_conf.h"
#include "usb_bsp.h"
#include "tm_stm32f4_delay.h"
#include "core_cm4.h"
//#include "tm_stm32f4_delay.c"  //contiene systick
#include <stdio.h>
#include <stdlib.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define CURSOR_STEP     7

extern uint8_t Buffer[6];
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern __IO uint8_t DemoEnterCondition;
extern __IO uint8_t UserButtonPressed;

uint32_t TiempoA;
/*----------------------------------------------------------------------------*/

extern uint32_t __IO Tiempos[];
extern char event_type[][4];
extern uint32_t next_slot;
extern uint32_t event_count[];

/* Private function prototypes -----------------------------------------------*/
extern USB_OTG_CORE_HANDLE           USB_OTG_dev;
extern uint32_t USBD_OTG_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);

void ejecuta_orden(volatile char *linea);
void codificador_hex1(volatile char *linea_hex1, int j);
void codificador_hex2(volatile char *linea_hex2, int jj);
void codificador1(char letra1);
void codificador2(char letra2);

extern uint32_t dms(void);
//extern void USART_puts(USART_TypeDef *USARTx, volatile int8_t tecla); //usart

uint8_t tecla1 = 0;
uint8_t tecla2 = 0;

uint8_t ajuste=0; //ajuste de tiempo

 char ultimo_valor_tecla1[3];
 char ultimo_valor_tecla2[3];
 extern int echo;//habilitado por defecto
 char strHEX1[2];
 char strHEX2[2];
 
 /************************************PULSOMETRO***********************************************/
 int activar_p=0;
 extern float ppm;
 
 
 //**************************************FLASH****************************************************
#define FLASH_USER_START_ADDR   ADDR_FLASH_SECTOR_2   /* Start @ of user Flash area*/         //**
#define FLASH_USER_END_ADDR     ADDR_FLASH_SECTOR_5   /* End @ of user Flash area*/           //**
/* Base address of the Flash sectors*/                                                        //**
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbytes*/     //**
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) /* Base @ of Sector 1, 16 Kbytes*/     //**
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) /* Base @ of Sector 2, 16 Kbytes*/     //**
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* Base @ of Sector 3, 16 Kbytes*/     //**
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbytes*/     //**
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) /* Base @ of Sector 5, 128 Kbytes*/    //**
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) /* Base @ of Sector 6, 128 Kbytes*/    //**
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) /* Base @ of Sector 7, 128 Kbytes*/    //**
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000) /* Base @ of Sector 8, 128 Kbytes*/    //**
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000) /* Base @ of Sector 9, 128 Kbytes*/    //**
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000) /* Base @ of Sector 10, 128 Kbytes*/   //**
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000) /* Base @ of Sector 11, 128 Kbytes*/   //**
                                                                                              //**
uint32_t StartSector = 0, EndSector = 0, Address = 0, i = 0 ;                                 //**
uint32_t GetSector(uint32_t Address);                                                         //**
//************************************************************************************************


/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

#define MAX_WORDLEN 10
volatile char received_str[MAX_WORDLEN + 1];
extern void USART_puts(   volatile char *s);  

void USART1_IRQHandler(void){   /// USART2 Interrupt request handler for ALL (Tx/Rx) USART2 interrupts
  
  if( USART_GetITStatus(USART1, USART_IT_RXNE)){  // Check the Interrupt status to ensure the Rx interrupt was triggered, not Tx
      static int cnt = 0;
    // Get the byte that was transferred
    char ch = USART1->DR;

         // Check for "Enter" key, or Maximum characters
       if((ch != '\n') && (cnt < MAX_WORDLEN)){
          received_str[cnt++] = ch;
        }
        else{
          received_str[cnt] = '\0';
          cnt = 0;
          USART_puts(received_str); 
        ejecuta_orden(received_str);      
          
        }       
  }

}

   
void ejecuta_orden(volatile char *linea)
{
  int i=0;
  if(linea[0]=='s'){   //orden setkey
    while(linea[i]!=' ') i++;  
    if(linea[i-1]=='1'){  //letra antes del espacio ¿tecla1?
      if((linea[i+1]=='0')&&(linea[i+2]=='x')) codificador_hex1(linea,i); //hexadecimal?
      else { codificador1(linea[i+1]);}  //enviamos la siguiente letra despues del espacio
      
    }else if(linea[i-1]=='2'){ //¿tecla2?
      if((linea[i+1]=='0')&&(linea[i+2]=='x')) codificador_hex2(linea,i); //hexadecimal
      else {codificador2(linea[i+1]);}  //enviamos la siguiente letra despues del espacio
    }else{
      USART_puts("orden setkey incorrecta\n introduzca setkeyX donde X es 1 o 2 \n");
    }
    return;
  }
  if(linea[0]=='g') { //orden getkey
    USART_puts("el valor de la tecla 1 es: ");
    USART_puts(ultimo_valor_tecla1);
    USART_puts("el valor de la tecla 2 es: ");
    USART_puts(ultimo_valor_tecla2);
    USART_puts("\n");
  }  
  if(linea[0]=='e') { //orden echo

    
    while(linea[i]!=' ') i++;  
    int echo_anterior=echo;
    echo=49; //para que siempre muestre por pantalla si esta deshabilitado o habilitado; luego se cambia al correspondiente
    if(linea[i+1]==48){ USART_puts("echo deshabilitado \n"); // el 0 de la consola aquí es 48
        echo=linea[i+1];  }
    else if(linea[i+1]==49){ USART_puts("echo habilitado \n");// El 1 de la consola es 49
        echo=linea[i+1];   }
    else {USART_puts("echo incorrecto \n");
        echo=echo_anterior;}
    

  }
    if(linea[0]=='h') { //orden set time  COJE HORA Y ENVIA ACK, el ajuste lo hace el otro programa
    volatile char nueva_linea[MAX_WORDLEN + 1]="vacia";
    while(linea[i]!=' ') i++;  
    for (int f=i+1;f<=MAX_WORDLEN; f++){ 
      nueva_linea[f-i-1]=linea[f];  // almacena todo lo que hay despues del espacio (hora en 0.1ms)
    }  
    uint32_t hora;
    char str11[15];
    sprintf(str11, "%s", nueva_linea);
    hora = atoi(str11);  // ahora hora es un entero
    /*Enviar ACK*/
    USART_puts("ACK"); 
    return;
  }  
  if(linea[0]=='t'){   //tabla de tiempos 'time table'

     TiempoA=dms();
     char str2[15];
     sprintf(str2, "%d", TiempoA);
     USART_puts(str2);
     USART_puts(" ms/10 \n");
     
     char str3[15],str4[15],str5[15];
     for(int i=0;i<next_slot;i++){
       
       sprintf(str3, "%d", Tiempos[i]);  //Pasamos de int32 a string 'int to string';
       sprintf(str5, "%d", event_count[i]);  //Pasamos de int32 a string;
       USART_puts(str3);
       strcpy(str4, "ms/10 -- ");
       strcat(str4, event_type[i]);
       strcat(str4, str5);
       strcat(str4, "\n");
       USART_puts(str4);
     }
     
  }
  if(linea[0]=='w'){ //Guardar datos en FLASH 'write flash'
    STM_EVAL_LEDOn(LED3); STM_EVAL_LEDOn(LED4); STM_EVAL_LEDOn(LED5); STM_EVAL_LEDOn(LED6);
     Address = FLASH_USER_START_ADDR;
     FLASH_Unlock();  // Unlock the Flash to enable the flash control register access ************* 
        /* Erase the user Flash area
                (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/
     
      FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |  /* Clear pending flags (if any) */
                  FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR); 

       StartSector = GetSector(FLASH_USER_START_ADDR);  /* Get the number of the start and end sectors */
       EndSector = GetSector(FLASH_USER_END_ADDR);
                
      for (i = StartSector; i < EndSector; i += 8)
      {
        
    // Device voltage range supposed to be [2.7V to 3.6V], the operation will be done by word 
            if (FLASH_EraseSector(i, VoltageRange_3) != FLASH_COMPLETE)
            { 
              // Error occurred while sector erase. User can add here some code to deal with this error  
              while (1)
              {
              }
             }
        
      }
       Address = FLASH_USER_START_ADDR;
      uint8_t sstrtoint;
      uint32_t add;
      FLASH_ProgramWord(Address, next_slot); // Guarda Primero el Ultimo slot
       Address = Address + 4;
       for(int i=0;i<next_slot;i++){
         FLASH_ProgramWord(Address, Tiempos[i]);
          Address = Address + 4;
         FLASH_ProgramWord(Address, event_count[i]);
          Address = Address + 4;
          
        // sstrtoint=strtol(event_type[i], NULL, 36);
          
         //sstrtoint = strtol(event_type[i][0], NULL, 16);
         sstrtoint= (int)event_type[i][0];
         FLASH_ProgramByte(Address, sstrtoint);
          Address = Address + 1;
          
        sstrtoint= (int)event_type[i][1];
         FLASH_ProgramByte(Address, sstrtoint);
          Address = Address + 1;
          
         sstrtoint= (int)event_type[i][2];
         FLASH_ProgramByte(Address, sstrtoint);
          Address = Address + 1;
          
          sstrtoint= (int)event_type[i][3];
         FLASH_ProgramByte(Address, sstrtoint);
          Address = Address + 1;
          
          add=sizeof(event_type[i]);
       }
       
      FLASH_Lock();  // Lock the Flash to disable the flash control register access (recommended to protect the FLASH memory against possible unwanted operation) ********
      STM_EVAL_LEDOff(LED3); STM_EVAL_LEDOff(LED4); STM_EVAL_LEDOn(LED5); STM_EVAL_LEDOff(LED6);
  }
  if(linea[0]=='c'){ //Borrar datos en FLASH 'clear flash'
   
  }
  if(linea[0]=='r'){ //Leer datos en FLASH 'read flash'
      Address = FLASH_USER_START_ADDR;
      next_slot = *(__IO uint32_t*)Address;  //primero leemos next_slot
      Address = Address + 4;
      //char str90[15];
      //int int90;
     // uint8_t int80;
      for(int i=0;i<next_slot;i++){
          Tiempos[i] = *(__IO uint32_t*)Address;
           Address = Address + 4;
          event_count[i] = *(__IO uint32_t*)Address;
           Address = Address + 4;
          event_type[i][0] = *(char*)Address;
          //sprintf(event_type[i], "%d", int80);  //Pasamos de int a string; 
          //event_type[i]= str90;
           Address = Address + 1;
           event_type[i][1] = *(char*)Address;
           Address = Address + 1;
           event_type[i][2] = *(char*)Address;
           Address = Address + 1;
           event_type[i][3] = *(char*)Address;
           Address = Address + 1;
      }  
       
           
            char str3[15],str4[15],str5[15];
     for(int i=0;i<next_slot;i++){
       
       sprintf(str3, "%d", Tiempos[i]);  //Pasamos de int32 a string 'int to string';
       sprintf(str5, "%d", event_count[i]);  //Pasamos de int32 a string;
       USART_puts(str3);
       strcpy(str4, "ms/10 -- ");
       strcat(str4, event_type[i]);
       strcat(str4, str5);
       strcat(str4, "\n");
       USART_puts(str4);
     }    
      
  }
  if(linea[0]=='a'){ //activar pulsometro
   activar_p=1;
  }
    if(linea[0]=='p'){ //pulso
      if(activar_p==0){
        USART_puts("Pulsometro inactivo, activar con orden 'a' \n");
      }
      else{
        char str3[15];
        sprintf(str3, "%f", ppm);  //Pasamos de float32 a string 'int to string';
        strcat(str3,"ppm");
        strcat(str3, "\n");
        USART_puts(str3);
      }
      }

   return;
}    /********************************************** CODIFICADORES ***************************************************************/
void codificador_hex1(volatile char *linea_hex1, int j){
  strHEX1[0]=NULL; strHEX1[1]=NULL; strHEX2[0]=NULL;
  strHEX1[0]=linea_hex1[j+3];
  strHEX2[0]=linea_hex1[j+4];
      strcat(strHEX1, strHEX2);
      //t = atoi(l);
      tecla1=strtol(strHEX1, NULL, 16);
}

void codificador_hex2(volatile char *linea_hex2, int jj){
  strHEX1[0]=NULL; strHEX1[1]=NULL; strHEX2[0]=NULL;
  strHEX1[0]=linea_hex2[jj+3];
  strHEX2[0]=linea_hex2[jj+4];
      strcat(strHEX1, strHEX2);
      //t = atoi(l);
      tecla2=strtol(strHEX1, NULL, 16);
}
void codificador1(char letra1){
  ultimo_valor_tecla1[0]=letra1;
  ultimo_valor_tecla1[1]='\n';
   
  if(letra1=='a')tecla1=0x04;
  if(letra1=='b')tecla1=0x05;
  if(letra1=='c')tecla1=0x06;
  if(letra1=='d')tecla1=0x07;
  if(letra1=='e')tecla1=0x08;
  if(letra1=='f')tecla1=0x09;
  if(letra1=='g')tecla1=0x0A;
  if(letra1=='h')tecla1=0x0B;
  if(letra1=='i')tecla1=0x0C;
  if(letra1=='j')tecla1=0x0D;
  if(letra1=='k')tecla1=0x0E;
  if(letra1=='l')tecla1=0x0F;
  if(letra1=='m')tecla1=0x10;
  if(letra1=='m')tecla1=0x11;
  if(letra1=='o')tecla1=0x12;
  if(letra1=='p')tecla1=0x13;
  if(letra1=='q')tecla1=0x14;
  if(letra1=='r')tecla1=0x15;
  if(letra1=='s')tecla1=0x16;
  if(letra1=='t')tecla1=0x17;
  if(letra1=='u')tecla1=0x18;
  if(letra1=='v')tecla1=0x19;
  if(letra1=='w')tecla1=0x1A;
  if(letra1=='x')tecla1=0x1B;
  if(letra1=='y')tecla1=0x1C;
  if(letra1=='z')tecla1=0x1D;
  if(letra1=='1')tecla1=0x1E;
  if(letra1=='2')tecla1=0x1F;
  if(letra1=='3')tecla1=0x20;
  if(letra1=='4')tecla1=0x21;
  if(letra1=='5')tecla1=0x22;
  if(letra1=='6')tecla1=0x23;
  if(letra1=='7')tecla1=0x24;
  if(letra1=='8')tecla1=0x25;
  if(letra1=='9')tecla1=0x26;
  if(letra1=='0')tecla1=0x27;
  if(letra1==' ')tecla1=0x2C;
  
  return;
}
void codificador2(char letra2){
  ultimo_valor_tecla2[0]=letra2;
  ultimo_valor_tecla2[1]='\n';
  
  if(letra2=='a')tecla2=0x04;
  if(letra2=='b')tecla2=0x05;
  if(letra2=='c')tecla2=0x06;
  if(letra2=='d')tecla2=0x07;
  if(letra2=='e')tecla2=0x08;
  if(letra2=='f')tecla2=0x09;
  if(letra2=='g')tecla2=0x0A;
  if(letra2=='h')tecla2=0x0B;
  if(letra2=='i')tecla2=0x0C;
  if(letra2=='j')tecla2=0x0D;
  if(letra2=='k')tecla2=0x0E;
  if(letra2=='l')tecla2=0x0F;
  if(letra2=='m')tecla2=0x10;
  if(letra2=='m')tecla2=0x11;
  if(letra2=='o')tecla2=0x12;
  if(letra2=='p')tecla2=0x13;
  if(letra2=='q')tecla2=0x14;
  if(letra2=='r')tecla2=0x15;
  if(letra2=='s')tecla2=0x16;
  if(letra2=='t')tecla2=0x17;
  if(letra2=='u')tecla2=0x18;
  if(letra2=='v')tecla2=0x19;
  if(letra2=='w')tecla2=0x1A;
  if(letra2=='x')tecla2=0x1B;
  if(letra2=='y')tecla2=0x1C;
  if(letra2=='z')tecla2=0x1D;
  if(letra2=='1')tecla2=0x1E;
  if(letra2=='2')tecla2=0x1F;
  if(letra2=='3')tecla2=0x20;
  if(letra2=='4')tecla2=0x21;
  if(letra2=='5')tecla2=0x22;
  if(letra2=='6')tecla2=0x23;
  if(letra2=='7')tecla2=0x24;
  if(letra2=='8')tecla2=0x25;
  if(letra2=='9')tecla2=0x26;
  if(letra2=='0')tecla2=0x27;  
  if(letra2==' ')tecla2=0x2C;
 // else{tecla2=letra2;}
  return;
}  

void EXTI0_IRQHandler(void)
{
  UserButtonPressed = 0x01;
  
  /* Clear the EXTI line pending bit */
  EXTI_ClearITPendingBit(USER_BUTTON_EXTI_LINE);
}
uint32_t GetSector(uint32_t Address)  //**********flash***********************************************************
{
  uint32_t sector = 0;
  
  if((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
  {
    sector = FLASH_Sector_0;  
  }
  else if((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
  {
    sector = FLASH_Sector_1;  
  }
  else if((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
  {
    sector = FLASH_Sector_2;  
  }
  else if((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
  {
    sector = FLASH_Sector_3;  
  }
  else if((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
  {
    sector = FLASH_Sector_4;  
  }
  else if((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
  {
    sector = FLASH_Sector_5;  
  }
  else if((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
  {
    sector = FLASH_Sector_6;  
  }
  else if((Address < ADDR_FLASH_SECTOR_8) && (Address >= ADDR_FLASH_SECTOR_7))
  {
    sector = FLASH_Sector_7;  
  }
  else if((Address < ADDR_FLASH_SECTOR_9) && (Address >= ADDR_FLASH_SECTOR_8))
  {
    sector = FLASH_Sector_8;  
  }
  else if((Address < ADDR_FLASH_SECTOR_10) && (Address >= ADDR_FLASH_SECTOR_9))
  {
    sector = FLASH_Sector_9;  
  }
  else if((Address < ADDR_FLASH_SECTOR_11) && (Address >= ADDR_FLASH_SECTOR_10))
  {
    sector = FLASH_Sector_10;  
  }
  else/*(Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_11))*/
  {
    sector = FLASH_Sector_11;  
  }

  return sector;
}
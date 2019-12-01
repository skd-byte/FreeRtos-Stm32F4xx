/*******************************************************************************
* Title                 :   UART Configuration and CLI terminal interface routine
* Filename              :   uart_cli.c
*******************************************************************************/

/******************************************************************************
* Includes
*******************************************************************************/
#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "stm32f4xx.h"
#include "uart_cli.h"

/******************************************************************************
* Module Preprocessor Constants
*******************************************************************************/

/******************************************************************************
* Module Preprocessor Macros
*******************************************************************************/



/******************************************************************************
* Module Typedefs
*******************************************************************************/

/******************************************************************************
* Module Variable Definitions
*******************************************************************************/


/******************************************************************************
* Function Prototypes
*******************************************************************************/

/******************************************************************************
* Function Definitions
*******************************************************************************/
/******************************************************************************
* Function :
*//**
* @brief
* @param  none
* @retval none
* @note
*/
void UART_CLIInit()
{
  UART_GPIOInit();                              // initialize UART2 GPIO
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;          // UART clock Enable

  USART2->CR1 &= (uint32_t)~USART_CR1_UE;        // disable UART1
  USART2->CR1 &= (uint32_t)~USART_CR1_M;         // 8-bit data length
  USART2->CR1 &= (uint32_t)~USART_CR1_PCE;       // None Parity
  USART2->CR2 &= (uint32_t)~USART_CR2_STOP;
  USART2->BRR =   138;                           // set baudrate to 115200 =>320000/1152 == 277.77
  USART2->CR1 =  (uint32_t)USART_CR1_UE;         // Enable UART2

}

/*************** END OF FUNCTIONS ***************************************************************************/
/******************************************************************************
* Function :
*//**
* @brief
* @param  none
* @retval none
* @note
*/
void UART_GPIOInit()
{
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

  //PA2 UART2_tx
  GPIOA->MODER   &= (uint32_t)~GPIO_MODER_MODER2;
  GPIOA->MODER   |=  GPIO_MODER_MODER2_1;
  GPIOA->OTYPER  &= (uint32_t)~GPIO_OTYPER_OT_2;
  GPIOA->PUPDR   &= (uint32_t)~GPIO_PUPDR_PUPDR2;
  GPIOA->PUPDR   |= GPIO_PUPDR_PUPDR2_0;
  GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR2;
  GPIOA->AFR[0]  |= 0x00000700;

  //PA3_UART_rx
  GPIOA->MODER   &= (uint32_t)~GPIO_MODER_MODER3;
  GPIOA->MODER   |=  GPIO_MODER_MODER3_1;
  GPIOA->OTYPER  &= (uint32_t)~GPIO_OTYPER_OT_3;
  GPIOA->PUPDR   &= (uint32_t)~GPIO_PUPDR_PUPDR3;
  GPIOA->PUPDR   |= GPIO_PUPDR_PUPDR3_0;
  GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR3;
  GPIOA->AFR[0]  |= 0x000007000;

}

/*************** END OF FUNCTIONS ***************************************************************************/
/******************************************************************************
* Function :
*//**
* @brief
* @param  none
* @retval none
* @note
*/
char Buff[256];
void CLI_Write(unsigned char *pcFormat, ...)
{
    char *pcBuff = &Buff[0];
    va_list list;
    int iSize = 256;
    int iRet;

    memset(pcBuff, 0, 252);

    while(1)
    {
        va_start(list,pcFormat);
        iRet = vsnprintf(pcBuff,iSize,(char const *)pcFormat,list);
        va_end(list);
        if(iRet > -1 && iRet < iSize)
        {
            UART_CLIWrite((uint8_t*)pcBuff, strlen((const char *)pcBuff));
            break;
        }
        else
        {
          UART_Message("Memory not sufficient");
          break;    //NOTE:check this
        }
    }
}
/*************** END OF FUNCTIONS ***************************************************************************/

/******************************************************************************
* Function :
*//**
* @brief  Outputs a character string to the console
* @param  none
* @retval none
* @note
*/
void UART_Message(const char *str)
{
  UART_CLIWrite((uint8_t*)str, strlen(str));
}
/*************** END OF FUNCTIONS ***************************************************************************/
/******************************************************************************
* Function :
*//**
* @brief
* @param  none
* @retval none
* @note
*/
void UART_CLIWrite(uint8_t *u8_Buff, uint32_t u32_size)
{
  uint32_t u32_index = 0;
  uint8_t  u8_tcFlag = RESET;
  USART2->CR1 |= (uint32_t)USART_CR1_UE;        //Enable UART2
  USART2->CR1 |= (uint32_t)USART_CR1_TE;        //Enable Transmit

  while(u32_size > (uint32_t)0)
  {
    if ((USART2->SR & USART_SR_TXE) == USART_SR_TXE)
    {
      USART2->DR = (uint8_t)u8_Buff[u32_index++];
      u32_size--;
    }
  }

  while(u8_tcFlag != SET)
  {
    if ((USART2->SR & USART_SR_TC) == USART_SR_TC)
    {
      USART2->SR &= ~USART_SR_TC; //Clear transfer complete flag
      u8_tcFlag = SET;
    }
  }
  USART2->CR1 &= (uint32_t)~USART_CR1_TE;    //Disable Tx
}
/*************** END OF FUNCTIONS ***************************************************************************/

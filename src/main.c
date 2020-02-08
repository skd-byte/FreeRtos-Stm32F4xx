#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "stm32f4xx.h"
#include "uart_cli.h"
#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"


/*
 * Tasks
 */
portTASK_FUNCTION_PROTO(vTask, pvParameters);

static const char *TaskFirstInsatnceMsg  = "Task1 is running\r\n";
static const char *TaskSecondInsatnceMsg = "Task2 is running\r\n";

#define mainDELAY_LOOP_COUNT   40000


/*
 * define GPIO Board LEDs
 */
#define RED         (1U<<5) //D13-PA5, Nucleo Board Led
#define RED_BIT     (1U<<10)
#define GPIOA_CLOCK (1<<0)

void Toggle_LED(void);
void GPIO_Init(void);


int main(int argc, char* argv[])
{
  UART_CLIInit();
  GPIO_Init();
  while (1)
  {


     xTaskCreate(vTask, "Task1", 150, (void*)TaskFirstInsatnceMsg, 1, NULL);
     xTaskCreate(vTask, "Task2", 150, (void*)TaskSecondInsatnceMsg, 1, NULL);

     /* Start the scheduler. */
     vTaskStartScheduler();

     /* Should never be reached */
     for( ;; );
   }
}




 void vApplicationTickHook( void )
 {

   //trace_printf("Entered vApplicationTickHook\n");

 }

 void vApplicationIdleHook( void ){

   trace_printf("Entered vApplicationIdleHook\n");

 }

 void vApplicationMallocFailedHook( void ){

   trace_printf("Entered vApplicationMallocFailedHook\n");
   for(;;);
 }

 void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName){

   ( void )pxTask;
   ( void )pcTaskName;

   for(;;);
 }

 /*
  * Tasks
  */

 void vTask( void *pvParameters)
 {
   volatile unsigned long ul = 0;
   while(1)
   {
     trace_printf((char*)pvParameters);
     /* Delay for a period. */
     for( ul = 0; ul < mainDELAY_LOOP_COUNT; ul++ )
     {
     }
  }
}


void GPIO_Init(void)
{
  RCC->AHB1ENR |= GPIOA_CLOCK;
  GPIOA->MODER |= RED_BIT;
}

void Toggle_LED(void)
{
  GPIOA->ODR ^= RED;
}


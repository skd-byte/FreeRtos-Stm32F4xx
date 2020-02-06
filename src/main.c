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
portTASK_FUNCTION_PROTO(vTask1, pvParameters);
portTASK_FUNCTION_PROTO(vTask2, pvParameters);

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


     xTaskCreate(vTask1, "Task1", configMINIMAL_STACK_SIZE, (void * ) NULL, tskIDLE_PRIORITY+1UL, NULL);
     xTaskCreate(vTask2, "Task2", configMINIMAL_STACK_SIZE, (void * ) NULL, tskIDLE_PRIORITY+1UL, NULL);

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

 void vTask1( void *pvParameters)
 {
   volatile unsigned long ul = 0;
   while(1)
   {
     trace_printf("Task1 is running\n");
     /* Delay for a period. */
     for( ul = 0; ul < mainDELAY_LOOP_COUNT; ul++ )
     {
     }
  }
}

 void vTask2( void *pvParameters)
 {
   volatile unsigned long ul = 0;
   while(1)
   {
     trace_printf("Task2 is running\n");

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


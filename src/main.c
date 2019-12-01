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
portTASK_FUNCTION_PROTO(vApplicationTaskTest, pvParameters);
portTASK_FUNCTION_PROTO(vApplicationTaskTest2, pvParameters);

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


     xTaskCreate(vApplicationTaskTest, "TestTask", configMINIMAL_STACK_SIZE, (void * ) NULL, tskIDLE_PRIORITY+1UL, NULL);
     xTaskCreate(vApplicationTaskTest2, "TestTask2", configMINIMAL_STACK_SIZE, (void * ) NULL, tskIDLE_PRIORITY+1UL, NULL);

     /* Start the scheduler. */
     vTaskStartScheduler();

     /* Should never be reached */
     for( ;; );
     }
 }




 void vApplicationTickHook( void ){

   trace_printf("Entered vApplicationTickHook\n");

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

 void vApplicationTaskTest( void *pvParameters){

   while(1){
     trace_printf("In Task vApplicationTaskTest\n");
   }
 }

 void vApplicationTaskTest2( void *pvParameters){

   while(1){
     trace_printf("\tIn Task vApplicationTaskTest2\n");
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


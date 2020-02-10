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


     xTaskCreate(vTask,     "Task1", 150, (void*)TaskFirstInsatnceMsg, 1, NULL);
     xTaskCreate(vTask,     "Task2", 150, (void*)TaskSecondInsatnceMsg, 2, NULL);
     /* Start the scheduler. */
     vTaskStartScheduler();

     /* Should never be reached */
     for( ;; );
   }
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
   char *pcTaskName;
   TickType_t xLastWakeTime;
   /* The string to print out is passed in via the parameter. Cast this to a
   character pointer. */
   pcTaskName = ( char * ) pvParameters;
   /* The xLastWakeTime variable needs to be initialized with the current tick
   count. Note that this is the only time the variable is written to explicitly.
   After this xLastWakeTime is automatically updated within vTaskDelayUntil(). */
   xLastWakeTime = xTaskGetTickCount();
   /* As per most tasks, this task is implemented in an infinite loop. */
   for( ;; )
   {
   /* Print out the name of this task. */
     trace_printf( pcTaskName );
   /* This task should execute every 250 milliseconds exactly. As per
   the vTaskDelay() function, time is measured in ticks, and the
   pdMS_TO_TICKS() macro is used to convert milliseconds into ticks.
   xLastWakeTime is automatically updated within vTaskDelayUntil(), so is not
   explicitly updated by the task. */
   vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 250 ) );
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


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
portTASK_FUNCTION_PROTO(vContinuousProcessingTask, pvParameters);
portTASK_FUNCTION_PROTO(vPeriodicTask, pvParameters);

static const char *TaskFirstInsatnceMsg  = "Continuous Task1 is running\n";
static const char *TaskSecondInsatnceMsg = "Continuous Task2 is running\n";
const char *pcTextForPeriodicTask        = "Periodic Task is running\n";

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
     xTaskCreate(vContinuousProcessingTask, "Task1", 150, (void*)TaskFirstInsatnceMsg,  1, NULL);
     xTaskCreate(vContinuousProcessingTask, "Task2", 150, (void*)TaskSecondInsatnceMsg, 1, NULL);

     xTaskCreate( vPeriodicTask, "Task 3", 1000, (void*)pcTextForPeriodicTask, 2, NULL );
     /* Start the scheduler. */
     vTaskStartScheduler();

     /* Should never be reached */
     for( ;; );
   }
}


void vApplicationMallocFailedHook( void )
{
   trace_printf("Entered vApplicationMallocFailedHook\n");
   for(;;);
}

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName)
{
   ( void )pxTask;
   ( void )pcTaskName;

   for(;;);
}

/*
 * Tasks
 */
void vContinuousProcessingTask( void *pvParameters )
{
  char *pcTaskName;

   /* The string to print out is passed in via the parameter.  Cast this to a
   character pointer. */
   pcTaskName = ( char * ) pvParameters;

   /* As per most tasks, this task is implemented in an infinite loop. */
   for( ;; )
   {
     /* Print out the name of this task.  This task just does this repeatedly
     without ever blocking or delaying. */
     trace_printf( pcTaskName );
   }
}

uint32_t count = 5;

void vPeriodicTask( void *pvParameters )
{
  TickType_t xLastWakeTime;
  const TickType_t xDelay5ms = pdMS_TO_TICKS( 5UL );

   /* The xLastWakeTime variable needs to be initialized with the current tick
   count.  Note that this is the only time we access this variable.  From this
   point on xLastWakeTime is managed automatically by the vTaskDelayUntil()
   API function. */
   xLastWakeTime = xTaskGetTickCount();

   /* As per most tasks, this task is implemented in an infinite loop. */
   for( ;; )
   {
     /* Print out the name of this task. */
     trace_printf( "Periodic Task is running\n");

     /* We want this task to execute exactly every 5 milliseconds. */
     vTaskDelayUntil( &xLastWakeTime, xDelay5ms );
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


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

/* Used to hold the handle of Task2. */
TaskHandle_t xTask2Handle;

static uint32_t ulIdleCycleCount = 0UL;

static const char *Task1Msg  = "Task1 is running\n";
static const char *Task2Msg  = "Task2 is running\n";


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
  xTaskCreate( vTask1, "Task 1", 1000, NULL, 2, NULL );
          /* The task is created at priority 2 ^. */

  /* Create the second task at priority 1 - which is lower than the priority
  given to Task1.  Again the task parameter is not used so is set to NULL -
  BUT this time we want to obtain a handle to the task so pass in the address
  of the xTask2Handle variable. */
  xTaskCreate( vTask2, "Task 2", 1000, NULL, 1, &xTask2Handle );
         /* The task handle is the last parameter ^^^^^^^^^^^^^ */

  /* Start the scheduler to start the tasks executing. */
  vTaskStartScheduler();

  /* The following line should never be reached because vTaskStartScheduler()
  will only return if there was not enough FreeRTOS heap memory available to
  create the Idle and (if configured) Timer tasks.  Heap management, and
  techniques for trapping heap exhaustion, are described in the book text. */
  for( ;; );
  return 0;
}
/*-----------------------------------------------------------*/

void vTask1( void *pvParameters )
{
  UBaseType_t uxPriority;

  /* This task will always run before Task2 as it has the higher priority.
  Neither Task1 nor Task2 ever block so both will always be in either the
  Running or the Ready state.

  Query the priority at which this task is running - passing in NULL means
  "return our own priority". */
  uxPriority = uxTaskPriorityGet( NULL );

  for( ;; )
  {
    /* Print out the name of this task. */
    trace_printf( "Task1 is running\r\n" );

    /* Setting the Task2 priority above the Task1 priority will cause
    Task2 to immediately start running (as then Task2 will have the higher
    priority of the    two created tasks). */
    trace_printf( "About to raise the Task2 priority\r\n" );
    vTaskPrioritySet( xTask2Handle, ( uxPriority + 1 ) );

    /* Task1 will only run when it has a priority higher than Task2.
    Therefore, for this task to reach this point Task2 must already have
    executed and set its priority back down to 0. */
  }
}

  /*-----------------------------------------------------------*/

void vTask2( void *pvParameters )
{
  UBaseType_t uxPriority;

  /* Task1 will always run before this task as Task1 has the higher priority.
  Neither Task1 nor Task2 ever block so will always be in either the
  Running or the Ready state.

  Query the priority at which this task is running - passing in NULL means
  "return our own priority". */
  uxPriority = uxTaskPriorityGet( NULL );

  for( ;; )
  {
    /* For this task to reach this point Task1 must have already run and
    set the priority of this task higher than its own.

    Print out the name of this task. */
    trace_printf( "Task2 is running\r\n" );

    /* Set our priority back down to its original value.  Passing in NULL
    as the task handle means "change our own priority".  Setting the
    priority below that of Task1 will cause Task1 to immediately start
    running again. */
    trace_printf( "About to lower the Task2 priority\r\n" );
    vTaskPrioritySet( NULL, ( uxPriority - 2 ) );
  }
}

/* Idle hook functions MUST be called vApplicationIdleHook(), take no parameters,
and return void. */
void vApplicationIdleHook( void )
{
  /* This hook function does nothing but increment a counter. */
  ulIdleCycleCount++;
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



void GPIO_Init(void)
{
  RCC->AHB1ENR |= GPIOA_CLOCK;
  GPIOA->MODER |= RED_BIT;
}

void Toggle_LED(void)
{
  GPIOA->ODR ^= RED;
}


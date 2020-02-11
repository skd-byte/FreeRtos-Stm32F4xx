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
  xTaskCreate( vTask1, "Task 1", 1000, NULL, 1, NULL );
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
  const TickType_t xDelay100ms = pdMS_TO_TICKS( 100UL );
  for( ;; )
  {
    /* Print out the name of this task. */
    trace_printf( "Task 1 is running\r\n" );
    /* Create task 2 at a higher priority. Again the task parameter is not
    used so is set to NULL - BUT this time the task handle is required so
    the address of xTask2Handle is passed as the last parameter. */
    xTaskCreate( vTask2, "Task 2", 1000, NULL, 2, &xTask2Handle );
    /* The task handle is the last parameter _____^^^^^^^^^^^^^ */
    /* Task 2 has/had the higher priority, so for Task 1 to reach here Task 2
    must have already executed and deleted itself. Delay for 100
    milliseconds. */
    vTaskDelay( xDelay100ms );
  }
}

  /*-----------------------------------------------------------*/

void vTask2( void *pvParameters )
{
  /* Task 2 does nothing but delete itself. To do this it could call vTaskDelete()
  using NULL as the parameter, but instead, and purely for demonstration purposes,
  it calls vTaskDelete() passing its own task handle. */
  trace_printf( "Task 2 is running and about to delete itself\r\n" );
  vTaskDelete( xTask2Handle );
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


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

static uint32_t ulIdleCycleCount = 0UL;

static const char *TaskFirstInsatnceMsg  = "Task1 is running";
static const char *TaskSecondInsatnceMsg = "Task2 is running";


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
     xTaskCreate(vTask, "Task1", 150, (void*)TaskFirstInsatnceMsg,  1, NULL);
     xTaskCreate(vTask, "Task2", 150, (void*)TaskSecondInsatnceMsg, 2, NULL);

     /* Start the scheduler. */
     vTaskStartScheduler();

     /* Should never be reached */
     for( ;; );
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

/*
 * Tasks
 */
void vTask( void *pvParameters )
{
  char *pcTaskName;
  const TickType_t xDelay250ms = pdMS_TO_TICKS( 250UL );

    /* The string to print out is passed in via the parameter.  Cast this to a
    character pointer. */
    pcTaskName = ( char * ) pvParameters;

    /* As per most tasks, this task is implemented in an infinite loop. */
    for( ;; )
    {
      /* Print out the name of this task AND the number of times ulIdleCycleCount
          has been incremented. */
      trace_printf("%s\nIdleCount= %d\n", pcTaskName, ulIdleCycleCount );

      /* Delay for a period.  This time we use a call to vTaskDelay() which
      puts the task into the Blocked state until the delay period has expired.
      The delay period is specified in 'ticks'. */
      vTaskDelay( xDelay250ms );
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


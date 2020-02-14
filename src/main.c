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
portTASK_FUNCTION_PROTO(vTaskSender, pvParameters);
portTASK_FUNCTION_PROTO(vTaskReceiver, pvParameters);

/* Used to hold the handle of Task2. */
TaskHandle_t xTask2Handle;

static uint32_t ulIdleCycleCount = 0UL;

/*
 * define GPIO Board LEDs
 */
#define RED         (1U<<5) //D13-PA5, Nucleo Board Led
#define RED_BIT     (1U<<10)
#define GPIOA_CLOCK (1<<0)

void Toggle_LED(void);
void GPIO_Init(void);

/* Declare a variable of type QueueHandle_t to hold the handle of the queue being created. */
QueueHandle_t xQueue;


int main(int argc, char* argv[])
{
  UART_CLIInit();
  GPIO_Init();

  xQueue = xQueueCreate( 5, sizeof( char * ) );

  if(xQueue != NULL)
    {
      xTaskCreate( vTaskSender, "Sender1", 1000, NULL, 2, NULL );

      xTaskCreate( vTaskReceiver, "Receiver",1000, NULL,1 ,NULL);

      /* Start the scheduler so the created tasks start executing. */
      vTaskStartScheduler();
    }

  for( ;; );
  return 0;
}


void vTaskSender( void *pvParameters )
{
  char *pcStringToSend;
  const size_t xMaxStringLength = 50;
  BaseType_t xStringNumber = 0;

  /* As per most tasks, this task is implemented within an infinite loop. */
  for( ;; )
  {
    pcStringToSend = ( char * ) malloc( xMaxStringLength );
    /* Write a string into the buffer. */
    snprintf( pcStringToSend, xMaxStringLength, "String number %d\r\n", xStringNumber );
    /* Increment the counter so the string is different on each iteration of this task. */
    xStringNumber++;

    xQueueSend( xQueue, /* The handle of the queue. */
               &pcStringToSend, /* The address of the pointer that points to the buffer. */
               portMAX_DELAY );
  }
}



void vTaskReceiver( void *pvParameters )
{
  char *pcReceivedString;
  for( ;; )
  {
    /* Receive the address of a buffer. */
    xQueueReceive( xQueue, /* The handle of the queue. */
    &pcReceivedString, /* Store the buffer’s address in pcReceivedString. */
    portMAX_DELAY );
    /* The buffer holds a string, print it out. */
    trace_printf(pcReceivedString );
    /* The buffer is not required any more - release it so it can be freed, or re-used. */
    free( pcReceivedString );
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

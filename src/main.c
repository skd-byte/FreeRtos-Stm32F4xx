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

/* Declare a variable of type QueueHandle_t. This is used to store the handle
 to the queue that is accessed by all three tasks. */
 QueueHandle_t xQueue;

 typedef enum
 {
   Sender1,
   Sender2,
 }eDataId;

 typedef struct
 {
   eDataId ID;
   int32_t val;
 }Data_t;

 Data_t data[2] = {
                    {Sender1, 100},
                    {Sender2, 200},
                  };

int main(int argc, char* argv[])
{
  UART_CLIInit();
  GPIO_Init();

  xQueue = xQueueCreate( 3, sizeof( Data_t ) );


  if(xQueue != NULL)
  {
    xTaskCreate( vTaskSender, "Sender1", 1000,(void *) &data[0], 2, NULL );
    xTaskCreate( vTaskSender, "Sender2", 1000,(void *) &data[1], 2, NULL );

    xTaskCreate( vTaskReceiver, "Receiver",1000, NULL,1 ,NULL);

    /* Start the scheduler so the created tasks start executing. */
    vTaskStartScheduler();
  }
  else
  {
    // Queue could not be creaated
  }

  for( ;; );
  return 0;
}


void vTaskSender( void *pvParameters )
{
  BaseType_t xStatus;
  const TickType_t xTicksToWait = pdMS_TO_TICKS( 100 );
  /* As per most tasks, this task is implemented within an infinite loop. */
  for( ;; )
  {


    xStatus = xQueueSendToBack( xQueue, pvParameters, xTicksToWait );
    if( xStatus != pdPASS )
    {
      /* The send operation could not complete because the queue was full -
      this must be an error as the queue should never contain more than
      one item! */
      trace_printf( "Could not send to the queue.\r\n" );
    }
  }
}



void vTaskReceiver( void *pvParameters )
{
  Data_t xReceivedStructure;
  BaseType_t xStatus;
  /* This task is also defined within an infinite loop. */
  for( ;; )
  {
    if( uxQueueMessagesWaiting( xQueue ) != 3 )
    {
      trace_printf( "Queue should be full!\r\n" );
    }

    xStatus = xQueueReceive( xQueue, &xReceivedStructure, 0 );
    if( xStatus == pdPASS )
    {
      /* Data was successfully received from the queue, print out the received
      value and the source of the value. */
      if( xReceivedStructure.ID == Sender1 )
      {
        trace_printf( "From Sender 1 = %d\n", xReceivedStructure.val );
      }
      else
      {
        trace_printf( "From Sender 2 = %d\n", xReceivedStructure.val );
      }
    }
    else
    {
      /* Data was not received from the queue even after waiting for 100ms.
      This must be an error as the sending tasks are free running and will be
      continuously writing to the queue. */
      trace_printf( "Could not receive from the queue.\r\n" );
    }
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


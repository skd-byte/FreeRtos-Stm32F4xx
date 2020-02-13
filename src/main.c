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

/* Declare a variable of type QueueHandle_t. This is used to store the handle
 to the queue that is accessed by all three tasks. */
 QueueHandle_t xQueue;

int main(int argc, char* argv[])
{
  UART_CLIInit();
  GPIO_Init();

  xQueue = xQueueCreate( 5, sizeof( int32_t ) );

  if(xQueue != NULL)
  {
    xTaskCreate( vTaskSender, "Sender1", 1000,(void *) 100, 1, NULL );
    xTaskCreate( vTaskSender, "Sender2", 1000,(void *) 200, 1, NULL );

    xTaskCreate( vTaskReceiver, "Receiver",1000, NULL,2 ,NULL);

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
  int32_t lValueToSend;
  BaseType_t xStatus;
  /* Two instances of this task are created so the value that is sent to the
  queue is passed in via the task parameter - this way each instance can use
  a different value. The queue was created to hold values of type int32_t,
  so cast the parameter to the required type. */
  lValueToSend = ( int32_t ) pvParameters;
  /* As per most tasks, this task is implemented within an infinite loop. */
  for( ;; )
  {
    /* Send the value to the queue.
    The first parameter is the queue to which data is being sent. The
    queue was created before the scheduler was started, so before this task
    started to execute.
    The second parameter is the address of the data to be sent, in this case
    the address of lValueToSend.
    The third parameter is the Block time – the time the task should be kept
    in the Blocked state to wait for space to become available on the queue
    should the queue already be full. In this case a block time is not
    specified because the queue should never contain more than one item, and
    therefore never be full. */
    xStatus = xQueueSendToBack( xQueue, &lValueToSend, 0 );
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
  /* Declare the variable that will hold the values received from the queue. */
  int32_t lReceivedValue;
  BaseType_t xStatus;
  const TickType_t xTicksToWait = pdMS_TO_TICKS( 100 );
  /* This task is also defined within an infinite loop. */
  for( ;; )
  {
    /* This call should always find the queue empty because this task will
    immediately remove any data that is written to the queue. */
    if( uxQueueMessagesWaiting( xQueue ) != 0 )
    {
      trace_printf( "Queue should have been empty!\r\n" );
    }
    /* Receive data from the queue.
    The first parameter is the queue from which data is to be received. The
    queue is created before the scheduler is started, and therefore before this
    task runs for the first time.
    The second parameter is the buffer into which the received data will be
    placed. In this case the buffer is simply the address of a variable that
    has the required size to hold the received data.
    The last parameter is the block time – the maximum amount of time that the
    task will remain in the Blocked state to wait for data to be available
    should the queue already be empty. */
    xStatus = xQueueReceive( xQueue, &lReceivedValue, xTicksToWait );
    if( xStatus == pdPASS )
    {
      /* Data was successfully received from the queue, print out the received
      value. */
      trace_printf( "Received = %d\n", lReceivedValue );
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


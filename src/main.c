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
portTASK_FUNCTION_PROTO(vSenderTask1, pvParameters);
portTASK_FUNCTION_PROTO(vSenderTask2, pvParameters);
portTASK_FUNCTION_PROTO(vReceiverTask, pvParameters);

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

/* Declare two variables of type QueueHandle_t. Both queues are added to the same queue set. */
static QueueHandle_t xQueue1 = NULL, xQueue2 = NULL;
/* Declare a variable of type QueueSetHandle_t. This is the queue set to which the two queues are added. */
static QueueSetHandle_t xQueueSet = NULL;


int main(int argc, char* argv[])
{
  UART_CLIInit();
  GPIO_Init();

  /* Create the two queues, both of which send character pointers. The priority
  of the receiving task is above the priority of the sending tasks, so the queues
  will never have more than one item in them at any one time*/
  xQueue1 = xQueueCreate( 1, sizeof( char * ) );
  xQueue2 = xQueueCreate( 1, sizeof( char * ) );
  /* Create the queue set. Two queues will be added to the set, each of which can
  contain 1 item, so the maximum number of queue handles the queue set will ever
  have to hold at one time is 2 (2 queues multiplied by 1 item per queue). */
  xQueueSet = xQueueCreateSet( 1 * 2 );
  /* Add the two queues to the set. */
  xQueueAddToSet( xQueue1, xQueueSet );
  xQueueAddToSet( xQueue2, xQueueSet );
  /* Create the tasks that send to the queues. */
  xTaskCreate( vSenderTask1, "Sender1", 1000, NULL, 1, NULL );
  xTaskCreate( vSenderTask2, "Sender2", 1000, NULL, 1, NULL );
  /* Create the task that reads from the queue set to determine which of the two
  queues contain data. */
  xTaskCreate( vReceiverTask, "Receiver", 1000, NULL, 2, NULL );
  /* Start the scheduler so the created tasks start executing. */
  vTaskStartScheduler();
  /* As normal, vTaskStartScheduler() should not return, so the following lines
  Will never execute. */
  for( ;; );
  return 0;
}

void vSenderTask2( void *pvParameters )
{
  const TickType_t xBlockTime = pdMS_TO_TICKS( 200 );
  const char * const pcMessage = "Message from vSenderTask2\r\n";
  /* As per most tasks, this task is implemented within an infinite loop. */
  for( ;; )
  {
  /* Block for 200ms. */
  vTaskDelay( xBlockTime );
  /* Send this task's string to xQueue2. It is not necessary to use a block
  time, even though the queue can only hold one item. This is because the
  priority of the task that reads from the queue is higher than the priority of
  this task; as soon as this task writes to the queue it will be pre-empted by
  the task that reads from the queue, so the queue will already be empty again
  by the time the call to xQueueSend() returns. The block time is set to 0. */
  xQueueSend( xQueue2, &pcMessage, 0 );
  }
}

void vSenderTask1( void *pvParameters )
{
  const TickType_t xBlockTime = pdMS_TO_TICKS( 100 );
  const char * const pcMessage = "Message from vSenderTask1\r\n";
  /* As per most tasks, this task is implemented within an infinite loop. */
  for( ;; )
  {
    /* Block for 100ms. */
    vTaskDelay( xBlockTime );
    /* Send this task's string to xQueue1. It is not necessary to use a block
    time, even though the queue can only hold one item. This is because the
    priority of the task that reads from the queue is higher than the priority of
    this task; as soon as this task writes to the queue it will be pre-empted by
    the task that reads from the queue, so the queue will already be empty again
    by the time the call to xQueueSend() returns. The block time is set to 0. */
    xQueueSend( xQueue1, &pcMessage, 0 );
  }
}



void vReceiverTask( void *pvParameters )
{
  QueueHandle_t xQueueThatContainsData;
  char *pcReceivedString;
  /* As per most tasks, this task is implemented within an infinite loop. */
  for( ;; )
  {
    /* Block on the queue set to wait for one of the queues in the set to contain data.
    Cast the QueueSetMemberHandle_t value returned from xQueueSelectFromSet() to a
    QueueHandle_t, as it is known all the members of the set are queues (the queue set
    does not contain any semaphores). */
    xQueueThatContainsData = ( QueueHandle_t ) xQueueSelectFromSet( xQueueSet,
    portMAX_DELAY );
    /* An indefinite block time was used when reading from the queue set, so
    xQueueSelectFromSet() will not have returned unless one of the queues in the set
    contained data, and xQueueThatContainsData cannot be NULL. Read from the queue. It
    is not necessary to specify a block time because it is known the queue contains
    data. The block time is set to 0. */
    xQueueReceive( xQueueThatContainsData, &pcReceivedString, 0 );
    /* Print the string received from the queue. */
    trace_printf( pcReceivedString );
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

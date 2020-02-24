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

#define GPIO_PIN_13        ((uint16_t)0x0200DU)  /* Pin 0 selected    */
#define MCU_IRQ_PIN         GPIO_PIN_13
#define MCU_IRQ_PORT        GPIOC
#define SW_INTERRUPT()     (EXTI->SWIER |= EXTI_SWIER_SWIER13)           // software interrupt generated

void EXTILine13_Host_Config();
void ExternalInterrupt_callback();




/*
 * define GPIO Board LEDs
 */
#define RED         (1U<<5) //D13-PA5, Nucleo Board Led
#define RED_BIT     (1U<<10)
#define GPIOA_CLOCK (1<<0)

void Toggle_LED(void);
void GPIO_Init(void);

portTASK_FUNCTION_PROTO(vStringPrinter, pvParameters);
portTASK_FUNCTION_PROTO(vIntegerGenerator, pvParameters);

QueueHandle_t xIntegerQueue, xStringQueue;

int main( void )
{
  UART_CLIInit();
  GPIO_Init();
  EXTILine13_Host_Config();

  /* Before a queue can be used it must first be created. Create both queues used
  by this example. One queue can hold variables of type uint32_t, the other queue
  can hold variables of type char*. Both queues can hold a maximum of 10 items. A
  real application should check the return values to ensure the queues have been
  successfully created. */
  xIntegerQueue = xQueueCreate( 10, sizeof( uint32_t ) );
  xStringQueue  = xQueueCreate( 10, sizeof( char * ) );

  /* Create the task that uses a queue to pass integers to the interrupt service
  routine. The task is created at priority 1. */
  xTaskCreate( vIntegerGenerator, "IntGen", 1000, NULL, 1, NULL );

  /* Create the task that prints out the strings sent to it from the interrupt
  service routine. This task is created at the higher priority of 2. */
  xTaskCreate( vStringPrinter, "String", 1000, NULL, 2, NULL );


  /* Start the scheduler so the created tasks start executing. */
  vTaskStartScheduler();

  /* If all is well then main() will never reach here as the scheduler will now be
  running the tasks. If main() does reach here then it is likely that there was
  insufficient heap memory available for the idle task to be created. Chapter 2
  provides more information on heap memory management. */

  for( ;; );
}


void vIntegerGenerator( void *pvParameters )
{
  TickType_t xLastExecutionTime;
  uint32_t ulValueToSend = 0;
  int i;
  /* Initialize the variable used by the call to vTaskDelayUntil(). */
  xLastExecutionTime = xTaskGetTickCount();
  for( ;; )
  {
    /* This is a periodic task. Block until it is time to run again. The task
    will execute every 200ms. */
    vTaskDelayUntil( &xLastExecutionTime, pdMS_TO_TICKS( 200 ) );
    /* Send five numbers to the queue, each value one higher than the previous
    value. The numbers are read from the queue by the interrupt service routine.
    The interrupt service routine always empties the queue, so this task is
    guaranteed to be able to write all five values without needing to specify a
    block time. */
    for( i = 0; i < 5; i++ )
    {
      xQueueSendToBack( xIntegerQueue, &ulValueToSend, 0 );
      ulValueToSend++;
    }

    trace_printf( "Generator task - About to generate an interrupt.\r\n" );
    SW_INTERRUPT();
    trace_printf( "Generator task - Interrupt generated.\r\n\r\n\r\n" );
  }
}

void vStringPrinter( void *pvParameters )
{
  char *pcString;
  for( ;; )
  {
    /* Block on the queue to wait for data to arrive. */
    xQueueReceive( xStringQueue, &pcString, portMAX_DELAY );
    /* Print out the string received. */
    trace_printf( pcString );
  }
}

/* external interrupt callback function */
void ExternalInterrupt_callback()
{
  Toggle_LED();
  BaseType_t xHigherPriorityTaskWoken;
  uint32_t ulReceivedNumber;
  /* The strings are declared static const to ensure they are not allocated on the
  interrupt service routine's stack, and so exist even when the interrupt service
  routine is not executing. */
  static const char *pcStrings[] =
  {
    "String 0\r\n",
    "String 1\r\n",
    "String 2\r\n",
    "String 3\r\n"
  };
  /* As always, xHigherPriorityTaskWoken is initialized to pdFALSE to be able to
  detect it getting set to pdTRUE inside an interrupt safe API function. Note that
  as an interrupt safe API function can only set xHigherPriorityTaskWoken to
  pdTRUE, it is safe to use the same xHigherPriorityTaskWoken variable in both
  the call to xQueueReceiveFromISR() and the call to xQueueSendToBackFromISR(). */
  xHigherPriorityTaskWoken = pdFALSE;
  /* Read from the queue until the queue is empty. */
  while( xQueueReceiveFromISR( xIntegerQueue, &ulReceivedNumber, &xHigherPriorityTaskWoken )
         != errQUEUE_EMPTY )
  {
    /* Truncate the received value to the last two bits (values 0 to 3
    inclusive), then use the truncated value as an index into the pcStrings[]
    array to select a string (char *) to send on the other queue. */
    ulReceivedNumber &= 0x03;
    xQueueSendToBackFromISR( xStringQueue,
    &pcStrings[ ulReceivedNumber ],
    &xHigherPriorityTaskWoken );
  }
  /* If receiving from xIntegerQueue caused a task to leave the Blocked state, and
  if the priority of the task that left the Blocked state is higher than the
  priority of the task in the Running state, then xHigherPriorityTaskWoken will
  have been set to pdTRUE inside xQueueReceiveFromISR().
  If sending to xStringQueue caused a task to leave the Blocked state, and if the
  priority of the task that left the Blocked state is higher than the priority of
  the task in the Running state, then xHigherPriorityTaskWoken will have been set
  to pdTRUE inside xQueueSendToBackFromISR().
  xHigherPriorityTaskWoken is used as the parameter to portYIELD_FROM_ISR(). If
  xHigherPriorityTaskWoken equals pdTRUE then calling portYIELD_FROM_ISR() will
  request a context switch. If xHigherPriorityTaskWoken is still pdFALSE then
  calling portYIELD_FROM_ISR() will have no effect.*/
  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

 void vApplicationMallocFailedHook( void )
 {

   trace_printf("Entered vApplicationMallocFailedHook\n");
   for(;;);
 }

 void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName){

   ( void )pxTask;
   ( void )pcTaskName;

   for(;;);
 }



/* initialize board LED */
void GPIO_Init(void)
{
  RCC->AHB1ENR |= GPIOA_CLOCK;
  GPIOA->MODER |= RED_BIT;
}


/* Board Led Toggle Function */
void Toggle_LED(void)
{
  GPIOA->ODR ^= RED;
}


void EXTILine13_Host_Config()
{

  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
  RCC->AHB1ENR           |= RCC_AHB1ENR_GPIOCEN;               // enable clock
  MCU_IRQ_PORT->MODER   &= (uint32_t)~(GPIO_MODER_MODER13);    // input mode

  SYSCFG->EXTICR[3] = SYSCFG_EXTICR4_EXTI13_PC ;
  EXTI->IMR  |= EXTI_IMR_MR13;          // unmasked the interrupt
  EXTI->EMR  |= EXTI_IMR_MR13;          // unmasked the interrupt
  EXTI->FTSR |= EXTI_FTSR_TR13;         // rising edge tringger on line 13

  NVIC_SetPriority(EXTI15_10_IRQn, 7);  // Enable and set EXTI Line0 Interrupt to the lowest priority
  // NOTE: This interrupt priority shoul be less than configMAX_SYSCALL_INTERRUPT_PRIORITY
  NVIC_EnableIRQ(EXTI15_10_IRQn);
}


void EXTI15_10_IRQHandler ()
{
  if((EXTI->PR & EXTI_PR_PR13) != RESET)
  {
    EXTI->PR = EXTI_PR_PR13;
    ExternalInterrupt_callback();
  }
}


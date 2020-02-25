#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "stm32f4xx.h"
#include "uart_cli.h"
#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"
#include "event_groups.h"
/*
 * Tasks
 */

#define GPIO_PIN_13        ((uint16_t)0x0200DU)  /* Pin 0 selected    */
#define MCU_IRQ_PIN         GPIO_PIN_13
#define MCU_IRQ_PORT        GPIOC
#define SW_INTERRUPT()     (EXTI->SWIER |= EXTI_SWIER_SWIER13)           // software interrupt generated

/*
 * define GPIO Board LEDs
 */
#define RED         (1U<<5) //D13-PA5, Nucleo Board Led
#define RED_BIT     (1U<<10)
#define GPIOA_CLOCK (1<<0)

void Toggle_LED(void);
void GPIO_Init(void);

void ExternalInterrupt_callback();
void EXTILine13_Host_Config();

portTASK_FUNCTION_PROTO(vEventBitSettingTask, pvParameters);
portTASK_FUNCTION_PROTO(vEventBitReadingTask, pvParameters);
portTASK_FUNCTION_PROTO(vInterruptGenerator, pvParameters);

/* Definitions for the event bits in the event group. */
#define mainFIRST_TASK_BIT ( 1UL << 0UL )  /* Event bit 0, which is set by a task. */
#define mainSECOND_TASK_BIT ( 1UL << 1UL ) /* Event bit 1, which is set by a task. */
#define mainISR_BIT ( 1UL << 2UL )         /* Event bit 2, which is set by an ISR. */

void vPrintStringFromDaemonTask( void *pvParameter1, uint32_t ulParameter2 );

EventGroupHandle_t xEventGroup;

int main( void )
{
  UART_CLIInit();
  GPIO_Init();
  EXTILine13_Host_Config();

  /* Before an event group can be used it must first be created. */
  xEventGroup = xEventGroupCreate();

  /* Create the task that sets event bits in the event group. */
  xTaskCreate( vEventBitSettingTask, "Bit Setter", 1000, NULL, 1, NULL );

  /* Create the task that waits for event bits to get set in the event group. */
  xTaskCreate( vEventBitReadingTask, "Bit Reader", 1000, NULL, 2, NULL );

  /* Create the task that is used to periodically generate a software interrupt. */
  xTaskCreate( vInterruptGenerator, "Int Gen", 1000, NULL, 3, NULL );

  vTaskStartScheduler();

  /* The following line should never be reached. */
  for( ;; );
  return 0;
}

void vEventBitReadingTask( void *pvParameters )
{
  EventBits_t xEventGroupValue;
  const EventBits_t xBitsToWaitFor = ( mainFIRST_TASK_BIT |
  mainSECOND_TASK_BIT | mainISR_BIT );

  for( ;; )
  {
    /* Block to wait for event bits to become set within the event group. */
    xEventGroupValue = xEventGroupWaitBits( /* The event group to read. */
    xEventGroup,
    /* Bits to test. */
    xBitsToWaitFor,
    /* Clear bits on exit if the
    unblock condition is met. */
    pdTRUE,
    /*This parameter is set to pdTRUE for the
    second execution. */
    pdTRUE,
    /* Don't time out. */
    portMAX_DELAY );
    /* Print a message for each bit that was set. */
    if( ( xEventGroupValue & mainFIRST_TASK_BIT ) != 0 )
    {
      trace_printf( "Bit reading task -\t Event bit 0 was set\n" );
    }
    if( ( xEventGroupValue & mainSECOND_TASK_BIT ) != 0 )
    {
      trace_printf( "Bit reading task -\t Event bit 1 was set\n" );
    }
    if( ( xEventGroupValue & mainISR_BIT ) != 0 )
    {
      trace_printf( "Bit reading task -\t Event bit 2 was set\n" );
    }
  }
}


void vInterruptGenerator( void *pvParameters )
{
  TickType_t xLastExecutionTime;

  /* Initialize the variable used by the call to vTaskDelayUntil(). */
  xLastExecutionTime = xTaskGetTickCount();
  for( ;; )
  {
    /* This is a periodic task. Block until it is time to run again. The task
    will execute every 200ms. */
    vTaskDelayUntil( &xLastExecutionTime, pdMS_TO_TICKS( 200 ) );

    trace_printf( "Generator task - About to generate an interrupt.\n" );
    SW_INTERRUPT();
    trace_printf( "Generator task - Interrupt generated.\n" );
  }
}

void vEventBitSettingTask( void *pvParameters )
{
  const TickType_t xDelay200ms = pdMS_TO_TICKS( 200UL ), xDontBlock = 0;
  for( ;; )
  {
    /* Delay for a short while before starting the next loop. */
    vTaskDelay( xDelay200ms );
    /* Print out a message to say event bit 0 is about to be set by the task,
    then set event bit 0. */
    trace_printf( "Bit setting task -\t about to set bit 0.\r\n" );
    xEventGroupSetBits( xEventGroup, mainFIRST_TASK_BIT );
    /* Delay for a short while before setting the other bit. */
    vTaskDelay( xDelay200ms );
    /* Print out a message to say event bit 1 is about to be set by the task,
    then set event bit 1. */
    trace_printf( "Bit setting task -\t about to set bit 1.\r\n" );
    xEventGroupSetBits( xEventGroup, mainSECOND_TASK_BIT );
  }
}

void vApplicationTickHook( void )
{

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

  RCC->APB2ENR           |= RCC_APB2ENR_SYSCFGEN;
  RCC->AHB1ENR           |= RCC_AHB1ENR_GPIOCEN;               // enable clock
  MCU_IRQ_PORT->MODER    &= (uint32_t)~(GPIO_MODER_MODER13);    // input mode

  SYSCFG->EXTICR[3] = SYSCFG_EXTICR4_EXTI13_PC ;
  EXTI->IMR  |= EXTI_IMR_MR13;          // unmasked the interrupt
  EXTI->FTSR |= EXTI_FTSR_TR13;         // rising edge tringger on line 13

  NVIC_SetPriority(EXTI15_10_IRQn, 6);  // Enable and set EXTI Line0 Interrupt to the lowest priority
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

/* external interrupt callback function */
void ExternalInterrupt_callback()
{
  Toggle_LED();
  /* The string is not printed within the interrupt service routine, but is instead sent to the RTOS daemon task for printing. It is therefore declared static to ensure the compiler does not allocate the string on the stack of the ISR, as the ISR's stack frame will not exist when the string is printed from the daemon task. */
  static const char *pcString = "Bit setting ISR -\t about to set bit 2.\n";
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  /* Print out a message to say bit 2 is about to be set. Messages cannot be
  printed from an ISR, so defer the actual output to the RTOS daemon task by
  pending a function call to run in the context of the RTOS daemon task. */
  xTimerPendFunctionCallFromISR( vPrintStringFromDaemonTask,
  ( void * ) pcString,
  0,
  &xHigherPriorityTaskWoken );
  /* Set bit 2 in the event group. */
  xEventGroupSetBitsFromISR( xEventGroup, mainISR_BIT, &xHigherPriorityTaskWoken );

  /* xTimerPendFunctionCallFromISR() and xEventGroupSetBitsFromISR() both write to
  the timer command queue, and both used the same xHigherPriorityTaskWoken
  variable. If writing to the timer command queue resulted in the RTOS daemon task
  leaving the Blocked state, and if the priority of the RTOS daemon task is higher
  than the priority of the currently executing task (the task this interrupt
  interrupted) then xHigherPriorityTaskWoken will have been set to pdTRUE.*/

  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void vPrintStringFromDaemonTask( void *pvParameter1, uint32_t ulParameter2 )
{
  /* Process the event - in this case just print out a message and the value of
  ulParameter2. pvParameter1 is not used in this example. */
  trace_printf( (char *)pvParameter1);
}

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

portTASK_FUNCTION_PROTO(vSyncingTask, pvParameters);


/* Definitions for the event bits in the event group. */
#define mainFIRST_TASK_BIT  ( 1UL << 0UL ) /* Event bit 0, set by the first task. */
#define mainSECOND_TASK_BIT ( 1UL << 1UL ) /* Event bit 1, set by the second task. */
#define mainTHIRD_TASK_BIT  ( 1UL << 2UL ) /* Event bit 2, set by the third task. */

EventGroupHandle_t xEventGroup;

int main( void )
{
  UART_CLIInit();
  GPIO_Init();
  EXTILine13_Host_Config();

  /* Before an event group can be used it must first be created. */
  xEventGroup = xEventGroupCreate();
  /* Create three instances of the task. Each task is given a different name,
  which is later printed out to give a visual indication of which task is
  executing. The event bit to use when the task reaches its synchronization point
  is passed into the task using the task parameter. */
  xTaskCreate( vSyncingTask, "Task 1", 1000, (void *) mainFIRST_TASK_BIT, 1, NULL );
  xTaskCreate( vSyncingTask, "Task 2", 1000, (void *) mainSECOND_TASK_BIT, 1, NULL );
  xTaskCreate( vSyncingTask, "Task 3", 1000, (void *) mainTHIRD_TASK_BIT, 1, NULL );
  /* Start the scheduler so the created tasks start executing. */
  vTaskStartScheduler();
  /* As always, the following line should never be reached. */
  for( ;; );
  return 0;
}

void vSyncingTask( void *pvParameters )
{
  const TickType_t xMaxDelay = pdMS_TO_TICKS( 4000UL );
  const TickType_t xMinDelay = pdMS_TO_TICKS( 200UL );
  TickType_t xDelayTime;
  EventBits_t uxThisTasksSyncBit;
  const EventBits_t uxAllSyncBits = ( mainFIRST_TASK_BIT |
  mainSECOND_TASK_BIT |
  mainTHIRD_TASK_BIT );
  /* Three instances of this task are created - each task uses a different event
  bit in the synchronization. The event bit to use is passed into each task
  instance using the task parameter. Store it in the uxThisTasksSyncBit
  variable. */
  uxThisTasksSyncBit = ( EventBits_t ) pvParameters;
  for( ;; )
  {
    /* Simulate this task taking some time to perform an action by delaying for a
    pseudo random time. This prevents all three instances of this task reaching
    the synchronization point at the same time, and so allows the example’s
    behavior to be observed more easily. */
    xDelayTime = ( rand() % xMaxDelay ) + xMinDelay;
    vTaskDelay( xDelayTime );
    /* Print out a message to show this task has reached its synchronization
    point. pcTaskGetTaskName() is an API function that returns the name assigned
    to the task when the task was created. */
    trace_printf( "AT Time: %d          %s  %s", xTaskGetTickCount(), pcTaskGetTaskName( NULL ), "reached sync point\n" );
    /* Wait for all the tasks to have reached their respective synchronization
    points. */
    xEventGroupSync( /* The event group used to synchronize. */
    xEventGroup,
    /* The bit set by this task to indicate it has reached the
    synchronization point. */
    uxThisTasksSyncBit,
    /* The bits to wait for, one bit for each task taking part
    in the synchronization. */
    uxAllSyncBits,
    /* Wait indefinitely for all three tasks to reach the
    synchronization point. */
    portMAX_DELAY );
    /* Print out a message to show this task has passed its synchronization
    point. As an indefinite delay was used the following line will only be
    executed after all the tasks reached their respective synchronization
    points. */
    trace_printf( "AT Time: %d          %s  %s", xTaskGetTickCount(), pcTaskGetTaskName( NULL ), "exited sync point\n" );
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
}


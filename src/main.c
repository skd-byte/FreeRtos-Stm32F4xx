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

#define mainBACKLIGHT_TIMER_PERIOD pdMS_TO_TICKS( 5000 )



portTASK_FUNCTION_PROTO(vHandlerTask, pvParameters);




SemaphoreHandle_t xBinarySemaphore;

int main( void )
{
  UART_CLIInit();
  GPIO_Init();
  EXTILine13_Host_Config();

  /* Before a semaphore is used it must be explicitly created. In this example
  a binary semaphore is created. */
  xBinarySemaphore = xSemaphoreCreateBinary();
  /* Check the semaphore was created successfully. */
  if( xBinarySemaphore != NULL )
  {
    /* Create the 'handler' task, which is the task to which interrupt
    processing is deferred. This is the task that will be synchronized with
    the interrupt. The handler task is created with a high priority to ensure
    it runs immediately after the interrupt exits. In this case a priority of
    3 is chosen. */
    xTaskCreate( vHandlerTask, "Handler", 1000, NULL, 3, NULL );

    vTaskStartScheduler();
  }
  /* As normal, the following line should never be reached. */
  for( ;; );
}

void vHandlerTask( void *pvParameters )
{
  /* As per most tasks, this task is implemented within an infinite loop. */
  for( ;; )
  {
    /* Use the semaphore to wait for the event. The semaphore was created
    before the scheduler was started, so before this task ran for the first
    time. The task blocks indefinitely, meaning this function call will only
    return once the semaphore has been successfully obtained - so there is
    no need to check the value returned by xSemaphoreTake(). */
    xSemaphoreTake( xBinarySemaphore, portMAX_DELAY );
    /* To get here the event must have occurred. Process the event (in this
    Case, just print out a message). */
    trace_printf( "Handler task - Processing event.\r\n" );
  }
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

/* external interrupt callback function */
void ExternalInterrupt_callback()
{
  Toggle_LED();

  BaseType_t xHigherPriorityTaskWoken;
  /* The xHigherPriorityTaskWoken parameter must be initialized to pdFALSE as
  it will get set to pdTRUE inside the interrupt safe API function if a
  context switch is required. */
  xHigherPriorityTaskWoken = pdFALSE;
  /* 'Give' the semaphore to unblock the task, passing in the address of
  xHigherPriorityTaskWoken as the interrupt safe API function's
  pxHigherPriorityTaskWoken parameter. */
  xSemaphoreGiveFromISR( xBinarySemaphore, &xHigherPriorityTaskWoken );
  /* Pass the xHigherPriorityTaskWoken value into portYIELD_FROM_ISR(). If
  xHigherPriorityTaskWoken was set to pdTRUE inside xSemaphoreGiveFromISR()
  then calling portYIELD_FROM_ISR() will request a context switch. If
  xHigherPriorityTaskWoken is still pdFALSE then calling
  portYIELD_FROM_ISR() will have no effect. */
  portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}

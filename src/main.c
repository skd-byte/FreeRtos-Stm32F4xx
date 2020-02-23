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

void vDeferredHandlingFunction( void *pvParameter1, uint32_t ulParameter2 );


int main( void )
{
  UART_CLIInit();
  GPIO_Init();
  EXTILine13_Host_Config();

  /* Start the scheduler so the created task starts executing. */
  vTaskStartScheduler();
  /* As normal, the following line should never be reached. */
  for( ;; );
}

void vDeferredHandlingFunction( void *pvParameter1, uint32_t ulParameter2 )
{
  /* Process the event - in this case just print out a message and the value of
  ulParameter2. pvParameter1 is not used in this example. */
  trace_printf( "Handler function - Processing event %d\n\r", ulParameter2 );
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

  static uint32_t ulParameterValue = 0;
  BaseType_t xHigherPriorityTaskWoken;
  /* The xHigherPriorityTaskWoken parameter must be initialized to pdFALSE as it will
  get set to pdTRUE inside the interrupt safe API function if a context switch is
  required. */
  xHigherPriorityTaskWoken = pdFALSE;
  /* Send a pointer to the interrupt's deferred handling function to the daemon task.
  The deferred handling function's pvParameter1 parameter is not used so just set to
  NULL. The deferred handling function's ulParameter2 parameter is used to pass a
  number that is incremented by one each time this interrupt handler executes. */
  xTimerPendFunctionCallFromISR( vDeferredHandlingFunction, /* Function to execute. */
  NULL, /* Not used. */
  ulParameterValue, /* Incrementing value. */
  &xHigherPriorityTaskWoken );
  ulParameterValue++;
  /* Pass the xHigherPriorityTaskWoken value into portYIELD_FROM_ISR(). If
  xHigherPriorityTaskWoken was set to pdTRUE inside xTimerPendFunctionCallFromISR() then
  calling portYIELD_FROM_ISR() will request a context switch. If
  xHigherPriorityTaskWoken is still pdFALSE then calling portYIELD_FROM_ISR() will have
  no effect. */
  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

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



portTASK_FUNCTION_PROTO(vKeyHitTask, pvParameters);
void prvBacklightTimerCallback( TimerHandle_t xTimer );


/*-----------------------------------------------------------*/

/* This example does not have a real backlight to turn on and off, so the
following variable is used to just hold the state of the backlight. */
static BaseType_t xSimulatedBacklightOn = pdFALSE;

/* The software timer used to turn the backlight off. */
static TimerHandle_t xBacklightTimer = NULL;



int main( void )
{

  UART_CLIInit();

  GPIO_Init();
  EXTILine13_Host_Config();

  /* The backlight is off at the start. */
  xSimulatedBacklightOn = pdFALSE;

  /* Create the one shot timer, storing the handle to the created timer in
  xOneShotTimer. */
  xBacklightTimer = xTimerCreate( "Backlight",        /* Text name for the timer - not used by FreeRTOS. */
                mainBACKLIGHT_TIMER_PERIOD, /* The timer's period in ticks. */
                pdFALSE,          /* Set uxAutoRealod to pdFALSE to create a one-shot timer. */
                0,              /* The timer ID is not used in this example. */
                prvBacklightTimerCallback );/* The callback function to be used by the timer being created. */


  xTaskCreate( vKeyHitTask, "Key poll", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL );

  /* Start the timer. */
  xTimerStart( xBacklightTimer, 0 );

  /* Start the scheduler. */
  vTaskStartScheduler();

  for( ;; );
  return 0;
}


void prvBacklightTimerCallback( TimerHandle_t xTimer )
{
  TickType_t xTimeNow = xTaskGetTickCount();

  /* The backlight timer expired, turn the backlight off. */
  xSimulatedBacklightOn = pdFALSE;

  /* Print the time at which the backlight was turned off. */
  trace_printf( "Timer expired, turning backlight OFF at time\t %d\n", xTimeNow );
}


void vKeyHitTask( void *pvParameters )
{
  const TickType_t xShortDelay = pdMS_TO_TICKS( 50 );
  TickType_t xTimeNow;

  trace_printf( "Press a key to turn the backlight on.\r\n" );

  for( ;; )
  {
    /* Has a key been pressed? */
    if( (GPIOC->IDR & GPIO_PIN_13 )== 0 )
    {
      /* Record the time at which the key press was noted. */
      xTimeNow = xTaskGetTickCount();

      /* A key has been pressed. */
      if( xSimulatedBacklightOn == pdFALSE )
      {
        /* The backlight was off so turn it on and print the time at
        which it was turned on. */
        xSimulatedBacklightOn = pdTRUE;
        trace_printf( "Key pressed, turning backlight ON at time\t %d\n", xTimeNow );
      }
      else
      {
        /* The backlight was already on so print a message to say the
        backlight is about to be reset and the time at which it was
        reset. */
        trace_printf( "Key pressed, resetting software timer at time\t %d\n", xTimeNow );
      }

      /* Reset the software timer.  If the backlight was previously off
      this call will start the timer.  If the backlight was previously on
      this call will restart the timer.  A real application will probably
      read key presses in an interrupt.  If this function was an interrupt
      service routine then xTimerResetFromISR() must be used instead of
      xTimerReset(). */
      xTimerReset( xBacklightTimer, xShortDelay );
    }

    /* Don't poll too quickly. */
    vTaskDelay( xShortDelay );
  }
}



 void vApplicationTickHook( void )
 {


 }

 void vApplicationIdleHook( void )
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

  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
  RCC->AHB1ENR           |= RCC_AHB1ENR_GPIOCEN;               // enable clock
  MCU_IRQ_PORT->MODER   &= (uint32_t)~(GPIO_MODER_MODER13);    // input mode
  // MCU_IRQ_PORT->PUPDR   |= (uint32_t)(GPIO_PUPDR_PUPDR13_0);    // input pull down

  SYSCFG->EXTICR[3] = SYSCFG_EXTICR4_EXTI13_PC ;
  EXTI->IMR  |= EXTI_IMR_MR13;          // unmasked the interrupt
  EXTI->FTSR |= EXTI_FTSR_TR13;         // rising edge tringger on line 13

  NVIC_SetPriority(EXTI15_10_IRQn, 2);  // Enable and set EXTI Line0 Interrupt to the lowest priority
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

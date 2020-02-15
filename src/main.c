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
portTASK_FUNCTION_PROTO(vReadMailbox, pvParameters);
portTASK_FUNCTION_PROTO(vUpdateMailbox, pvParameters);

void vAutoReloadTimerCallback(TimerHandle_t xTimer);
void vOneShotTimerCallback(TimerHandle_t xTimer);

static uint32_t ulIdleCycleCount = 0UL;
/*
 * define GPIO Board LEDs
 */
#define RED         (1U<<5) //D13-PA5, Nucleo Board Led
#define RED_BIT     (1U<<10)
#define GPIOA_CLOCK (1<<0)

void Toggle_LED(void);
void GPIO_Init(void);



/* The periods assigned to the one-shot and auto-reload timers are 3.333 second and half a second respectively. */
#define mainONE_SHOT_TIMER_PERIOD pdMS_TO_TICKS( 3333 )
#define mainAUTO_RELOAD_TIMER_PERIOD pdMS_TO_TICKS( 500 )

int main(int argc, char* argv[])
{
  UART_CLIInit();
  GPIO_Init();
  TimerHandle_t xAutoReloadTimer, xOneShotTimer;
  BaseType_t xTimer1Started, xTimer2Started;
  /* Create the one shot timer, storing the handle to the created timer in xOneShotTimer. */
  xOneShotTimer = xTimerCreate(
  /* Text name for the software timer - not used by FreeRTOS. */
  "OneShot",
  /* The software timer's period in ticks. */
  mainONE_SHOT_TIMER_PERIOD,
  /* Setting uxAutoRealod to pdFALSE creates a one-shot software timer. */
  pdFALSE,
  /* This example does not use the timer id. */
  0,
  /* The callback function to be used by the software timer being created. */
  vOneShotTimerCallback );

  /* Create the auto-reload timer, storing the handle to the created timer in xAutoReloadTimer. */
  xAutoReloadTimer = xTimerCreate(
  /* Text name for the software timer - not used by FreeRTOS. */
  "AutoReload",
  /* The software timer's period in ticks. */
  mainAUTO_RELOAD_TIMER_PERIOD,
  /* Setting uxAutoRealod to pdTRUE creates an auto-reload timer. */
  pdTRUE,
  /* This example does not use the timer id. */
  0,
  /* The callback function to be used by the software timer being created. */
  vAutoReloadTimerCallback );
  /* Check the software timers were created. */
  if( ( xOneShotTimer != NULL ) && ( xAutoReloadTimer != NULL ) )
  {
    /* Start the software timers, using a block time of 0 (no block time). The scheduler has
    not been started yet so any block time specified here would be ignored anyway. */
    xTimer1Started = xTimerStart( xOneShotTimer, 0 );
    xTimer2Started = xTimerStart( xAutoReloadTimer, 0 );
    /* The implementation of xTimerStart() uses the timer command queue, and xTimerStart()
    will fail if the timer command queue gets full. The timer service task does not get
    created until the scheduler is started, so all commands sent to the command queue will
    stay in the queue until after the scheduler has been started. Check both calls to
    xTimerStart() passed. */
    if( ( xTimer1Started == pdPASS ) && ( xTimer2Started == pdPASS ) )
    {
      /* Start the scheduler. */
      vTaskStartScheduler();
    }
  }
  vTaskStartScheduler();
  for( ;; );
  return 0;
}


void vOneShotTimerCallback(TimerHandle_t xTimer)
{
  TickType_t xTimeNow;
  /* Obtain the current tick count. */
  xTimeNow = xTaskGetTickCount();
  /* Output a string to show the time at which the callback was executed. */
  trace_printf( "One-shot timer callback executing=%d\n", xTimeNow );
}



void vAutoReloadTimerCallback(TimerHandle_t xTimer)
{
  TickType_t xTimeNow;
  /* Obtain the current tick count. */
  xTimeNow = xTaskGetTickCount();
  /* Output a string to show the time at which the callback was executed. */
  trace_printf( "Auto-reload timer callback executing=%d\n", xTimeNow );
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

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

/*
 * define GPIO Board LEDs
 */
#define RED         (1U<<5) //D13-PA5, Nucleo Board Led
#define RED_BIT     (1U<<10)
#define GPIOA_CLOCK (1<<0)

void Toggle_LED(void);
void GPIO_Init(void);

portTASK_FUNCTION_PROTO(prvPrintTask, pvParameters);
static void prvNewPrintString( const char *pcString );

SemaphoreHandle_t xMutex;

int main( void )
{
  UART_CLIInit();
  GPIO_Init();

  /* Before a semaphore is used it must be explicitly created. In this example a
  mutex type semaphore is created. */
  xMutex = xSemaphoreCreateMutex();

  /* Check the semaphore was created successfully before creating the tasks. */
  if( xMutex != NULL )
  {
    /* Create two instances of the tasks that write to stdout. The string they
    write is passed in to the task as the task’s parameter. The tasks are
    created at different priorities so some pre-emption will occur. */
    xTaskCreate( prvPrintTask, "Print1", 1000,
    "Task 1 ***************************************\r\n", 1, NULL );

    xTaskCreate( prvPrintTask, "Print2", 1000,
    "Task 2 ---------------------------------------\r\n", 2, NULL );

    /* Start the scheduler so the created tasks start executing. */
    vTaskStartScheduler();
  }

  for( ;; );
}

void prvPrintTask( void *pvParameters )
{
  char *pcStringToPrint;
  const TickType_t xMaxBlockTimeTicks = 0x20;

  /* Two instances of this task are created. The string printed by the task is
  passed into the task using the task’s parameter. The parameter is cast to the
  required type. */
  pcStringToPrint = ( char * ) pvParameters;
  for( ;; )
  {
    /* Print out the string using the newly defined function. */
    prvNewPrintString( pcStringToPrint );

    /* Wait a pseudo random time. Note that rand() is not necessarily reentrant,
    but in this case it does not really matter as the code does not care what
    value is returned. In a more secure application a version of rand() that is
    known to be reentrant should be used - or calls to rand() should be protected
    using a critical section. */
    vTaskDelay( ( rand() % xMaxBlockTimeTicks ) );
  }
}

static void prvNewPrintString( const char *pcString )
{
  /* The mutex is created before the scheduler is started, so already exists by the
  time this task executes.
  Attempt to take the mutex, blocking indefinitely to wait for the mutex if it is
  not available straight away. The call to xSemaphoreTake() will only return when
  the mutex has been successfully obtained, so there is no need to check the
  function return value.*/
  xSemaphoreTake( xMutex, portMAX_DELAY );
  {
    /* The following line will only execute once the mutex has been successfully
    obtained. Standard out can be accessed freely now as only one task can have
    the mutex at any one time. */
    UART_Message(pcString );

    /* The mutex MUST be given back! */
  }
  xSemaphoreGive( xMutex );
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



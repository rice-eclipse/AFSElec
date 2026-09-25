#include <cstdio>
#include "BlinkAgent.h"
#include "pico/cyw43_arch.h"


constexpr int LED_HALF_PERIOD_MS = 500;
constexpr int LED_BLINK_DURATION_MS = 10000;

//Standard Task priority
#define TASK_PRIORITY		( tskIDLE_PRIORITY + 1UL )

#define GP_BLINK 2


/***
 * Debug function to look at Task Stats
 */
void runTimeStats(   ){
	TaskStatus_t *pxTaskStatusArray;
	volatile UBaseType_t uxArraySize, x;
	unsigned long ulTotalRunTime;


   // Get number of takss
   uxArraySize = uxTaskGetNumberOfTasks();
   printf("Number of tasks %d\n", uxArraySize);

   //Allocate a TaskStatus_t structure for each task.
   pxTaskStatusArray = (TaskStatus_t *)pvPortMalloc( uxArraySize * sizeof( TaskStatus_t ) );

   if( pxTaskStatusArray != NULL ){
      // Generate raw status information about each task.
      uxArraySize = uxTaskGetSystemState( pxTaskStatusArray,
                                 uxArraySize,
                                 &ulTotalRunTime );

	 // Print stats
	 for( x = 0; x < uxArraySize; x++ )
	 {
		 printf("Task: %d \t cPri:%d \t bPri:%d \t hw:%d \t%s\n",
				pxTaskStatusArray[ x ].xTaskNumber ,
				pxTaskStatusArray[ x ].uxCurrentPriority ,
				pxTaskStatusArray[ x ].uxBasePriority ,
				pxTaskStatusArray[ x ].usStackHighWaterMark ,
				pxTaskStatusArray[ x ].pcTaskName
				);
	 }


      // Free array
      vPortFree( pxTaskStatusArray );
   } else {
	   printf("Failed to allocate space for stats\n");
   }

   //Get heap allocation information
   HeapStats_t heapStats;
   vPortGetHeapStats(&heapStats);
   printf("HEAP avl: %d, blocks %d, alloc: %d, free: %d\n",
		   heapStats.xAvailableHeapSpaceInBytes,
		   heapStats.xNumberOfFreeBlocks,
		   heapStats.xNumberOfSuccessfulAllocations,
		   heapStats.xNumberOfSuccessfulFrees
		   );
}


/***
 * Main task to boot the other Agents
 * @param params - unused
 */
void mainTask(void *params){
	BlinkAgent blink(GP_BLINK);

	printf("Boot task started\n");
	blink.start("Blink", TASK_PRIORITY);

	for (;;){
		vTaskDelay(2000);
		 runTimeStats();
	}
}

void ledTask(void *params){
	for (;;){
		cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
		vTaskDelay(pdMS_TO_TICKS(LED_HALF_PERIOD_MS));
		cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
		vTaskDelay(pdMS_TO_TICKS(LED_HALF_PERIOD_MS));
	}
}

void testLedTask(void *params){
	int start = xTaskGetTickCount();
	int duration = pdMS_TO_TICKS(LED_BLINK_DURATION_MS);

	while ((xTaskGetTickCount() - start) < duration){
		cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
		vTaskDelay(pdMS_TO_TICKS(LED_HALF_PERIOD_MS));
		cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
		vTaskDelay(pdMS_TO_TICKS(LED_HALF_PERIOD_MS));
	}

	// Leave the LED solid on
	cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

	// Nothing left to do, so remove this task
	vTaskDelete(NULL);
}



/***
 * Launch the tasks and scheduler
 */
void vLaunch( void) {

	//Start blink task
    TaskHandle_t task;
    xTaskCreate(mainTask, "MainThread", 500, NULL, TASK_PRIORITY, &task);

	xTaskCreate(ledTask, "OnboardLED", 256, NULL, TASK_PRIORITY, NULL);
	//xTaskCreate(testLedTask, "OnboardLED", 256, NULL, TASK_PRIORITY, NULL);
    /* Start the tasks and timer running. */
    vTaskStartScheduler();
}




int main(){


	stdio_init_all();
	sleep_ms(1000);

	if (cyw43_arch_init()) {
		printf("cyw43 init failed\n");
	}

	printf("Go\n");

    //Start tasks and scheduler
    const char *rtos_name = "FreeRTOS";
    printf("Starting %s on core 0:\n", rtos_name);
    vLaunch();

	for (;;){
		sleep_ms(3000);
	}



}



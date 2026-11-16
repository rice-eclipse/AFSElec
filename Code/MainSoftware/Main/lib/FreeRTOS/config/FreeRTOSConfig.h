/**
 * FreeRTOSConfig.h - FreeRTOS configuration for the AFS flight computer (RP2350B, Cortex-M33, SMP).
 *
 * Based on the Raspberry Pi pico-examples FreeRTOS configuration. Both cores run the scheduler;
 * use core affinity (rtos::Task::pinToCore / vTaskCoreAffinitySet) to keep a task on one core.
 *
 * Every option is documented at https://www.freertos.org/a00110.html
 */
#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* ----------------------------------------------------------------------------
 * Scheduler
 * ------------------------------------------------------------------------- */
#define configUSE_PREEMPTION                    1
#define configUSE_TIME_SLICING                  1   /* equal-priority tasks share the CPU each tick */
#define configUSE_TICKLESS_IDLE                 0
#define configTICK_RATE_HZ                      ( ( TickType_t ) 1000 )   /* 1 tick = 1 ms */
#define configMAX_PRIORITIES                    32
#define configIDLE_SHOULD_YIELD                 1
#define configUSE_16_BIT_TICKS                  0
#define configMAX_TASK_NAME_LEN                 16

/* Stack sizes are in WORDS (4 bytes), not bytes. */
#define configSTACK_DEPTH_TYPE                  uint32_t
#define configMINIMAL_STACK_SIZE                ( configSTACK_DEPTH_TYPE ) 512   /* 2 kB: idle task etc. */

/* ----------------------------------------------------------------------------
 * Dual core (SMP) - set by the RP2xxx SMP port
 * ------------------------------------------------------------------------- */
#if FREE_RTOS_KERNEL_SMP
    #ifndef configNUMBER_OF_CORES
        #define configNUMBER_OF_CORES           2
    #endif
    #define configNUM_CORES                     configNUMBER_OF_CORES
    #define configTICK_CORE                     0
    #define configRUN_MULTIPLE_PRIORITIES       1   /* tasks of different priorities may run at the same time on the two cores */
    #if configNUMBER_OF_CORES > 1
        #define configUSE_CORE_AFFINITY         1
    #endif
    #define configUSE_PASSIVE_IDLE_HOOK         0
#endif

/* ----------------------------------------------------------------------------
 * Pico SDK interoperability: sleep_ms(), mutex_t, queue_t, sem_t etc. from the SDK block only the
 * calling task (instead of spinning the whole core) when called from a FreeRTOS task.
 * ------------------------------------------------------------------------- */
#define configSUPPORT_PICO_SYNC_INTEROP         1
#define configSUPPORT_PICO_TIME_INTEROP         1

/* ----------------------------------------------------------------------------
 * Memory
 * ------------------------------------------------------------------------- */
#define configSUPPORT_DYNAMIC_ALLOCATION        1
#ifndef configSUPPORT_STATIC_ALLOCATION
    #define configSUPPORT_STATIC_ALLOCATION     1
#endif
#define configKERNEL_PROVIDED_STATIC_MEMORY     1   /* kernel supplies idle/timer task memory itself */
#define configTOTAL_HEAP_SIZE                   ( 128 * 1024 )   /* heap_4, out of 520 kB SRAM */
#define configAPPLICATION_ALLOCATED_HEAP        0

/* ----------------------------------------------------------------------------
 * Features
 * ------------------------------------------------------------------------- */
#define configUSE_MUTEXES                       1
#define configUSE_RECURSIVE_MUTEXES             1
#define configUSE_COUNTING_SEMAPHORES           1
#define configUSE_QUEUE_SETS                    1
#define configUSE_TASK_NOTIFICATIONS            1
#define configTASK_NOTIFICATION_ARRAY_ENTRIES   3
#define configQUEUE_REGISTRY_SIZE               8
#define configUSE_APPLICATION_TASK_TAG          0
#define configUSE_NEWLIB_REENTRANT              0
#define configENABLE_BACKWARD_COMPATIBILITY     1
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS 5
#define configMESSAGE_BUFFER_LENGTH_TYPE        size_t
#define configUSE_CO_ROUTINES                   0
#define configMAX_CO_ROUTINE_PRIORITIES         1

/* Software timers */
#define configUSE_TIMERS                        1
#define configTIMER_TASK_PRIORITY               ( configMAX_PRIORITIES - 1 )
#define configTIMER_QUEUE_LENGTH                10
#define configTIMER_TASK_STACK_DEPTH            1024

/* ----------------------------------------------------------------------------
 * Hooks and debugging (hooks are implemented in rtos.cpp)
 * ------------------------------------------------------------------------- */
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
#define configUSE_DAEMON_TASK_STARTUP_HOOK      0
#define configCHECK_FOR_STACK_OVERFLOW          2   /* checks stack pattern on every context switch */
#define configUSE_MALLOC_FAILED_HOOK            1
#define configUSE_TRACE_FACILITY                1   /* uxTaskGetSystemState(), rtos::printTaskList() */
#define configUSE_STATS_FORMATTING_FUNCTIONS    0
#define configGENERATE_RUN_TIME_STATS           0

/* Assert: report file/line over stdio and halt (see rtos.cpp) */
#ifndef __ASSEMBLER__
    #ifdef __cplusplus
    extern "C" {
    #endif
    void rtos_assert_failed( const char * file, int line );
    #ifdef __cplusplus
    }
    #endif
#endif
#define configASSERT( x )    do { if( !( x ) ) rtos_assert_failed( __FILE__, __LINE__ ); } while( 0 )

/* ----------------------------------------------------------------------------
 * RP2350 (Cortex-M33) port settings - required values for the RP2350_ARM_NTZ port
 * ------------------------------------------------------------------------- */
#define configENABLE_MPU                        0
#define configENABLE_TRUSTZONE                  0
#define configRUN_FREERTOS_SECURE_ONLY          1
#define configENABLE_FPU                        1
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    16

/* ----------------------------------------------------------------------------
 * Optional API functions
 * ------------------------------------------------------------------------- */
#define INCLUDE_vTaskPrioritySet                1
#define INCLUDE_uxTaskPriorityGet               1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_xTaskDelayUntil                 1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_xTaskGetCurrentTaskHandle       1
#define INCLUDE_uxTaskGetStackHighWaterMark     1
#define INCLUDE_xTaskGetIdleTaskHandle          1
#define INCLUDE_eTaskGetState                   1
#define INCLUDE_xTimerPendFunctionCall          1
#define INCLUDE_xTaskAbortDelay                 1
#define INCLUDE_xTaskGetHandle                  1
#define INCLUDE_xTaskResumeFromISR              1
#define INCLUDE_xQueueGetMutexHolder            1

#endif /* FREERTOS_CONFIG_H */

// rtos.h implementation + FreeRTOS hook functions
#include "rtos.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/platform.h"

namespace rtos {

void start() {
    vTaskStartScheduler();
    // Only reached if the idle/timer tasks could not be created
    panic("FreeRTOS: vTaskStartScheduler returned (not enough heap?)");
}

uint32_t currentCore() {
    return get_core_num();
}

static UBaseType_t coreMask(int core) {
    if (core == 0) return 1u << 0;
    if (core == 1) return 1u << 1;
    return tskNO_AFFINITY;
}

TaskHandle_t spawn(TaskFn fn, const char* name, uint32_t stackWords, UBaseType_t priority, void* arg, int core) {
    TaskHandle_t h = nullptr;
    if (xTaskCreateAffinitySet(fn, name, stackWords, arg, priority, coreMask(core), &h) != pdPASS) return nullptr;
    return h;
}

void pinToCore(TaskHandle_t task, int core) {
    vTaskCoreAffinitySet(task, coreMask(core));
}

void notifyFromISR(TaskHandle_t task, uint32_t bits) {
    BaseType_t woken = pdFALSE;
    xTaskNotifyFromISR(task, bits, eSetBits, &woken);
    portYIELD_FROM_ISR(woken);
}

bool Task::start(const char* name, uint32_t stackWords, UBaseType_t priority, int core) {
    if (handle_) return false;  // already running
    handle_ = spawn(&Task::entry, name, stackWords, priority, this, core);
    return handle_ != nullptr;
}

void Task::entry(void* self) {
    static_cast<Task*>(self)->run();
    // run() returned: clean up instead of falling off the end of a task (which is not allowed)
    static_cast<Task*>(self)->handle_ = nullptr;
    vTaskDelete(nullptr);
}

void Task::notifyFromISR(uint32_t bits) {
    rtos::notifyFromISR(handle_, bits);
}

bool Semaphore::giveFromISR() {
    BaseType_t woken = pdFALSE;
    bool ok = xSemaphoreGiveFromISR(h_, &woken) == pdTRUE;
    portYIELD_FROM_ISR(woken);
    return ok;
}

bool EventGroup::setFromISR(uint32_t bits) {
    // Deferred to the timer task (requires INCLUDE_xTimerPendFunctionCall)
    BaseType_t woken = pdFALSE;
    bool ok = xEventGroupSetBitsFromISR(h_, bits, &woken) == pdPASS;
    portYIELD_FROM_ISR(woken);
    return ok;
}

void Timer::trampoline(TimerHandle_t t) {
    Timer* self = static_cast<Timer*>(pvTimerGetTimerID(t));
    if (self && self->cb_) self->cb_(self->user_);
}

void printTaskList() {
    UBaseType_t n = uxTaskGetNumberOfTasks();
    TaskStatus_t* list = (TaskStatus_t*)pvPortMalloc(n * sizeof(TaskStatus_t));
    if (!list) {
        printf("printTaskList: out of heap\n");
        return;
    }
    n = uxTaskGetSystemState(list, n, nullptr);
    static const char* const states[] = {"running", "ready", "blocked", "suspended", "deleted", "invalid"};
    printf("%-16s %-9s %4s %5s %11s\n", "task", "state", "prio", "cores", "free stack");
    for (UBaseType_t i = 0; i < n; i++) {
        const TaskStatus_t& t = list[i];
        UBaseType_t aff = vTaskCoreAffinityGet(t.xHandle);
        const char* cores = (aff & 3) == 3 ? "any" : (aff & 1) ? "0" : "1";
        printf("%-16s %-9s %4lu %5s %6lu words\n", t.pcTaskName, states[t.eCurrentState > 5 ? 5 : t.eCurrentState],
               (unsigned long)t.uxCurrentPriority, cores, (unsigned long)t.usStackHighWaterMark);
    }
    printf("heap: %u bytes free, %u minimum ever\n", (unsigned)freeHeap(), (unsigned)minFreeHeap());
    vPortFree(list);
}

} // namespace rtos

// ============================================================================
// FreeRTOS hooks (enabled in FreeRTOSConfig.h). Each one reports over stdio and halts, because
// continuing after any of these would mean corrupted memory.
// ============================================================================
extern "C" {

void vApplicationStackOverflowHook(TaskHandle_t task, char* name) {
    (void)task;
    panic("FreeRTOS: stack overflow in task '%s' - give it a bigger stack", name ? name : "?");
}

void vApplicationMallocFailedHook(void) {
    panic("FreeRTOS: out of heap (configTOTAL_HEAP_SIZE = %u, free %u)", (unsigned)configTOTAL_HEAP_SIZE,
          (unsigned)xPortGetFreeHeapSize());
}

void rtos_assert_failed(const char* file, int line) {
    panic("FreeRTOS assert failed: %s:%d", file, line);
}

} // extern "C"

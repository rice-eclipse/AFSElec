/**
 * rtos.h - Thin C++ wrappers around FreeRTOS (SMP, both RP2350 cores) for the flight computer.
 *
 * Everything here is a small inline layer over the FreeRTOS C API; you can mix it freely with
 * plain FreeRTOS calls (xTaskCreate, xQueueSend, ...). All timeouts are in milliseconds
 * (1 tick = 1 ms), and rtos::FOREVER waits without a timeout.
 *
 *   #include "rtos.h"
 *
 *   rtos::Queue<SensorSample> samples(64);
 *   rtos::Mutex spiLock;
 *
 *   void sensorTask(void*) {
 *       rtos::Periodic rate(10);                  // 100 Hz, drift-free
 *       while (true) {
 *           SensorSample s = readSensors();
 *           samples.send(s, 0);                   // don't block if the logger is behind
 *           rate.wait();
 *       }
 *   }
 *
 *   void loggerTask(void*) {
 *       SensorSample s;
 *       while (true) if (samples.receive(s)) writeCsvRow(s);
 *   }
 *
 *   int main() {
 *       stdio_init_all();
 *       rtos::spawn(sensorTask, "sensors", 2048, 3, nullptr, 0);   // pinned to core 0
 *       rtos::spawn(loggerTask, "logger",  2048, 2, nullptr, 1);   // pinned to core 1
 *       rtos::start();                                              // never returns
 *   }
 *
 * Stack sizes are in WORDS (1 word = 4 bytes), as in FreeRTOS.
 */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include "timers.h"

namespace rtos {

constexpr uint32_t FOREVER = 0xFFFFFFFFu;   // timeout value meaning "wait forever"
constexpr int ANY_CORE = -1;

/** Milliseconds -> ticks, with FOREVER -> portMAX_DELAY. */
inline TickType_t toTicks(uint32_t ms) {
    return ms == FOREVER ? portMAX_DELAY : pdMS_TO_TICKS(ms);
}

// ============================================================================
// Scheduler and time
// ============================================================================

/** Starts the scheduler on both cores. Never returns (unless there's not enough heap). */
void start();

/** True once start() has been called and the scheduler is running. */
inline bool isRunning() { return xTaskGetSchedulerState() == taskSCHEDULER_RUNNING; }

/** Blocks the calling task for `ms` milliseconds (other tasks run meanwhile). */
inline void delay(uint32_t ms) { vTaskDelay(toTicks(ms)); }

/** Gives up the rest of this time slice to other ready tasks of the same priority. */
inline void yield() { taskYIELD(); }

/** Milliseconds since the scheduler started (wraps after ~49 days). */
inline uint32_t millis() { return (uint32_t)xTaskGetTickCount() * portTICK_PERIOD_MS; }

/** Core (0 or 1) the calling code is running on. */
uint32_t currentCore();

/** Free / never-reached-minimum FreeRTOS heap, in bytes. */
inline size_t freeHeap() { return xPortGetFreeHeapSize(); }
inline size_t minFreeHeap() { return xPortGetMinimumEverFreeHeapSize(); }

/** Prints every task: state, priority, core affinity, and unused stack (words). */
void printTaskList();

/**
 * Fixed-rate loop helper using vTaskDelayUntil, so the period doesn't drift with the loop's run time.
 *
 *   rtos::Periodic rate(10);   // every 10 ms
 *   while (true) { work(); rate.wait(); }
 */
class Periodic {
public:
    explicit Periodic(uint32_t periodMs) : period_(toTicks(periodMs)), last_(xTaskGetTickCount()) {}
    /** Waits for the next period. Returns false if the deadline was already missed (loop overran). */
    bool wait() { return xTaskDelayUntil(&last_, period_) == pdTRUE; }
    void reset() { last_ = xTaskGetTickCount(); }
    void setPeriod(uint32_t periodMs) { period_ = toTicks(periodMs); }

private:
    TickType_t period_;
    TickType_t last_;
};

/** Disables task switching and interrupts on this core and takes the SMP lock, while in scope. Keep short. */
class CriticalSection {
public:
    CriticalSection() { taskENTER_CRITICAL(); }
    ~CriticalSection() { taskEXIT_CRITICAL(); }
    CriticalSection(const CriticalSection&) = delete;
    CriticalSection& operator=(const CriticalSection&) = delete;
};

// ============================================================================
// Tasks
// ============================================================================

using TaskFn = void (*)(void* arg);

/**
 * Creates a task. A task function must never return; loop forever or call vTaskDelete(nullptr).
 * @param stackWords stack size in words (4 bytes each); 1024-2048 is typical with printf
 * @param priority   0 (idle) .. configMAX_PRIORITIES-1; higher runs first
 * @param core       0 or 1 to pin the task to a core, or ANY_CORE
 * @return the task handle, or nullptr if there was not enough heap
 */
TaskHandle_t spawn(TaskFn fn, const char* name, uint32_t stackWords, UBaseType_t priority,
                   void* arg = nullptr, int core = ANY_CORE);

/** Pins an existing task to one core (or ANY_CORE). nullptr = calling task. */
void pinToCore(TaskHandle_t task, int core);

/**
 * Object-oriented task: subclass it, implement run(), and call start(). The object must outlive
 * the task (make it global or static).
 *
 *   class Blinker : public rtos::Task {
 *       void run() override { while (true) { gpio_xor_mask(1u << LED); rtos::delay(500); } }
 *   };
 *   Blinker blinker;
 *   blinker.start("blink", 512, 1);
 */
class Task {
public:
    virtual ~Task() = default;
    bool start(const char* name, uint32_t stackWords, UBaseType_t priority, int core = ANY_CORE);
    TaskHandle_t handle() const { return handle_; }

    void suspend() { vTaskSuspend(handle_); }
    void resume() { vTaskResume(handle_); }
    /** Sends a task notification (sets bits in notification value 0). Safe from tasks only. */
    void notify(uint32_t bits = 1) { xTaskNotify(handle_, bits, eSetBits); }
    /** Same, from an interrupt handler. */
    void notifyFromISR(uint32_t bits = 1);
    /** Unused stack in words; small values mean the stack is nearly full. */
    uint32_t stackHeadroom() const { return uxTaskGetStackHighWaterMark(handle_); }

protected:
    virtual void run() = 0;

    /** Waits for a notification. Returns the notified bits (0 on timeout) and clears them. */
    static uint32_t waitNotify(uint32_t timeoutMs = FOREVER) {
        uint32_t bits = 0;
        xTaskNotifyWait(0, 0xFFFFFFFFu, &bits, toTicks(timeoutMs));
        return bits;
    }

private:
    static void entry(void* self);
    TaskHandle_t handle_ = nullptr;
};

/** Notify / wait helpers for plain (function) tasks. */
inline void notify(TaskHandle_t task, uint32_t bits = 1) { xTaskNotify(task, bits, eSetBits); }
void notifyFromISR(TaskHandle_t task, uint32_t bits = 1);
inline uint32_t waitNotify(uint32_t timeoutMs = FOREVER) {
    uint32_t bits = 0;
    xTaskNotifyWait(0, 0xFFFFFFFFu, &bits, toTicks(timeoutMs));
    return bits;
}

// ============================================================================
// Synchronization
// ============================================================================

/** Mutex with priority inheritance. Not usable from interrupts. */
class Mutex {
public:
    Mutex() : h_(xSemaphoreCreateMutexStatic(&buf_)) {}
    ~Mutex() { vSemaphoreDelete(h_); }
    Mutex(const Mutex&) = delete;
    Mutex& operator=(const Mutex&) = delete;

    bool lock(uint32_t timeoutMs = FOREVER) { return xSemaphoreTake(h_, toTicks(timeoutMs)) == pdTRUE; }
    bool tryLock() { return lock(0); }
    void unlock() { xSemaphoreGive(h_); }
    SemaphoreHandle_t handle() const { return h_; }

private:
    StaticSemaphore_t buf_;
    SemaphoreHandle_t h_;
};

/** A mutex the same task may lock several times (unlock the same number of times). */
class RecursiveMutex {
public:
    RecursiveMutex() : h_(xSemaphoreCreateRecursiveMutexStatic(&buf_)) {}
    ~RecursiveMutex() { vSemaphoreDelete(h_); }
    RecursiveMutex(const RecursiveMutex&) = delete;
    RecursiveMutex& operator=(const RecursiveMutex&) = delete;

    bool lock(uint32_t timeoutMs = FOREVER) { return xSemaphoreTakeRecursive(h_, toTicks(timeoutMs)) == pdTRUE; }
    bool tryLock() { return lock(0); }
    void unlock() { xSemaphoreGiveRecursive(h_); }

private:
    StaticSemaphore_t buf_;
    SemaphoreHandle_t h_;
};

/**
 * Locks a mutex for the lifetime of the object (waits forever by default).
 *   { rtos::LockGuard<rtos::Mutex> g(spiLock); spiTransfer(); }
 */
template <typename M>
class LockGuard {
public:
    explicit LockGuard(M& m, uint32_t timeoutMs = FOREVER) : m_(m), locked_(m.lock(timeoutMs)) {}
    ~LockGuard() { if (locked_) m_.unlock(); }
    bool locked() const { return locked_; }
    explicit operator bool() const { return locked_; }
    LockGuard(const LockGuard&) = delete;
    LockGuard& operator=(const LockGuard&) = delete;

private:
    M& m_;
    bool locked_;
};

/** Counting semaphore (maxCount = 1 gives a binary semaphore, e.g. for "data ready" signals). */
class Semaphore {
public:
    explicit Semaphore(uint32_t maxCount = 1, uint32_t initial = 0)
        : h_(xSemaphoreCreateCountingStatic(maxCount, initial, &buf_)) {}
    ~Semaphore() { vSemaphoreDelete(h_); }
    Semaphore(const Semaphore&) = delete;
    Semaphore& operator=(const Semaphore&) = delete;

    bool take(uint32_t timeoutMs = FOREVER) { return xSemaphoreTake(h_, toTicks(timeoutMs)) == pdTRUE; }
    bool give() { return xSemaphoreGive(h_) == pdTRUE; }
    /** From an interrupt handler (requests a context switch if a higher-priority task woke). */
    bool giveFromISR();
    uint32_t count() const { return uxSemaphoreGetCount(h_); }

private:
    StaticSemaphore_t buf_;
    SemaphoreHandle_t h_;
};

/**
 * Fixed-size queue of T (copied by value). The storage is allocated from the FreeRTOS heap.
 * A length-1 queue used with overwrite()/peek() makes a "latest value" mailbox.
 */
template <typename T>
class Queue {
public:
    explicit Queue(uint32_t length) : h_(xQueueCreate(length, sizeof(T))) {}
    ~Queue() { if (h_) vQueueDelete(h_); }
    Queue(const Queue&) = delete;
    Queue& operator=(const Queue&) = delete;

    bool valid() const { return h_ != nullptr; }

    /** Adds to the back. Returns false if still full after timeoutMs. */
    bool send(const T& item, uint32_t timeoutMs = FOREVER) { return xQueueSend(h_, &item, toTicks(timeoutMs)) == pdTRUE; }
    /** Adds to the front (jumps the line). */
    bool sendToFront(const T& item, uint32_t timeoutMs = FOREVER) {
        return xQueueSendToFront(h_, &item, toTicks(timeoutMs)) == pdTRUE;
    }
    /** For length-1 queues: replaces the stored item (never blocks). */
    void overwrite(const T& item) { xQueueOverwrite(h_, &item); }
    bool sendFromISR(const T& item) {
        BaseType_t woken = pdFALSE;
        bool ok = xQueueSendFromISR(h_, &item, &woken) == pdTRUE;
        portYIELD_FROM_ISR(woken);
        return ok;
    }

    /** Removes from the front. Returns false on timeout. */
    bool receive(T& out, uint32_t timeoutMs = FOREVER) { return xQueueReceive(h_, &out, toTicks(timeoutMs)) == pdTRUE; }
    /** Reads the front item without removing it. */
    bool peek(T& out, uint32_t timeoutMs = 0) { return xQueuePeek(h_, &out, toTicks(timeoutMs)) == pdTRUE; }
    bool receiveFromISR(T& out) {
        BaseType_t woken = pdFALSE;
        bool ok = xQueueReceiveFromISR(h_, &out, &woken) == pdTRUE;
        portYIELD_FROM_ISR(woken);
        return ok;
    }

    uint32_t count() const { return uxQueueMessagesWaiting(h_); }
    uint32_t spaces() const { return uxQueueSpacesAvailable(h_); }
    bool empty() const { return count() == 0; }
    void clear() { xQueueReset(h_); }
    QueueHandle_t handle() const { return h_; }

private:
    QueueHandle_t h_;
};

/** Group of up to 24 event flags that tasks can wait on (e.g. LAUNCH, APOGEE, LANDED). */
class EventGroup {
public:
    EventGroup() : h_(xEventGroupCreateStatic(&buf_)) {}
    ~EventGroup() { vEventGroupDelete(h_); }
    EventGroup(const EventGroup&) = delete;
    EventGroup& operator=(const EventGroup&) = delete;

    uint32_t set(uint32_t bits) { return xEventGroupSetBits(h_, bits); }
    bool setFromISR(uint32_t bits);
    uint32_t clear(uint32_t bits) { return xEventGroupClearBits(h_, bits); }
    uint32_t get() const { return xEventGroupGetBits(h_); }
    /**
     * Waits for `bits`. Returns the event bits at the time the wait ended; check them to see
     * whether it timed out.
     * @param all            true: wait for every bit, false: any one of them
     * @param clearOnExit    clear the waited-for bits before returning
     */
    uint32_t wait(uint32_t bits, bool all = false, bool clearOnExit = false, uint32_t timeoutMs = FOREVER) {
        return xEventGroupWaitBits(h_, bits, clearOnExit ? pdTRUE : pdFALSE, all ? pdTRUE : pdFALSE, toTicks(timeoutMs));
    }

private:
    StaticEventGroup_t buf_;
    EventGroupHandle_t h_;
};

/**
 * Software timer. The callback runs in the FreeRTOS timer task (highest priority), so keep it short
 * and never block in it.
 */
class Timer {
public:
    using Callback = void (*)(void* user);

    Timer(const char* name, uint32_t periodMs, bool periodic, Callback cb, void* user = nullptr)
        : cb_(cb), user_(user),
          h_(xTimerCreateStatic(name, toTicks(periodMs), periodic ? pdTRUE : pdFALSE, this, &Timer::trampoline, &buf_)) {}
    ~Timer() { xTimerDelete(h_, portMAX_DELAY); }
    Timer(const Timer&) = delete;
    Timer& operator=(const Timer&) = delete;

    bool start(uint32_t timeoutMs = FOREVER) { return xTimerStart(h_, toTicks(timeoutMs)) == pdPASS; }
    bool stop(uint32_t timeoutMs = FOREVER) { return xTimerStop(h_, toTicks(timeoutMs)) == pdPASS; }
    /** Restarts the countdown from now. */
    bool reset(uint32_t timeoutMs = FOREVER) { return xTimerReset(h_, toTicks(timeoutMs)) == pdPASS; }
    bool setPeriod(uint32_t periodMs, uint32_t timeoutMs = FOREVER) {
        return xTimerChangePeriod(h_, toTicks(periodMs), toTicks(timeoutMs)) == pdPASS;
    }
    bool active() const { return xTimerIsTimerActive(h_) != pdFALSE; }

private:
    static void trampoline(TimerHandle_t t);
    Callback cb_;
    void* user_;
    StaticTimer_t buf_;
    TimerHandle_t h_;
};

} // namespace rtos

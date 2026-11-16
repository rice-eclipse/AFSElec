#include "FreeRTOS_test.h"
#include <stdio.h>
#include "pico/stdlib.h"

namespace {

bool g_verbose = false;
#define VLOG(...) do { if (g_verbose) printf(__VA_ARGS__); } while (0)

struct Checker {
    int pass = 0, fail = 0;
    bool operator()(const char* name, bool ok) {
        if (!ok || g_verbose) printf("[FreeRTOS] %s: %s\n", ok ? "PASS" : "FAIL", name);
        ok ? pass++ : fail++;
        return ok;
    }
};

constexpr uint32_t HELPER_STACK = 512;  // words

// Shared state for the helper tasks (the test runs once at a time)
rtos::Semaphore* g_done;                 // helpers give this when they finish
volatile uint32_t g_coreSeen[2];
rtos::Queue<uint32_t>* g_queue;
rtos::Mutex* g_mutex;
volatile uint32_t g_counter;
volatile uint32_t g_notified;
rtos::EventGroup* g_events;
volatile uint32_t g_timerTicks;
volatile uint32_t g_spin;
volatile uint32_t g_spinDuringSleep;
volatile bool g_stopSpin;

constexpr uint32_t QUEUE_ITEMS = 1000;
constexpr uint32_t MUTEX_LOOPS = 20000;

void coreTask(void* arg) {
    uint32_t idx = (uint32_t)(uintptr_t)arg;
    g_coreSeen[idx] = rtos::currentCore() + 1;   // +1 so 0 means "never ran"
    g_done->give();
    vTaskDelete(nullptr);
}

void producerTask(void*) {
    for (uint32_t i = 1; i <= QUEUE_ITEMS; i++) g_queue->send(i);
    g_done->give();
    vTaskDelete(nullptr);
}

void mutexTask(void*) {
    for (uint32_t i = 0; i < MUTEX_LOOPS; i++) {
        rtos::LockGuard<rtos::Mutex> lock(*g_mutex);
        uint32_t v = g_counter;   // non-atomic read-modify-write: only correct if the mutex works
        if ((i & 0xFF) == 0) rtos::yield();
        g_counter = v + 1;
    }
    g_done->give();
    vTaskDelete(nullptr);
}

void notifyTask(void*) {
    g_notified = rtos::waitNotify(500);
    g_done->give();
    vTaskDelete(nullptr);
}

void eventTask(void*) {
    rtos::delay(10);
    g_events->set(0x2);
    rtos::delay(10);
    g_events->set(0x4);
    vTaskDelete(nullptr);
}

void timerCallback(void*) {
    g_timerTicks++;
}

void spinTask(void*) {
    while (!g_stopSpin) g_spin++;
    g_done->give();
    vTaskDelete(nullptr);
}

void sleeperTask(void*) {
    uint32_t before = g_spin;
    sleep_ms(50);   // Pico SDK sleep: with FreeRTOS interop this blocks only this task
    g_spinDuringSleep = g_spin - before;
    g_done->give();
    vTaskDelete(nullptr);
}

bool waitDone(uint32_t n, uint32_t timeoutMs) {
    for (uint32_t i = 0; i < n; i++) {
        if (!g_done->take(timeoutMs)) return false;
    }
    return true;
}

uint32_t msSince(uint64_t t0) {
    return (uint32_t)((time_us_64() - t0) / 1000);
}

} // namespace

bool FreeRTOS_test(bool verbose) {
    g_verbose = verbose;
    Checker check;
    VLOG("\n===== FreeRTOS test =====\n");

    if (!check("scheduler running (call from inside a task)", rtos::isRunning())) {
        printf("[FreeRTOS] call FreeRTOS_test() from a task after rtos::start(), aborting\n");
        return false;
    }
    VLOG("  kernel %s, %d cores, tick %lu Hz, caller on core %lu\n", tskKERNEL_VERSION_NUMBER, configNUMBER_OF_CORES,
         (unsigned long)configTICK_RATE_HZ, (unsigned long)rtos::currentCore());

    // Run above the helper tasks so they can't delay this task's timing measurements
    const UBaseType_t oldPriority = uxTaskPriorityGet(nullptr);
    vTaskPrioritySet(nullptr, configMAX_PRIORITIES - 2);

    // Let the idle task free anything deleted earlier, then remember the heap level
    rtos::delay(20);
    const size_t heapBefore = rtos::freeHeap();

    rtos::Semaphore done(16, 0);
    g_done = &done;

    // --- Both cores run tasks ---
    g_coreSeen[0] = g_coreSeen[1] = 0;
    bool spawned = rtos::spawn(coreTask, "t_core0", HELPER_STACK, 2, (void*)0, 0) &&
                   rtos::spawn(coreTask, "t_core1", HELPER_STACK, 2, (void*)1, 1);
    check("tasks pinned to core 0 and core 1 run there",
          spawned && waitDone(2, 200) && g_coreSeen[0] == 1 && g_coreSeen[1] == 2);

    // --- delay() timing ---
    uint64_t t0 = time_us_64();
    rtos::delay(100);
    uint32_t dt = msSince(t0);
    VLOG("  delay(100) took %lu ms\n", (unsigned long)dt);
    check("delay(100) takes 99-102 ms", dt >= 99 && dt <= 102);

    // --- Periodic (vTaskDelayUntil) ---
    rtos::Periodic rate(5);
    bool onTime = true;
    t0 = time_us_64();
    for (int i = 0; i < 20; i++) onTime &= rate.wait();
    dt = msSince(t0);
    VLOG("  20 x Periodic(5 ms) took %lu ms\n", (unsigned long)dt);
    check("Periodic(5 ms) x20 = 100 ms, no missed deadlines", onTime && dt >= 98 && dt <= 102);

    // --- Queue between cores (scoped so its heap storage is freed before the leak check) ---
    {
    rtos::Queue<uint32_t> queue(16);
    g_queue = &queue;
    bool qOk = queue.valid() && rtos::spawn(producerTask, "t_prod", HELPER_STACK, 2, nullptr, 1);
    uint32_t expect = 1, got;
    while (qOk && expect <= QUEUE_ITEMS) {
        if (!queue.receive(got, 200) || got != expect) {
            printf("[FreeRTOS] queue: expected %lu, got %lu\n", (unsigned long)expect, (unsigned long)got);
            qOk = false;
        }
        expect++;
    }
    qOk &= waitDone(1, 200) && queue.empty();
    check("Queue: 1000 items in order from core 1", qOk);
    check("Queue receive times out when empty", !queue.receive(got, 10));
    g_queue = nullptr;
    }

    // --- Mutex under contention from both cores ---
    rtos::Mutex mutex;
    g_mutex = &mutex;
    g_counter = 0;
    t0 = time_us_64();
    bool mOk = rtos::spawn(mutexTask, "t_mtx0", HELPER_STACK, 2, nullptr, 0) &&
               rtos::spawn(mutexTask, "t_mtx1", HELPER_STACK, 2, nullptr, 1) && waitDone(2, 3000);
    VLOG("  mutex: counter %lu (expected %lu) in %lu ms\n", (unsigned long)g_counter,
         (unsigned long)(2 * MUTEX_LOOPS), (unsigned long)msSince(t0));
    check("Mutex: 2 cores x 20000 increments, none lost", mOk && g_counter == 2 * MUTEX_LOOPS);
    bool held = mutex.lock(0);
    bool again = mutex.tryLock();   // a second take of a (non-recursive) mutex must fail
    check("Mutex lock / tryLock fails while held", held && !again);
    if (held) mutex.unlock();

    // --- Semaphore timeout ---
    rtos::Semaphore empty(1, 0);
    t0 = time_us_64();
    bool took = empty.take(20);
    dt = msSince(t0);
    check("Semaphore take(20) on empty times out after ~20 ms", !took && dt >= 19 && dt <= 22);
    check("Semaphore give/take", empty.give() && empty.count() == 1 && empty.take(0) && empty.count() == 0);

    // --- Task notification ---
    g_notified = 0;
    TaskHandle_t nt = rtos::spawn(notifyTask, "t_notify", HELPER_STACK, 3);
    rtos::delay(5);
    if (nt) rtos::notify(nt, 0x5);
    check("task notification delivers bits", nt && waitDone(1, 600) && g_notified == 0x5);

    // --- EventGroup ---
    rtos::EventGroup events;
    g_events = &events;
    rtos::spawn(eventTask, "t_events", HELPER_STACK, 2);
    t0 = time_us_64();
    uint32_t bits = events.wait(0x6, /*all*/ true, /*clear*/ true, 200);
    dt = msSince(t0);
    check("EventGroup wait(all) returns once both bits are set (~20 ms)", (bits & 0x6) == 0x6 && dt >= 18 && dt <= 30);
    check("EventGroup clearOnExit cleared the bits", (events.get() & 0x6) == 0);

    // --- Software timer ---
    g_timerTicks = 0;
    rtos::Timer timer("t_timer", 10, /*periodic*/ true, timerCallback);
    timer.start();
    rtos::delay(105);
    timer.stop();
    uint32_t ticks = g_timerTicks;
    VLOG("  periodic 10 ms timer fired %lu times in 105 ms\n", (unsigned long)ticks);
    check("Timer: 10 ms periodic fires ~10 times in 105 ms", ticks >= 9 && ticks <= 11);

    // --- Pico SDK sleep_ms() yields to other tasks (configSUPPORT_PICO_TIME_INTEROP) ---
    g_spin = 0;
    g_spinDuringSleep = 0;
    g_stopSpin = false;
    bool sOk = rtos::spawn(spinTask, "t_spin", HELPER_STACK, 1, nullptr, 0) &&
               rtos::spawn(sleeperTask, "t_sleep", HELPER_STACK, 4, nullptr, 0);
    sOk &= waitDone(1, 300);  // sleeper finished
    g_stopSpin = true;
    sOk &= waitDone(1, 300);  // spinner finished
    VLOG("  low-priority task ran %lu loops while a higher-priority task was in sleep_ms(50)\n",
         (unsigned long)g_spinDuringSleep);
    check("SDK sleep_ms() inside a task lets other tasks on the same core run", sOk && g_spinDuringSleep > 1000);

    if (verbose) rtos::printTaskList();

    // --- No heap leaked (deleted tasks are freed by the idle task) ---
    rtos::delay(50);
    size_t heapAfter = rtos::freeHeap();
    VLOG("  heap free: %u before, %u after\n", (unsigned)heapBefore, (unsigned)heapAfter);
    check("no heap leaked by the test", heapAfter >= heapBefore);
    check("heap has room left (> 16 kB free)", heapAfter > 16 * 1024);

    g_done = nullptr;
    vTaskPrioritySet(nullptr, oldPriority);
    printf("[FreeRTOS] %s: %d passed, %d failed\n", check.fail ? "FAIL" : "PASS", check.pass, check.fail);
    return check.fail == 0;
}

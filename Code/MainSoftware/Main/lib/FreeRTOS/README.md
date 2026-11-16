# FreeRTOS: real-time kernel on both RP2350 cores

This library adds the FreeRTOS kernel (V11.1, SMP) to the project, using Raspberry Pi's official
RP2350 Cortex-M33 port. It also adds `rtos.h`, a set of thin C++ wrappers. With it, the flight
software can run as separate tasks, for example a sensor loop, a logger and a state machine, each at
its own rate and priority, on either core.

| Folder / file              | What it is                                                            |
|----------------------------|-----------------------------------------------------------------------|
| `FreeRTOS-Kernel/`         | Kernel sources, unmodified (from `raspberrypi/FreeRTOS-Kernel`, see `VENDORED.txt`) |
| `config/FreeRTOSConfig.h`  | Kernel configuration for this board                                   |
| `rtos.h` / `rtos.cpp`      | C++ wrappers + hook functions (stack overflow, out of heap, assert)   |
| `FreeRTOS_test.h/.cpp`     | Bench test                                                            |

Link with `target_link_libraries(<target> FreeRTOS)`. It brings in `FreeRTOS-Kernel` and the `heap_4`
allocator. The bench test target is `FreeRTOS_test`.

> **Not linked into `Main` yet.** Linking FreeRTOS changes how the Pico SDK behaves underneath:
> `sleep_ms()`, SDK mutexes and `malloc` become FreeRTOS-aware. `main()` then has to start the
> scheduler. Do this when you move `Main.cpp` over to tasks.

## Minimal program

```cpp
#include "pico/stdlib.h"
#include "rtos.h"

void blinkTask(void*) {
    rtos::Periodic rate(500);              // runs every 500 ms with no drift
    while (true) {
        gpio_xor_mask(1u << PICO_DEFAULT_LED_PIN);
        rate.wait();
    }
}

int main() {
    stdio_init_all();
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    rtos::spawn(blinkTask, "blink", 512, 1);   // name, stack (words), priority
    rtos::start();                              // never returns
}
```

Rules:
* A task function must **never return**. It must loop forever or call `vTaskDelete(nullptr)`.
  (Subclasses of `rtos::Task` are cleaned up automatically if `run()` returns.)
* **Stack sizes are in words** (4 bytes). Use 512 for simple tasks. Use 1024 to 2048 for anything
  that calls `printf`, uses floats heavily, or calls into FatFs. A stack overflow is caught and
  reported by name (`configCHECK_FOR_STACK_OVERFLOW 2`).
* **Priorities** run from 0 (idle) to 31. A higher number runs first. Tasks with the same priority share time
  in turns. Typical layout: sensors highest, state machine next, logger below that, telemetry and printing lowest.
* Every timeout in `rtos.h` is in milliseconds. `rtos::FOREVER` waits with no timeout.

## Both cores (SMP)

The scheduler runs on **both cores**. By default a task can run on either core. To pin a task:

```cpp
rtos::spawn(sensorTask, "sensors", 2048, 5, nullptr, /*core*/ 0);
rtos::spawn(loggerTask, "logger",  2048, 3, nullptr, /*core*/ 1);
```

Pinning makes timing predictable. For example, the SD card's occasional slow writes (up to about
250 ms) on core 1 then can't delay sensor sampling on core 0. Do not use the SDK's `multicore_launch_core1()`:
FreeRTOS owns core 1.

## The wrappers (`rtos.h`)

| Wrapper | Use it for |
|---|---|
| `rtos::spawn(fn, name, stackWords, prio, arg, core)` | Create a task (returns `TaskHandle_t`, `nullptr` if out of heap) |
| `class X : public rtos::Task { void run() override; }` + `x.start(...)` | Object-style task |
| `rtos::delay(ms)`, `rtos::millis()`, `rtos::yield()` | Blocking delay without spinning, uptime |
| `rtos::Periodic rate(ms); rate.wait();` | Fixed-rate loops (`vTaskDelayUntil`). `wait()` returns false if the loop overran |
| `rtos::Mutex` / `RecursiveMutex` + `rtos::LockGuard<rtos::Mutex> g(m);` | Shared resources (I2C bus, SD card, global state). Priority inheritance is built in |
| `rtos::Semaphore s(max, initial)` | Signals, e.g. from a GPIO interrupt with `s.giveFromISR()` to a task with `s.take()` |
| `rtos::Queue<T> q(len)` | Pass data between tasks, copied by value. `sendFromISR` works in interrupt handlers |
| `Queue<T>` of length 1 + `overwrite()` / `peek()` | Mailbox that always holds the latest value (e.g. the latest state estimate) |
| `rtos::notify(task, bits)` / `rtos::waitNotify(ms)` | The fastest way to wake one specific task |
| `rtos::EventGroup` | Flags several tasks can wait on (`LAUNCH`, `APOGEE`, `LANDED`, ...) |
| `rtos::Timer t(name, ms, periodic, callback)` | Software timers. The callback runs in the timer task, so keep it short and never block in it |
| `rtos::CriticalSection cs;` | Very short sections where interrupts are off on this core and the cross-core lock is held |
| `rtos::printTaskList()` | Every task's state, priority, core, unused stack, plus free heap |

`Mutex`, `Semaphore`, `EventGroup` and `Timer` use static memory, so they are safe to create as globals.
`Queue` storage comes from the FreeRTOS heap. Everything mixes freely with the plain FreeRTOS C API.

### Example: sensor task to logger task

```cpp
struct Sample { uint32_t tMs; ICM45686::Data imu; float altM; };

rtos::Queue<Sample> samples(128);   // ~0.6 s of buffering at 200 Hz
rtos::Mutex i2cLock;                // TMP117 and INA219 share i2c0

void sensorTask(void*) {            // core 0, high priority
    rtos::Periodic rate(5);         // 200 Hz
    while (true) {
        Sample s{rtos::millis()};
        imu.read(s.imu);
        s.altM = baro.getAltitude();
        if (!samples.send(s, 0)) { /* logger is behind: count a dropped sample */ }
        rate.wait();
    }
}

void loggerTask(void*) {            // core 1, lower priority. SD stalls only delay this task
    Sample s;
    while (true) {
        if (samples.receive(s)) logFile.add(s.tMs).add(s.imu.ax, 4) /* ... */ .endRow();
    }
}
```

## Interrupts

* Call only `...FromISR` functions from an interrupt handler: `Semaphore::giveFromISR`,
  `Queue::sendFromISR`, `rtos::notifyFromISR`, `EventGroup::setFromISR`. Each one switches to the woken task
  right away if that task has a higher priority.
* An interrupt that calls FreeRTOS must have a priority value numerically **≥ 16**
  (`configMAX_SYSCALL_INTERRUPT_PRIORITY`). The Pico SDK default (0x80) is fine.

## Pico SDK interop

`configSUPPORT_PICO_SYNC_INTEROP` and `configSUPPORT_PICO_TIME_INTEROP` are on. Inside a task:
* `sleep_ms()` / `sleep_us()` block only the calling task. Other tasks on that core keep running.
  The existing sensor drivers can therefore be used unchanged.
* SDK `mutex_t`, `semaphore_t` and `queue_t` block the task instead of spinning.
* `malloc`/`new` are thread-safe (`PICO_USE_MALLOC_MUTEX`). FreeRTOS objects come from their own
  128 kB heap (`configTOTAL_HEAP_SIZE`, `heap_4`). `rtos::freeHeap()` shows what's left.

Shared buses still need a lock. If two tasks use `spi0` (sensors) or `i2c0` (TMP117 and INA219),
wrap each transaction in a `rtos::Mutex`, or keep every device on a bus inside one task.

## Errors

The hooks in `rtos.cpp` call `panic()`, which prints the reason over stdio and halts:
* `FreeRTOS: stack overflow in task 'logger'`: give that task a bigger stack.
* `FreeRTOS: out of heap`: raise `configTOTAL_HEAP_SIZE` or create fewer or smaller objects.
* `FreeRTOS assert failed: file:line`: the FreeRTOS API was misused, e.g. a blocking call from an interrupt.

## Configuration highlights (`config/FreeRTOSConfig.h`)

| Option | Value | Notes |
|---|---|---|
| `configNUMBER_OF_CORES` | 2 | SMP, with core affinity |
| `configTICK_RATE_HZ` | 1000 | 1 tick = 1 ms |
| `configMAX_PRIORITIES` | 32 | |
| `configTOTAL_HEAP_SIZE` | 128 kB | Out of 520 kB SRAM |
| `configMINIMAL_STACK_SIZE` | 512 words | Idle tasks |
| `configCHECK_FOR_STACK_OVERFLOW` | 2 | Checked on every context switch |
| `configUSE_MALLOC_FAILED_HOOK` | 1 | |
| `configSUPPORT_STATIC_ALLOCATION` | 1 | Kernel provides idle/timer task memory |
| `configENABLE_FPU` / `MPU` / `TRUSTZONE` | 1 / 0 / 0 | Required values for the RP2350 ARM port |

## Bench test

```cpp
bool FreeRTOS_test(bool verbose = false);
```

Call it **from inside a task** after `rtos::start()`. It takes about 1 s. It checks:
* tasks pinned to core 0 and core 1 run on those cores
* `delay` and `Periodic` timing
* a 1000-item `Queue` sent across cores
* a `Mutex` with both cores contending (40 000 increments, none lost)
* `Semaphore` timeouts, task notifications, `EventGroup` and software timer timing
* SDK `sleep_ms()` yields to a lower-priority task on the same core
* no heap is leaked

```cpp
void bootTask(void*) {
    FreeRTOS_test(true);
    // ... other tests, then start the flight tasks ...
    vTaskDelete(nullptr);
}

int main() {
    stdio_init_all();
    rtos::spawn(bootTask, "boot", 4096, 2);
    rtos::start();
}
```

## Updating the kernel

Copy the same files from a newer `raspberrypi/FreeRTOS-Kernel` checkout: the root `*.c` files,
`include/`, `portable/MemMang/` and `portable/ThirdParty/GCC/RP2350_ARM_NTZ/`. Update `VENDORED.txt`.
The kernel is MIT licensed (`FreeRTOS-Kernel/LICENSE.md`).

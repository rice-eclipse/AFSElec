#include "shared.h"
#include "config.h"

SensorStatus        g_status;
RunStats            g_stats;
rtos::Queue<Sample> g_samples(SAMPLE_QUEUE_LEN);   // allocated from the FreeRTOS heap before main()
rtos::EventGroup    g_events;
volatile bool       g_sensorsDone = false;
char                g_logName[48];

rtos::Queue<NavRow>* g_navRows = nullptr;
volatile bool        g_navFailed = false;

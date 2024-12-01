#define __GLIBC_USE
#include "freertos/FreeRTOS.h"

#include "Common.h"

auto BlinkUpdateTicks = pdMS_TO_TICKS(10);

void blinker_main(void *) {
    log("Starting blinker_main\n");

    TickType_t LastWakeTime = xTaskGetTickCount();

    while (1) {
        // Wait for the next iteration
        vTaskDelayUntil(&LastWakeTime, BlinkUpdateTicks);

        auto WorkingLEDs = LEDs.acquire();

        // Update Value
        WorkingLEDs->render(xTaskGetTickCount() / BlinkUpdateTicks);
    }
}
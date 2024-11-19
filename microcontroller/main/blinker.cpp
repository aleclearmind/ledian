#define __GLIBC_USE
#include "freertos/FreeRTOS.h"

#include "Common.h"

auto BlinkUpdateTicks = pdMS_TO_TICKS(10);

void blinker_main(void *) {
    TickType_t LastWakeTime = xTaskGetTickCount();

    while (1) {
        // Wait for the next iteration
        vTaskDelayUntil(&LastWakeTime, BlinkUpdateTicks);

        // Update Value
        auto WorkingLEDs = LEDs.working();
        WorkingLEDs.render(xTaskGetTickCount() / BlinkUpdateTicks);

        // Commit
        LEDs.commit();
    }
}
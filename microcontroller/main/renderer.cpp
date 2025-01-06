/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include <mutex>
#include <string.h>
#include "driver/rmt_common.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "led_strip_encoder.h"
#include "soc/gpio_num.h"

#include "Common.h"

#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)

void renderer_main(void *data)
{
    log("Starting renderer_main\n");

    rmt_encoder_handle_t led_encoder = NULL;
    led_strip_encoder_config_t encoder_config = {
        .resolution = RMT_LED_STRIP_RESOLUTION_HZ,
    };
    ESP_ERROR_CHECK(rmt_new_led_strip_encoder(&encoder_config, &led_encoder));

    while (1) {

        // Ensure we have something new
        while (not LEDs.shouldRender()) {
            // Wait to be notified
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }

        rmt_channel_handle_t led_chan = NULL;
        rmt_tx_channel_config_t tx_chan_config = {
            .gpio_num = (gpio_num_t) 10,
            .clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
            .resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ,
            .mem_block_symbols = 64, // increase the block size can make the LED less flickering
            .trans_queue_depth = 4, // set the number of transactions that can be pending in the background
        };
        ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &led_chan));

        ESP_ERROR_CHECK(rmt_enable(led_chan));

        rmt_transmit_config_t tx_config = {
            .loop_count = 0, // no transfer loop
        };

        // Flush RGB values to LEDs
        // WIP: use all strips
        {
          std::lock_guard<std::mutex> AcquireLock(LEDs.Lock);
          log("Send\n");
          LEDs.render().dump(true);
          ESP_ERROR_CHECK(rmt_transmit(led_chan,
                                       led_encoder,
                                       &LEDs.render().Strips[0].LEDs,
                                       LEDs.render().Strips[0].size(),
                                       &tx_config));
          ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
          LEDs.setRendered();
        }

        // Cleanup
        ESP_ERROR_CHECK(rmt_disable(led_chan));
        ESP_ERROR_CHECK(rmt_del_channel(led_chan));

    }
}

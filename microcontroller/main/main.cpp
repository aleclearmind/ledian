#include <inttypes.h>
#include <stdio.h>
#include <cstdlib>

#include "driver/uart.h"

#include "Colors.h"
#include "CoordinateSystem.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_intr_alloc.h"
#include "esp_system.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "hal/uart_types.h"
#include "sdkconfig.h"

#include "Common.h"
#include "Command.h"
#include "LED.h"
#include "Logging.h"

void printBanner() {
  // Print chip information
  esp_chip_info_t chip_info;
  uint32_t flash_size;
  esp_chip_info(&chip_info);
  log("This is %s chip with %d CPU core(s), %s%s%s%s, ", CONFIG_IDF_TARGET,
         chip_info.cores,
         (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
         (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
         (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
         (chip_info.features & CHIP_FEATURE_IEEE802154)
             ? ", 802.15.4 (Zigbee/Thread)"
             : "");

  unsigned major_rev = chip_info.revision / 100;
  unsigned minor_rev = chip_info.revision % 100;
  log("silicon revision v%d.%d, ", major_rev, minor_rev);
  if (esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
    log("Get flash size failed");
    return;
  }

  log("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
         (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded"
                                                       : "external");

  log("Minimum free heap size: %" PRIu32 " bytes\n",
         esp_get_minimum_free_heap_size());
}

const size_t UARTBufferSize = 4096;
#define EX_UART_NUM UART_NUM_0
uint8_t UARTBuffer[UARTBufferSize];

static void uart_event_task(QueueHandle_t UARTQueue)
{
    uart_event_t event;
    for (;;) {
        //Waiting for UART event.
        if (xQueueReceive(UARTQueue, (void *)&event, (TickType_t)portMAX_DELAY)) {
            bzero(UARTBuffer, UARTBufferSize);
            log("uart event\n");
            switch (event.type) {
            //Event of UART receiving data
            /*We'd better handler data event fast, there would be much more data events than
            other types of events. If we take too much time on data event, the queue might
            be full.*/
            case UART_DATA:
                log("[UART DATA]: %d\n", event.size);
                break;
            //Event of HW FIFO overflow detected
            case UART_FIFO_OVF:
                log("hw fifo overflow\n");
                // If fifo overflow happened, you should consider adding flow control for your application.
                // The ISR has already reset the rx FIFO,
                // As an example, we directly flush the rx buffer here in order to read more data.
                // uart_flush_input(EX_UART_NUM);
                xQueueReset(UARTQueue);
                break;
            //Event of UART ring buffer full
            case UART_BUFFER_FULL:
                log("ring buffer full\n");
                // If buffer full happened, you should consider increasing your buffer size
                // As an example, we directly flush the rx buffer here in order to read more data.
                // uart_flush_input(EX_UART_NUM);
                xQueueReset(UARTQueue);
                break;
            //Event of UART RX break detected
            case UART_BREAK:
                log("uart rx break\n");
                break;
            //Event of UART parity check error
            case UART_PARITY_ERR:
                log("uart parity error\n");
                break;
            //Event of UART frame error
            case UART_FRAME_ERR:
                log("uart frame error\n");
                break;
            //Others
            default:
                log("uart event type: %d\n", event.type);
                break;
            }
        }
    }
}

extern "C" void app_main() {
#ifdef HIGHSPEED
  /* Configure parameters of an UART driver,
    * communication pins and install the driver */
  uart_config_t uart_config = {
      .baud_rate = 460800,
      .data_bits = UART_DATA_8_BITS,
      .parity    = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };
  int intr_alloc_flags = 0;

  uart_port_t ECHO_UART_PORT_NUM = (uart_port_t) 0;
#define BUF_SIZE 1024
#define ECHO_TEST_TXD 16
#define ECHO_TEST_RXD 17
#define ECHO_TEST_RTS 0
#define ECHO_TEST_CTS 0

  ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
  ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));
  ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));

#if 0
  // Configure a temporary buffer for the incoming data
  uint8_t *data = (uint8_t *) malloc(BUF_SIZE);

  while (1) {
      ESP_LOGE("lol", "go");
      // Read data from the UART
      int len = uart_read_bytes(ECHO_UART_PORT_NUM, data, (BUF_SIZE - 1), 20 / portTICK_PERIOD_MS);
      // Write data back to the UART
      uart_write_bytes(ECHO_UART_PORT_NUM, (const char *) data, len);
      if (len) {
          data[len] = '\0';
          ESP_LOGI("lel", "Recv str: %s", (char *) data);
      }
  }
#endif

#endif

  log("Printing banner\n");

  printBanner();

  esp_log_level_set("gpio", ESP_LOG_WARN);

  // Create tasks

  log("Creating tasks\n");

  // xTaskCreate(renderer_main, "renderer", 4096, NULL, 1, &RendererTask);
#define LED
#if !defined(HIGHSPEED) || defined(LED)
  TaskHandle_t RendererTask;
  xTaskCreate(renderer_main2, "renderer2", 4096, NULL, 1, &RendererTask);

  // Register the renderer task
  LEDs.setRenderer(RendererTask);
#endif

  // xTaskCreate(blinker_main, "blinker", 4096 * 4, NULL, 1, NULL);
  xTaskCreate(command_parser_main, "command_parser", 4096, NULL, 1, NULL);

  const TickType_t xDelay = 50 / portTICK_PERIOD_MS;
  size_t CurrentRow = 0;
  size_t CurrentColumn = 0;
  size_t CurrentColor = 0;
  // std::array Colors = { RGBColor(254, 0, 0) };
  std::array Colors = { RGBColor(100, 0, 0), RGBColor(0, 100, 0), RGBColor(0, 0, 100) };
  RGBColor Off(10, 10, 10);

  bool Blink = false;

  {
    auto WorkingLEDs = LEDs.acquire();

    for (coordinate_t Column = 0; Column < TheCoordinateSystem::columns(); ++Column)
      for (coordinate_t Row = 0; Row < TheCoordinateSystem::lines(); ++Row)
        WorkingLEDs->set(Column, Row, Off, false);

    WorkingLEDs->set(CurrentColumn, CurrentRow, Colors[CurrentColor], Blink);
  }

  bool OnlyFirst3 = false;

#if 0
  while (1)
    vTaskDelay(xDelay);
#endif

  while (1) {
    vTaskDelay(xDelay);

    {
      auto WorkingLEDs = LEDs.acquire();

      log("Turning off %d %d\n", CurrentColumn, CurrentRow);
      // WorkingLEDs->set(CurrentColumn, CurrentRow, Off, false);

      ++CurrentColumn;

      if (OnlyFirst3 and CurrentColumn % 6 == 0) {
        CurrentColumn = 0;
      }

      if (not OnlyFirst3 and CurrentColumn % TheCoordinateSystem::columns() == 0) {
        CurrentColumn = 0;
        CurrentRow = (CurrentRow + 1) % TheCoordinateSystem::lines();
      }

      CurrentColor = (CurrentColor + 1) % Colors.size();

      log("Turning on %d %d\n", CurrentColumn, CurrentRow);
      WorkingLEDs->set(CurrentColumn, CurrentRow, Colors[CurrentColor], Blink);
    }

  }

}

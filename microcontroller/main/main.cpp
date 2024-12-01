#include <inttypes.h>
#include <stdio.h>
#include <cstdlib>

#include "Colors.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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
  printf("This is %s chip with %d CPU core(s), %s%s%s%s, ", CONFIG_IDF_TARGET,
         chip_info.cores,
         (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
         (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
         (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
         (chip_info.features & CHIP_FEATURE_IEEE802154)
             ? ", 802.15.4 (Zigbee/Thread)"
             : "");

  unsigned major_rev = chip_info.revision / 100;
  unsigned minor_rev = chip_info.revision % 100;
  printf("silicon revision v%d.%d, ", major_rev, minor_rev);
  if (esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
    printf("Get flash size failed");
    return;
  }

  printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
         (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded"
                                                       : "external");

  printf("Minimum free heap size: %" PRIu32 " bytes\n",
         esp_get_minimum_free_heap_size());
}

extern "C" void app_main() {
  log("Printing banner\n");

  printBanner();

  // Initialized LEDs
  auto &WorkingLEDs = LEDs.working();

  using TheCoordinateSystem = std::decay_t<decltype(WorkingLEDs)>::TheCoordinateSystem;

#if 0
  for (unsigned Line = 0; Line < TheCoordinateSystem::lines(); ++Line)
    for (unsigned Column = 0; Column < TheCoordinateSystem::columns(); ++Column)
      WorkingLEDs.set(Column, Line, RGBColor(0, 255, 0), true);

  WorkingLEDs.set(0, 0, RGBColor(255, 0, 0), false);
  WorkingLEDs.set(0, 1, RGBColor(255, 0, 0), true);
  WorkingLEDs.set(0, 2, RGBColor(255, 0, 0), false);

  WorkingLEDs.set(0, 3, RGBColor(0, 255, 0), false);
  WorkingLEDs.set(0, 4, RGBColor(0, 255, 0), true);
  WorkingLEDs.set(0, 5, RGBColor(0, 255, 0), false);

  WorkingLEDs.set(0, 6, RGBColor(0, 0, 255), false);
  WorkingLEDs.set(0, 7, RGBColor(0, 0, 255), true);
  WorkingLEDs.set(0, 8, RGBColor(0, 0, 255), false);

  LEDs.commit();
#endif

  // Create tasks
  TaskHandle_t RendererTask;

  log("Creating tasks\n");
  xTaskCreate(renderer_main, "renderer", 4096, NULL, 1, &RendererTask);

  // Register the renderer task
  LEDs.setRenderer(RendererTask);

  xTaskCreate(blinker_main, "blinker", 4096 * 4, NULL, 1, NULL);
  // xTaskCreate(command_parser_main, "command_parser", 4096, NULL, 1, NULL);

  const TickType_t xDelay = 500 / portTICK_PERIOD_MS;
  size_t CurrentRow = 0;
  size_t CurrentColumn = 0;
  size_t CurrentColor = 0;
  std::array<RGBColor, 3> Colors = { RGBColor(254, 0, 0), RGBColor(0, 254, 0), RGBColor(0, 0, 254) };
  RGBColor Off(0, 0, 0);

  {
    auto WorkingLEDs = LEDs.acquire();
    WorkingLEDs->set(CurrentColumn, CurrentRow, Colors[CurrentColor], true);
  }

  while (1) {
    log("Wait\n");
    vTaskDelay(xDelay);

    {
      auto WorkingLEDs = LEDs.acquire();

      WorkingLEDs->set(CurrentColumn, CurrentRow, Off, false);

      ++CurrentColumn;
      if (CurrentColumn % TheCoordinateSystem::columns() == 0) {
        CurrentColumn = 0;
        CurrentRow = (CurrentRow + 1) % TheCoordinateSystem::lines();
      }

      CurrentColor = (CurrentColor + 1) % Colors.size();

      WorkingLEDs->set(CurrentColumn, CurrentRow, Colors[CurrentColor], true);
    }


    log("Wait\n");
    vTaskDelay(xDelay);


  }

#if 0
  Trace M(event_ids::Main);

  Trace B(event_ids::BlinkLED);
  for (int I = 0; I < 3; ++I) {
    // TODO
  }

  LEDs.resize(MaxLEDs);

  {
    Trace T(event_ids::InitialSetup);
    for (size_t J = 0; J < MaxPorts; ++J) {
      size_t Index = 0;
      while (true) {
        if (Index >= MaxLEDs)
          break;
        LEDs.Strips[J].setBlinking(Index);
        LEDs.Strips[J].LEDs[Index++] = RGBColor(255, 0, 0);
        if (Index >= MaxLEDs)
          break;
        LEDs.Strips[J].LEDs[Index++] = RGBColor(0, 255, 0);
        if (Index >= MaxLEDs)
          break;
        LEDs.Strips[J].LEDs[Index++] = RGBColor(0, 0, 255);
      }
    }
  }

  size_t Time = 0;

  while (true) {
    Trace T(event_ids::MainLoopIteration, Time);
    if (Command::parse()) {
      LEDs.dump(true);
      LEDs.render(Time);
    }
    // miosix::Thread::sleep(10);
    ++Time;
  }

  esp_restart();
#endif
}

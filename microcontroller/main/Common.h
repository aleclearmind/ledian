#pragma once

#define __GLIBC_USE
#include "freertos/FreeRTOS.h"

#include "LED.h"

class LEDBuffers {
public:
  static constexpr size_t MaxLEDs = 11 * 40;
  static constexpr size_t MaxPorts = 4;
  using ConfiguredLEDArray = LEDArray<MaxLEDs, MaxPorts>;

private:
  std::array<ConfiguredLEDArray, 2> LEDBuffers;
  ConfiguredLEDArray *RenderBuffer = &LEDBuffers[0];
  ConfiguredLEDArray *WorkingBuffer = &LEDBuffers[1];
  bool Rendered = false;
  TaskHandle_t RendererTask;

public:
  void commit() {
    // WIP: add mutex
    std::swap(RenderBuffer, WorkingBuffer);
    Rendered = false;
    if (RendererTask != nullptr)
      xTaskNotifyGive(RendererTask);
    *WorkingBuffer = *RenderBuffer;
  }

  void setRenderer(TaskHandle_t RendererTask) { this->RendererTask = RendererTask;}
  void setRendered() { Rendered = true; }
  bool shouldRender() const { return not Rendered; }

public:
  ConfiguredLEDArray &working() { return *WorkingBuffer; }
  const ConfiguredLEDArray &render() { return *RenderBuffer; }

};

inline LEDBuffers LEDs;

void renderer_main(void *);
void blinker_main(void *);
void command_parser_main(void *);

#pragma once

#include <mutex>
#include "freertos/FreeRTOS.h"

#include "LED.h"

class LEDBuffers;

static constexpr size_t MaxLEDs = 11 * 40;
static constexpr size_t MaxPorts = 4;
using ConfiguredLEDArray = LEDArray<MaxLEDs, MaxPorts>;

class WriteLock {
private:
  LEDBuffers &LEDs;
  std::lock_guard<std::mutex> Lock;

public:
  WriteLock(LEDBuffers &LEDs);
  ~WriteLock();

public:
  ConfiguredLEDArray &operator*();
  ConfiguredLEDArray *operator->();
};


class LEDBuffers {
  friend class WriteLock;

private:
  std::array<ConfiguredLEDArray, 2> LEDBuffers;
  ConfiguredLEDArray *RenderBuffer = &LEDBuffers[0];
  ConfiguredLEDArray *WorkingBuffer = &LEDBuffers[1];
  bool Rendered = false;
  TaskHandle_t RendererTask;
  std::mutex Lock;

private:
  void commit() {
    std::swap(RenderBuffer, WorkingBuffer);
    Rendered = false;
    if (RendererTask != nullptr)
      xTaskNotifyGive(RendererTask);
    *WorkingBuffer = *RenderBuffer;
  }

public:
  WriteLock acquire() { return WriteLock(*this); }

public:
  void setRenderer(TaskHandle_t RendererTask) { this->RendererTask = RendererTask;}
  void setRendered() { Rendered = true; }
  bool shouldRender() const { return not Rendered; }

public:
  ConfiguredLEDArray &working() { return *WorkingBuffer; }
  const ConfiguredLEDArray &render() { return *RenderBuffer; }

};

inline WriteLock::WriteLock(LEDBuffers &LEDs) : LEDs(LEDs), Lock(LEDs.Lock) {}
inline WriteLock::~WriteLock() { LEDs.commit(); }
inline ConfiguredLEDArray &WriteLock::operator*() { return LEDs.working(); }
inline ConfiguredLEDArray *WriteLock::operator->() { return &LEDs.working(); }

inline LEDBuffers LEDs;

void renderer_main(void *);
void blinker_main(void *);
void command_parser_main(void *);

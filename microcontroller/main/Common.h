#pragma once

#include <mutex>
#include "freertos/FreeRTOS.h"

#include "LED.h"

#define HIGHSPEED

class LEDBuffers;

static constexpr size_t MaxLEDs = 24 * 80;
static constexpr size_t MaxPorts = 4;

class WriteLock {
private:
  LEDBuffers &LEDs;
  std::lock_guard<std::mutex> Lock;

public:
  WriteLock(LEDBuffers &LEDs);
  ~WriteLock();

public:
  LEDArray &operator*();
  LEDArray *operator->();
};


class LEDBuffers {
  friend class WriteLock;

private:
  std::array<LEDArray, 2> LEDBuffers;
  LEDArray *RenderBuffer = &LEDBuffers[0];
  LEDArray *WorkingBuffer = &LEDBuffers[1];
  bool Rendered = false;
  TaskHandle_t RendererTask;

public:
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
  LEDArray &working() { return *WorkingBuffer; }
  const LEDArray &render() { return *RenderBuffer; }

};

inline WriteLock::WriteLock(LEDBuffers &LEDs) : LEDs(LEDs), Lock(LEDs.Lock) {}
inline WriteLock::~WriteLock() { LEDs.commit(); }
inline LEDArray &WriteLock::operator*() { return LEDs.working(); }
inline LEDArray *WriteLock::operator->() { return &LEDs.working(); }

inline LEDBuffers LEDs;

void renderer_main(void *);
void renderer_main2(void *);
void blinker_main(void *);
void command_parser_main(void *);

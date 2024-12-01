#pragma once

#include "Logging.h"
#include <stdint.h>

class RGBColor;

class HSVColor {
public:
  uint8_t Hue;
  uint8_t Saturation;
  uint8_t Value;

public:
  constexpr HSVColor() : Hue(0), Saturation(0), Value(0) {}
  constexpr HSVColor(uint8_t Hue, uint8_t Saturation, uint8_t Value)
      : Hue(Hue), Saturation(Saturation), Value(Value) {}

public:
  RGBColor toRGBColor() const;
};

class RGBColor {
public:
  uint8_t Red;
  uint8_t Green;
  uint8_t Blue;

public:
  constexpr RGBColor() : Red(), Green(0), Blue(0) {}
  constexpr RGBColor(uint8_t Red, uint8_t Green, uint8_t Blue)
      : Red(Red), Green(Green), Blue(Blue) {}

  bool operator==(const RGBColor &) const = default;

public:
  HSVColor toHSVColor() const;

public:
  void dump(bool Colorful, bool Blink) const {
    if (Colorful) {
      if (Blink)
        log("\033[5m");
      log("\033[38;2;%d;%d;%dm\u2588", Red, Green, Blue);
      log("\033[0m");
    } else {
      if (Blink)
        log("*");
      else
        log(" ");
      log("#%02x%02x%02x", Red, Green, Blue);
      if (Blink)
        log("*");
      else
        log(" ");
    }

  }
};

static_assert(sizeof(RGBColor) == 3);
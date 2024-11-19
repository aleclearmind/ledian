#pragma once

#include "ArrayRef.h"
#include "Colors.h"
#include "CoordinateSystem.h"
#include "Logging.h"

template <size_t Gpio, size_t Index> struct WS2812Pin {

  static void setOutput() {
    // TODO
  }

  static void setInput() {
    // TODO
  }

  static void writeBuffer(ArrayRef<const uint8_t> Buffer) {
    // TODO
  }
};

class LEDDescriptor {
public:
  HSVColor Color;
  uint8_t Blink;

public:
  LEDDescriptor() : Color(), Blink(0) {}
  LEDDescriptor(HSVColor Color) : Color(Color), Blink(0) {}
  LEDDescriptor(RGBColor Color) : Color(Color.toHSVColor()), Blink(0) {}
  LEDDescriptor(HSVColor Color, uint8_t Blink) : Color(Color), Blink(Blink) {}
  LEDDescriptor(RGBColor Color, uint8_t Blink)
      : Color(Color.toHSVColor()), Blink(Blink) {}

public:
  bool verify() const { return Blink < 2; }
};

static_assert(sizeof(HSVColor) == 3);
static_assert(sizeof(LEDDescriptor) == 4);

template <size_t MaxSize>
struct Strip {
public:
  coordinate_t Index = 0;
  std::array<RGBColor, MaxSize> LEDs;
  std::array<uint8_t, (MaxSize + 7) / 8> Blink;

public:
  bool blinks(size_t Index) const {
    return (((Blink[Index / 8] >> (Index % 8)) & 1) != 0);
  }

  size_t size() const { return MaxSize; }

public:
  void setBlinking(size_t Index) { Blink[Index / 8] |= 1 << (Index % 8); }

  void clearBlinking(size_t Index) { Blink[Index / 8] &= ~(1 << (Index % 8)); }

public:
  void dump(bool Colorful) const {
    log("Strip %d: ", Index);
    for (coordinate_t LEDIndex = 0; LEDIndex < MaxSize; ++LEDIndex) {
      bool Blinks = blinks(LEDIndex);
      LEDs[LEDIndex].dump(Colorful, Blinks);
    }
  }
};

template<size_t MaxSize>
struct ConstLEDRef {
  const Strip<MaxSize> &Strip;
  coordinate_t Index = 0;

  const RGBColor &color() const { return Strip.LEDs[Index]; }
  bool blinks() const { return Strip.blinks(Index); }

  void dump(bool Colorful) const {
    color().dump(Colorful, blinks());
  }
};

template<size_t MaxSize>
struct LEDRef {
  Strip<MaxSize> &Strip;
  coordinate_t Index = 0;

  RGBColor &color() const { return Strip.LEDs[Index]; }
  bool blinks() const { return Strip.blinks(Index); }
  void setBlinking() const { return Strip.setBlinking(Index); }
  void clearBlinking() const { return Strip.clearBlinking(Index); }

  void dump() const {
    ConstLEDRef<MaxSize>(Strip, Index).dump();
  }

};

template <size_t MaxSize, size_t MaxPorts>
struct LEDArray {
public:
  using LEDRef = LEDRef<MaxSize>;
  using ConstLEDRef = ConstLEDRef<MaxSize>;

private:
  size_t ActualSize;

public:
  std::array<Strip<MaxSize>, MaxPorts> Strips;

public:
  // TODO: hardcoded and redundants
  using TheCoordinateSystem =
      CoordinateSystem<4, 2,
                       std::array<Panel, 4>{Panel{Corner::NorthWest, 0},
                                            Panel{Corner::NorthEast, 1},
                                            Panel{Corner::SouthWest, 2},
                                            Panel{Corner::SouthEast, 3}},
                       40, 11>;

public:
  LEDArray() : ActualSize(MaxSize) {
    for (coordinate_t Index = 0; Index < MaxPorts; ++Index) {
      Strips[Index].Index = Index;
    }
  }

private:
  ConstLEDRef led(size_t Column, size_t Line) const {
    LEDCoordinate Coordinate =
        TheCoordinateSystem::convert(Point{Column, Line});
    return { Strips[Coordinate.StripIndex], Coordinate.LEDIndex };
  }

  LEDRef led(size_t Column, size_t Line) {
    LEDCoordinate Coordinate =
        TheCoordinateSystem::convert(Point{Column, Line});
    return { Strips[Coordinate.StripIndex], Coordinate.LEDIndex };
  }

public:
  size_t size() const { return ActualSize; }

public:
  void set(size_t Column, size_t Line, const RGBColor &Color, bool Blink) {
    LEDRef LED = led(Column, Line);
    log("LEDArray.set(Column: %d, Line: %d, Color: ", Column, Line);
    Color.dump(false, Blink);
    log(", StripIndex: %d, LED index: %d)\n", LED.Strip.Index, LED.Index);

    LED.color() = Color;

    if (Blink)
      LED.setBlinking();
    else
      LED.clearBlinking();
  }

  void resize(size_t NewSize) {
    assert(NewSize <= MaxSize);
    ActualSize = NewSize;
    for (size_t J = 0; J < MaxPorts; J++) {
      for (size_t I = NewSize; I < MaxSize; ++I) {
        Strips[J].LEDs[I] = {};
      }
    }
  }

public:
  void render(size_t Time) {
    Trace TT(event_ids::Render);
    renderImpl<0>(Time);
  }

  template <size_t J>
  void renderImpl(size_t Time) {
    if constexpr (J >= MaxPorts) {
      return;
    } else {
      Trace TT(event_ids::RenderStrip, J);

      Trace TTT(event_ids::AdjustBlinking);
      for (size_t I = 0; I < ActualSize; ++I) {
        HSVColor Color = Strips[J].LEDs[I].toHSVColor();

        if (Strips[J].blinks(I)) {
          constexpr uint8_t MinValue = 0;
          constexpr uint8_t MaxValue = 10;
          size_t ScaledTime = Time / 1;
          uint8_t ValueShift = ScaledTime % MaxValue;
          if ((ScaledTime / MaxValue) & 1)
            ValueShift = (MaxValue - 1) - ValueShift;
          ValueShift += MinValue;
          Color.Value = ValueShift;
        }

        Strips[J].LEDs[I] = Color.toRGBColor();
      }
      TTT.stop();

      Trace TFlush(event_ids::FlushBuffer);
      ArrayRef<const uint8_t> Buffer{
          reinterpret_cast<const uint8_t *>(&Strips[J].LEDs[0]),
          ActualSize * sizeof(RGBColor)};
      if (J == 0) {
        WS2812Pin<0, 0>::setOutput();
        WS2812Pin<0, 0>::writeBuffer(Buffer);
        WS2812Pin<0, 0>::setInput();
      } else if (J == 1) {
        WS2812Pin<1, 1>::setOutput();
        WS2812Pin<1, 1>::writeBuffer(Buffer);
        WS2812Pin<1, 1>::setInput();
      } else if (J == 2) {
        WS2812Pin<2, 2>::setOutput();
        WS2812Pin<2, 2>::writeBuffer(Buffer);
        WS2812Pin<2, 2>::setInput();
      } else if (J == 3) {
        WS2812Pin<3, 5>::setOutput();
        WS2812Pin<3, 5>::writeBuffer(Buffer);
        WS2812Pin<3, 5>::setInput();
      }

      TFlush.stop();

      TT.stop();

      renderImpl<J + 1>(Time);
    }
  }

public:
  void dump(bool Colorful) const {
    log("Strips:\n");
    for (coordinate_t StripIndex = 0; StripIndex < MaxPorts; ++StripIndex) {
      log("  Strip %d: ", StripIndex);
      Strips[StripIndex].dump(Colorful);
      log("\n");
    }

    log("\nVisual:\n");

    if (Colorful) {
      log("        ");
      for (coordinate_t Column = 0; Column < TheCoordinateSystem::columns(); ++Column) {
        if (Column % TheCoordinateSystem::columnsPerPanel() == 0)
          log("  ");
        if (Column >= 10)
          log("%d", Column / 10);
        else
         log(" ");
      }
      log("\n");
      log("        ");
      for (coordinate_t Column = 0; Column < TheCoordinateSystem::columns(); ++Column) {
        if (Column % TheCoordinateSystem::columnsPerPanel() == 0)
          log("  ");
        log("%d", Column % 10);
      }
    } else {
      log("        ");
      for (coordinate_t Column = 0; Column < TheCoordinateSystem::columns(); ++Column) {
        if (Column % TheCoordinateSystem::columnsPerPanel() == 0)
          log("  ");
        log(" Col #%02d ", Column);
      }
    }

    for (coordinate_t Line = 0; Line < TheCoordinateSystem::lines(); ++Line) {
      if (Line % TheCoordinateSystem::linesPerPanel() == 0)
        log("\n");

      log("Line %02d:", Line);
      for (coordinate_t Column = 0; Column < TheCoordinateSystem::columns(); ++Column) {
        if (Column % TheCoordinateSystem::columnsPerPanel() == 0)
          log("  ");
        led(Column, Line).dump(Colorful);
      }

      log("\n");
    }
    log("\n");

  }
};

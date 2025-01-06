#pragma once

#include <limits>

#include "ArrayRef.h"
#include "Colors.h"
#include "CoordinateSystem.h"
#include "Logging.h"

class LEDDescriptor {
private:
  uint8_t Red = 0;
  uint8_t Green = 0;
  uint8_t Blue = 0;
  // WIP: remove blink
  uint8_t Blink = 0;

public:
  RGBColor color() const { return RGBColor(Red, Green, Blue); }
  bool blinks() const { return Blink != 0; }

public:
  bool verify() const { return Blink < 2; }
};

static_assert(sizeof(HSVColor) == 3);
static_assert(sizeof(LEDDescriptor) == 4);

template <size_t StripSize>
class Strip {
public:
  static constexpr auto NoBlinking = std::numeric_limits<coordinate_t>::max();

public:
  coordinate_t StripIndex = 0;
  std::array<RGBColor, StripSize> LEDs;
  coordinate_t Blinking = NoBlinking;
  RGBColor BlinkingColor;

public:
  bool blinks() const {
    return Blinking != NoBlinking;
  }

  bool blinks(size_t Index) const {
    return Index == Blinking;
  }

  size_t size() const { return sizeof(RGBColor) * StripSize; }

public:
  void setBlinking(size_t Index) {
    Blinking = Index;
    BlinkingColor = LEDs[Blinking];
  }

  void clearBlinking(size_t Index) { Blinking = NoBlinking; }

public:
  void dump(bool Colorful) const {
    log("Strip %d: ", StripIndex);
    for (coordinate_t LEDIndex = 0; LEDIndex < StripSize; ++LEDIndex) {
      bool Blinks = blinks(LEDIndex);
      if (Blinks)
        BlinkingColor.dump(Colorful, Blinks);
      else
        LEDs[LEDIndex].dump(Colorful, Blinks);
    }
  }
};

// TODO: hardcoded and redundants
#if 1
using TheCoordinateSystem =
  CoordinateSystem<4, 2,
                   std::array<Panel, 4>{Panel{Corner::NorthWest, 1},
                                        Panel{Corner::NorthEast, 2},
                                        Panel{Corner::SouthWest, 0},
                                        Panel{Corner::SouthEast, 3}},
                   40, 12>;
// using TheCoordinateSystem =
//   CoordinateSystem<1, 1,
//                    std::array<Panel, 1>{Panel{Corner::NorthWest, 0}},
//                    40, 12>;
// using TheCoordinateSystem =
//   CoordinateSystem<4, 2,
//                    std::array<Panel, 4>{Panel{Corner::NorthWest, 0},
//                                         Panel{Corner::NorthEast, 1},
//                                         Panel{Corner::SouthWest, 2},
//                                         Panel{Corner::SouthEast, 3}},
//                    40, 24>;
// using TheCoordinateSystem =
//   CoordinateSystem<1, 1,
//                     std::array<Panel, 1>{Panel{Corner::NorthWest, 0}},
//                     20, 12>;
#else
using TheCoordinateSystem =
  CoordinateSystem<4, 2,
                    std::array<Panel, 4>{Panel{Corner::NorthWest, 0},
                                        Panel{Corner::NorthEast, 1},
                                        Panel{Corner::SouthWest, 2},
                                        Panel{Corner::SouthEast, 3}},
                    40, 11>;
#endif

using ConfiguredStrip = Strip<TheCoordinateSystem::cellsPerPanel()>;

struct ConstLEDRef {
  const ConfiguredStrip &Strip;
  coordinate_t Index = 0;

  const RGBColor &color() const { return Strip.LEDs[Index]; }
  bool blinks() const { return Strip.blinks(Index); }

  void dump(bool Colorful) const {
    color().dump(Colorful, blinks());
  }
};

struct LEDRef {
  ConfiguredStrip &Strip;
  coordinate_t Index = 0;

  RGBColor &color() const { return Strip.LEDs[Index]; }
  bool blinks() const { return Strip.blinks(Index); }
  void setBlinking() const { return Strip.setBlinking(Index); }
  void clearBlinking() const { return Strip.clearBlinking(Index); }

  void dump(bool Colorful) const {
    ConstLEDRef(Strip, Index).dump(Colorful);
  }

};

struct LEDArray {
public:
  using LEDRef = LEDRef;
  using ConstLEDRef = ConstLEDRef;

public:
  std::array<ConfiguredStrip, TheCoordinateSystem::panelsCount()> Strips;

public:

public:
  LEDArray() {
    for (coordinate_t Index = 0; Index < Strips.size(); ++Index) {
      Strips[Index].StripIndex = Index;
    }
  }

public:
  bool blinks() const {
    return std::any_of(Strips.begin(), Strips.end(), [] (const ConfiguredStrip &Strip) {
      return Strip.blinks();
    });
  }

private:
  ConstLEDRef led(size_t Column, size_t Line) const {
    LEDCoordinate Coordinate =
        TheCoordinateSystem::convert(Point{Column, Line});
    return { Strips[Coordinate.StripIndex], Coordinate.LEDIndex };
  }

  LEDRef led(size_t Column, size_t Line) {
    assert(Column < 80);
    assert(Line < 24);
    LEDCoordinate Coordinate =
        TheCoordinateSystem::convert(Point{Column, Line});
    return { Strips[Coordinate.StripIndex], Coordinate.LEDIndex };
  }


public:
  void set(size_t Column, size_t Line, const RGBColor &Color, bool Blink) {
    LEDRef LED = led(Column, Line);

#if 0
    log("LEDArray.set(Column: %d, Line: %d, Color: ", Column, Line);
    Color.dump(false, Blink);
    log(", StripIndex: %d, LED index: %d)\n", LED.Strip.Index, LED.Index);
#endif

    LED.color() = Color;

    if (Blink)
      LED.setBlinking();
    else
      LED.clearBlinking();
  }

public:
  void render(size_t Time) {
    Trace TT(event_ids::Render);
    renderImpl<0>(Time);
  }

  template <size_t J>
  void renderImpl(size_t Time) {
    if constexpr (J >= TheCoordinateSystem::panelsCount()) {
      return;
    } else {
      Trace TT(event_ids::RenderStrip, J);

      if (Strips[J].blinks()) {
        RGBColor &TargetColor = Strips[J].LEDs[Strips[J].Blinking];
        HSVColor Color = Strips[J].BlinkingColor.toHSVColor();

        constexpr uint8_t MinValue = 0;
        constexpr uint8_t MaxValue = 255;
        size_t ScaledTime = Time * 10;
        uint8_t ValueShift = ScaledTime % MaxValue;
        if ((ScaledTime / MaxValue) & 1)
          ValueShift = (MaxValue - 1) - ValueShift;
        ValueShift += MinValue;
        Color.Value = ValueShift;
        TargetColor = Color.toRGBColor();
      }

      TT.stop();

      renderImpl<J + 1>(Time);
    }
  }

public:
  void dump(bool Colorful) const {
    log("Strips:\n");
    for (coordinate_t StripIndex = 0; StripIndex < TheCoordinateSystem::panelsCount(); ++StripIndex) {
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

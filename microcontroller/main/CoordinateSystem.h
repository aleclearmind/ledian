#pragma once

#include <array>
#include <cstdlib>
#include <stdio.h>

using coordinate_t = size_t;

class Point {
public:
  coordinate_t Column = 0;
  coordinate_t Line = 0;

public:
  constexpr Point(coordinate_t Column, coordinate_t Line)
      : Column(Column), Line(Line) {}

public:
  constexpr bool operator==(const Point &Other) const {
    return Column == Other.Column and Line == Other.Line;
  }

  constexpr Point operator+(const Point &Other) const {
    Point Result = *this;
    Result.Column += Other.Column;
    Result.Line += Other.Line;
    return Result;
  }

  constexpr Point operator-(const Point &Other) const {
    Point Result = *this;
    Result.Column -= Other.Column;
    Result.Line -= Other.Line;
    return Result;
  }

public:
  constexpr coordinate_t indexInRectangle(coordinate_t Columns) {
    return Line * Columns + Column;
  }
};

class LEDCoordinate {
public:
  coordinate_t StripIndex = 0;
  coordinate_t LEDIndex = 0;

public:
  constexpr LEDCoordinate(coordinate_t StripIndex, coordinate_t LEDIndex)
      : StripIndex(StripIndex), LEDIndex(LEDIndex) {}

public:
  constexpr bool operator==(const LEDCoordinate &Other) const {
    return StripIndex == Other.StripIndex and LEDIndex == Other.LEDIndex;
  }
};

namespace Corner {
enum Values { NorthWest, NorthEast, SouthEast, SouthWest };

constexpr bool isWest(Values V) {
  switch (V) {
  case NorthWest:
  case SouthWest:
    return true;
  case NorthEast:
  case SouthEast:
    return false;
  default:
    abort();
  }
}

constexpr bool isEast(Values V) { return not isWest(V); }

constexpr bool isNorth(Values V) {
  switch (V) {
  case NorthWest:
  case NorthEast:
    return true;
  case SouthEast:
  case SouthWest:
    return false;
  default:
    abort();
  }
}

constexpr bool isSouth(Values V) { return not isNorth(V); }

} // namespace Corner

// Note: currently we assume that the strip is laid out zig-zaging horizonthally
// TODO: handle vertical
class Panel {
public:
  Corner::Values Start = Corner::NorthWest;
  coordinate_t StripIndex = 0;
  unsigned Skip = 0;

public:
  constexpr bool isLeftToRight(coordinate_t Line) const {
    return (true ^ Line) % 2 == 1;
    return (Corner::isWest(Start) ^ Line) % 2 == 1;
  }

  constexpr bool isTopToBottom() const {
    return true;
    return Corner::isNorth(Start);
  }
};

#if 0
static_assert(Panel{Corner::NorthWest}.isLeftToRight(0));
static_assert(not Panel{Corner::NorthWest}.isLeftToRight(1));
static_assert(Panel{Corner::SouthWest}.isLeftToRight(0));
static_assert(not Panel{Corner::SouthWest}.isLeftToRight(1));
static_assert(not Panel{Corner::NorthEast}.isLeftToRight(0));
static_assert(Panel{Corner::NorthEast}.isLeftToRight(1));
static_assert(not Panel{Corner::SouthEast}.isLeftToRight(0));
static_assert(Panel{Corner::SouthEast}.isLeftToRight(1));
#endif

template <coordinate_t PanelsCount = 0, coordinate_t PanelLines = 0,
          std::array<Panel, PanelsCount> Panels = std::array<Panel, PanelsCount>(),
          coordinate_t ColumnsPerPanel = 0, coordinate_t LinesPerPanel = 0>
class CoordinateSystem {
public:
  static constexpr coordinate_t panelsCount() { return PanelsCount; }
  static constexpr coordinate_t panelLines() { return PanelLines; }
  static constexpr coordinate_t panelColumns() {
    return PanelsCount / PanelLines;
  }
  static constexpr coordinate_t linesPerPanel() { return LinesPerPanel; }
  static constexpr coordinate_t columnsPerPanel() { return ColumnsPerPanel; }

  static constexpr coordinate_t lines() {
    return linesPerPanel() * panelLines();
  }
  static constexpr coordinate_t columns() {
    return columnsPerPanel() * panelColumns();
  }
  static constexpr coordinate_t cells() { return lines() * columns(); }
  static constexpr coordinate_t cellsPerPanel() { return ColumnsPerPanel * LinesPerPanel; }

public:
  static constexpr Point
  panelCoordinate(coordinate_t ColumnPanel, coordinate_t LinePanel,
                  Corner::Values Corner = Corner::NorthWest) {
    Point NorthWestCorner{ColumnPanel * columnsPerPanel(),
                          LinePanel * linesPerPanel()};
    switch (Corner) {
    case Corner::NorthWest:
      return NorthWestCorner;
    case Corner::NorthEast:
      return NorthWestCorner + Point(columnsPerPanel(), 0);
    case Corner::SouthEast:
      return NorthWestCorner + Point(columnsPerPanel(), linesPerPanel());
    case Corner::SouthWest:
      return NorthWestCorner + Point(0, linesPerPanel());
    default:
      abort();
    }
  }

  static constexpr std::pair<const Panel &, Point>
  findPanel(const Point &Point) {
    coordinate_t LinePanel = Point.Line / linesPerPanel();
    coordinate_t ColumnPanel = Point.Column / columnsPerPanel();
    coordinate_t PanelIndex = panelColumns() * LinePanel + ColumnPanel;
    return {Panels[PanelIndex],
            Point - panelCoordinate(ColumnPanel, LinePanel)};
  }

  static constexpr LEDCoordinate convert(const Point &ThePoint) {
    const auto &[Panel, InPanelPoint] = findPanel(ThePoint);

    coordinate_t CorrectLine =
        (Panel.isTopToBottom() ? InPanelPoint.Line
                               : linesPerPanel() - 1 - InPanelPoint.Line);

    coordinate_t CorrectColumn =
        (Panel.isLeftToRight(InPanelPoint.Line)
             ? InPanelPoint.Column
             : columnsPerPanel() - 1 - InPanelPoint.Column);

    return LEDCoordinate{
        Panel.StripIndex,
        Point(CorrectColumn, CorrectLine).indexInRectangle(columnsPerPanel()) -
            Panel.Skip};
  }

public:
  static constexpr bool verify() { return PanelsCount % PanelLines == 0; }
};

#if 0
static_assert(CoordinateSystem<1, 1, std::array<Panel, 1>{}, 40, 11>::verify());
static_assert(CoordinateSystem<2, 1, std::array<Panel, 2>{}, 40, 11>::verify());
static_assert(
    not CoordinateSystem<1, 2, std::array<Panel, 1>{}, 40, 11>::verify());

using MyPanel =
    CoordinateSystem<4, 2,
                     std::array<Panel, 4>{Panel{Corner::NorthWest, 0},
                                          Panel{Corner::NorthEast, 1},
                                          Panel{Corner::SouthWest, 2},
                                          Panel{Corner::SouthEast, 3}},
                     40, 11>;

static_assert(MyPanel::verify());

static_assert(MyPanel::panelCoordinate(0, 0) == Point(0, 0));
static_assert(MyPanel::panelCoordinate(1, 0) == Point(40, 0));
static_assert(MyPanel::panelCoordinate(0, 1) == Point(0, 11));
static_assert(MyPanel::panelCoordinate(1, 1) == Point(40, 11));

static_assert(MyPanel::findPanel(Point{0, 0}).first.StripIndex == 0);
static_assert(MyPanel::findPanel(Point{0, 0}).second == Point(0, 0));

static_assert(MyPanel::findPanel(Point{39, 0}).first.StripIndex == 0);
static_assert(MyPanel::findPanel(Point{39, 0}).second == Point(39, 0));

static_assert(MyPanel::findPanel(Point{40, 0}).first.StripIndex == 1);
static_assert(MyPanel::findPanel(Point{41, 0}).first.StripIndex == 1);

static_assert(MyPanel::findPanel(Point{79, 0}).first.StripIndex == 1);
static_assert(MyPanel::findPanel(Point{79, 0}).second == Point(39, 0));

static_assert(MyPanel::findPanel(Point{0, 10}).first.StripIndex == 0);
static_assert(MyPanel::findPanel(Point{0, 11}).first.StripIndex == 2);

static_assert(MyPanel::findPanel(Point{79, 10}).first.StripIndex == 1);
static_assert(MyPanel::findPanel(Point{79, 11}).first.StripIndex == 3);
static_assert(MyPanel::findPanel(Point{79, 21}).first.StripIndex == 3);
static_assert(MyPanel::findPanel(Point{79, 21}).second == Point(39, 10));

static_assert(MyPanel::convert(Point{0, 0}) == LEDCoordinate(0, 0));
static_assert(MyPanel::convert(Point{1, 0}) == LEDCoordinate(0, 1));

static_assert(not MyPanel::findPanel(Point{79, 0}).first.isLeftToRight(0));
static_assert(MyPanel::convert(Point{79, 0}) == LEDCoordinate(1, 0));
static_assert(MyPanel::convert(Point{78, 0}) == LEDCoordinate(1, 1));

static_assert(MyPanel::findPanel(Point{79, 1}).first.isLeftToRight(1));
static_assert(MyPanel::convert(Point{79, 1}) == LEDCoordinate(1, 40 + 40 - 1));
static_assert(MyPanel::convert(Point{78, 1}) ==
              LEDCoordinate(1, 40 + 40 - 1 - 1));

static_assert(MyPanel::convert(Point{0, 11 + 11 - 1}) == LEDCoordinate(2, 0));
static_assert(MyPanel::convert(Point{1, 11 + 11 - 1}) == LEDCoordinate(2, 1));
#endif
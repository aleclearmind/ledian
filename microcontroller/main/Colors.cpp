#include "Colors.h"

RGBColor HSVColor::toRGBColor() const {
  RGBColor Result;

  if (Saturation == 0) {
    Result.Red = Value;
    Result.Green = Value;
    Result.Blue = Value;
    return Result;
  }

  // converting to 16 bit to prevent overflow
  unsigned h = Hue;
  unsigned s = Saturation;
  unsigned v = Value;

  uint8_t region = h / 43;
  unsigned remainder = (h - (region * 43)) * 6;

  uint8_t p = (v * (255 - s)) >> 8;
  uint8_t q = (v * (255 - ((s * remainder) >> 8))) >> 8;
  uint8_t t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;

  switch (region) {
  case 0:
    Result.Red = v;
    Result.Green = t;
    Result.Blue = p;
    break;
  case 1:
    Result.Red = q;
    Result.Green = v;
    Result.Blue = p;
    break;
  case 2:
    Result.Red = p;
    Result.Green = v;
    Result.Blue = t;
    break;
  case 3:
    Result.Red = p;
    Result.Green = q;
    Result.Blue = v;
    break;
  case 4:
    Result.Red = t;
    Result.Green = p;
    Result.Blue = v;
    break;
  default:
    Result.Red = v;
    Result.Green = p;
    Result.Blue = q;
    break;
  }

  return Result;
}

HSVColor RGBColor::toHSVColor() const {
  HSVColor Result;

  uint8_t rgbMin =
      Red < Green ? (Red < Blue ? Red : Blue) : (Green < Blue ? Green : Blue);
  uint8_t rgbMax =
      Red > Green ? (Red > Blue ? Red : Blue) : (Green > Blue ? Green : Blue);

  Result.Value = rgbMax;
  if (Result.Value == 0) {
    Result.Hue = 0;
    Result.Saturation = 0;
    return Result;
  }

  Result.Saturation = 255 * ((long)(rgbMax - rgbMin)) / Result.Value;
  if (Result.Saturation == 0) {
    Result.Hue = 0;
    return Result;
  }

  if (rgbMax == Red)
    Result.Hue = 0 + 43 * (Green - Blue) / (rgbMax - rgbMin);
  else if (rgbMax == Green)
    Result.Hue = 85 + 43 * (Blue - Red) / (rgbMax - rgbMin);
  else
    Result.Hue = 171 + 43 * (Red - Green) / (rgbMax - rgbMin);

  return Result;
}

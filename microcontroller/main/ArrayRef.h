#pragma once

#include <stdlib.h>

template <typename T> struct ArrayRef {
  ArrayRef(T *Data, size_t Size) : Data(Data), Size(Size) {}

  T *Data;
  size_t Size;
};

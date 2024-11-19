#include "Common.h"

#include "Command.h"

namespace Command {

struct cursor_t {
  uint32_t Column;
  uint32_t Line;

  void operator++() { ++Column; }

  cursor_t operator+(size_t Columns) const {
    cursor_t Result = *this;
    Result.Column += Columns;
    assert(Result.verify());
    return Result;
  }

  bool verify() const {
    // TODO: fix
    return Column < 80 and Line < 22;
  }
};

enum class BufferType { FixedSize, Array };

class Context {
public:
  bool SaidHello = false;
  cursor_t WriteCursor;

  Context() : WriteCursor({0, 0}) {}
};

class Helo {
public:
  static constexpr const char *Name = "Helo";
  static constexpr char ID = 1;
  static constexpr BufferType Type = BufferType::FixedSize;
  using FixedType = std::array<uint8_t, 4>;

private:
  Context &C;

public:
  Helo(Context &C) : C(C) {}

  void parse(const FixedType *Object) {
    static FixedType Reference = {'H', 'E', 'L', 'O'};
    assert(0 == memcmp(Object, &Reference, sizeof(FixedType)));
    C.SaidHello = true;
  }
};

class UpdateRange {
public:
  static constexpr const char *Name = "UpdateRange";
  static constexpr char ID = 2;
  static constexpr BufferType Type = BufferType::Array;
  using ArrayType = LEDDescriptor;

private:
  Context &C;
  cursor_t LocalWriteCursor;

public:
  UpdateRange(Context &C) : C(C), LocalWriteCursor(C.WriteCursor) {
    assert(LocalWriteCursor.verify());
  }

public:
  void preparse(size_t Elements) {
    assert(C.SaidHello);
    assert((LocalWriteCursor + (Elements - 1)).verify());
  }

  void parseOne(const LEDDescriptor *Object) {
    assert(Object->verify());
    LEDs.working().set(LocalWriteCursor.Column, LocalWriteCursor.Line,
                       Object->Color.toRGBColor(), Object->Blink);
    ++LocalWriteCursor;
  }
};

class MoveCursor {
public:
  static constexpr const char *Name = "MoveCursor";
  static constexpr char ID = 3;
  static constexpr BufferType Type = BufferType::FixedSize;
  using FixedType = cursor_t;

private:
  Context &C;

public:
  MoveCursor(Context &C) : C(C) {}

  void parse(const cursor_t *Object) {
    assert(C.SaidHello);
    log("MoveCursor(Column: %ld, Line: %ld)\n", Object->Column, Object->Line);
    assert(Object->verify());
    C.WriteCursor = *Object;
  }
};

struct ConfigureMessage {
  uint8_t Debug = 0;

  bool verify() const { return Debug < 2; }
};

class Configure {
public:
  static constexpr const char *Name = "Configure";
  static constexpr char ID = 4;
  static constexpr BufferType Type = BufferType::FixedSize;
  using FixedType = ConfigureMessage;

private:
  Context &C;

public:
  Configure(Context &C) : C(C) {}

  void parse(const ConfigureMessage *Object) {
    assert(C.SaidHello);
    log("Configure(Debug: %ld)\n", Object->Debug);
    assert(Object->verify());

    EnableDebug = Object->Debug;
  }
};

Context C;
using identifier_t = uint8_t;
using length_t = uint32_t;

static constexpr size_t MaxReadSize = sizeof(cursor_t);

std::optional<ArrayRef<uint8_t>> read(size_t Size);

template <typename T> T *read() {
  auto MaybeData = read(sizeof(T));
  if (MaybeData)
    return reinterpret_cast<T *>(MaybeData->Data);
  else
    return nullptr;
}

template <typename T> void parseFixedSize(length_t Length) {
  Trace TT(event_ids::ParseFixedSize, Length);
  assert(sizeof(typename T::FixedType) == Length);
  T Instance(C);
  Instance.parse(read<typename T::FixedType>());
}

template <typename T> void parseArray(length_t Length) {
  Trace TT(event_ids::ParseArray, Length);
  constexpr size_t ElementSize = sizeof(typename T::ArrayType);
  assert(Length % ElementSize == 0);
  length_t Elements = Length / ElementSize;
  log("Got array of %ld elements\n", Elements);
  T Instance(C);
  {
    Trace TTT(event_ids::PreParse);
    Instance.preparse(Elements);
  }
  for (length_t I = 0; I < Elements; ++I) {
    Trace TTT(event_ids::ParseOne, I);
    Instance.parseOne(read<typename T::ArrayType>());
  }
}

template <typename T> void dispatch(length_t Length) {
  log("Got command %s\n", T::Name);

  if constexpr (T::Type == BufferType::FixedSize) {
    parseFixedSize<T>(Length);
  } else if constexpr (T::Type == BufferType::Array) {
    parseArray<T>(Length);
  } else {
    abort();
  }
}

std::optional<ArrayRef<uint8_t>> read(size_t Size) {
  Trace T(event_ids::Read, Size);
  assert(Size <= MaxReadSize);
  static uint8_t ReadBuffer[MaxReadSize];
  size_t ReadData = 0;
  uint8_t *ReadPointer = &ReadBuffer[0];

  while (ReadData < Size) {
    ReadData += fread(ReadPointer, 1, Size, stdin);
    ReadPointer += ReadData;
  }

  log("ReadData: %d\n", ReadData);
  fflush(stdout);


#if 0
  static size_t UnackedBytes = 0;
  UnackedBytes += ReadData;
  while (UnackedBytes >= 16) {
    puts("A16");
    UnackedBytes -= 16;
  }
#endif

  for (size_t I = 0; I < ReadData; ++I) {
    log("%.2x ", ReadBuffer[I]);
  }
  puts("\n");

  assert(ReadData == Size);

  return ArrayRef{&ReadBuffer[0], Size};
}

bool hasData() {
  Trace T(event_ids::HasData);
  // TODO
  return true;
}

bool parse() {
  Trace T(event_ids::Parse);

  identifier_t *MaybeIDPointer = read<identifier_t>();
  if (MaybeIDPointer != nullptr) {
    identifier_t ID = *MaybeIDPointer;
    length_t Length = *read<length_t>();
    log("ID: %d length: %ld\n", ID, Length);

    switch (ID) {
    case Helo::ID:
      log("Helo\n");
      dispatch<Helo>(Length);
      break;

    case UpdateRange::ID:
      log("UpdateRange\n");
      dispatch<UpdateRange>(Length);
      break;

    case MoveCursor::ID:
      log("MoveCursor\n");
      dispatch<MoveCursor>(Length);
      break;
    default:
      log("default\n");
      break;
    }

    puts("ACK");
    return true;
  }

  return false;
}

} // namespace Command

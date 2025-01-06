#pragma once

int timespec_get(struct timespec *ts, int base);
#include <chrono>

constexpr bool StaticEnableDebug = true;
inline bool EnableDebug = true;

template <typename... T> void log(const char *Format, T... Args) {
  if constexpr (not StaticEnableDebug) {
    return;
  }

  if (not EnableDebug)
    return;

  iprintf(Format, Args...);
  fflush(stdout);
}

using ledian_clock = std::chrono::high_resolution_clock;
inline decltype(ledian_clock::now()) start_time;

namespace event_ids {
enum Values {
  Invalid,
  Main,
  BlinkLED,
  InitialSetup,
  MainLoopIteration,
  Read,
  ParseFixedSize,
  ParseArray,
  PreParse,
  ParseOne,
  HasData,
  Parse,
  Render,
  RenderStrip,
  AdjustBlinking,
  FlushBuffer
};

inline const char *getName(Values V) {
  switch (V) {
  case Invalid:
    return "Invalid";
  case Main:
    return "Main";
  case BlinkLED:
    return "BlinkLED";
  case InitialSetup:
    return "InitialSetup";
  case MainLoopIteration:
    return "MainLoopIteration";
  case Read:
    return "Read";
  case ParseFixedSize:
    return "ParseFixedSize";
  case ParseArray:
    return "ParseArray";
  case PreParse:
    return "PreParse";
  case ParseOne:
    return "ParseOne";
  case HasData:
    return "HasData";
  case Parse:
    return "Parse";
  case Render:
    return "Render";
  case RenderStrip:
    return "RenderStrip";
  case AdjustBlinking:
    return "AdjustBlinking";
  case FlushBuffer:
    return "FlushBuffer";
  default:
    abort();
    break;
  }
}

} // namespace event_ids

inline long long last_event_tick = 0;
inline long long last_emitted_event_tick = 0;

template <size_t expected_size, size_t event_id_bits, size_t arg_bits,
          size_t ts_bits>
struct TraceEntryImpl {
  struct Data {
    int start : 1 = 0;
    int event_id : event_id_bits = 0;
    int arg : arg_bits = 0;

    using ts_type = std::conditional_t<(ts_bits > 32), long long, int>;
    ts_type ts : ts_bits = 0;
  };
  static_assert(expected_size == sizeof(Data) * 8);

  Data data;

  event_ids::Values event_id() const {
    return static_cast<event_ids::Values>(data.event_id);
  }

  static TraceEntryImpl make(bool start, int event_id, int arg) {
    TraceEntryImpl result;
    result.data.start = start ? 1 : 0;

    assert(event_id < (1LL << event_id_bits));
    result.data.event_id = event_id;

    assert(arg < (1LL << arg_bits));
    result.data.arg = arg;

    // TODO: get ticks
    long long tick = 0;
    long long tick_delta = tick - last_event_tick;

    if (tick_delta >= (1LL << ts_bits)) {
      result.data.ts = (1LL << ts_bits) - 1;
    } else {
      result.data.ts = tick_delta;
    }

    last_event_tick = tick;

    return result;
  }
};

using TraceEntry = TraceEntryImpl<128, 31, 32, 64>;
// using TraceEntry = TraceEntryImpl<64, 7, 24, 32>;
// using TraceEntry = TraceEntryImpl<32, 7, 8, 16>;
inline std::array<TraceEntry, 1> trace_buffer;
inline TraceEntry *free_trace_entry = nullptr;

class Trace {
#define PREFIX "TRACE: "
private:
  uint8_t event_id = 0;
  uint32_t arg = 0;

public:
  Trace(uint8_t event_id, size_t arg = 0) : event_id(event_id), arg(arg) {
    emit(true);
    (void) this->arg;
  }

  ~Trace() { stop(); }

  void stop() {
    if (event_id != 0xFF) {
      emit(false);
      event_id = 0xFF;
    }
  }

private:
  void emit(bool start) {
#if 0
    if (free_trace_entry == trace_buffer.end()) {
      for (unsigned I = 0; I < trace_buffer.size(); ++I) {
        const TraceEntry &Entry = trace_buffer[I];
        log(PREFIX "{\"name\": \"%s ", event_ids::getName(Entry.event_id()));
        log("(%ld)\", ", Entry.data.arg);
        log("\"cat\": \"task\", \"ph\": \"%s\", ", Entry.data.start ? "B" : "E");
        last_emitted_event_tick += Entry.data.ts;
        log("\"ts\": %lld, \"pid\": 1, \"tid\": 1},\n", last_emitted_event_tick);
      }
      
      free_trace_entry = trace_buffer.begin();
    }

    *free_trace_entry = TraceEntry::make(start, event_id, arg);

    ++free_trace_entry;
#endif
  }

#undef PREFIX
};

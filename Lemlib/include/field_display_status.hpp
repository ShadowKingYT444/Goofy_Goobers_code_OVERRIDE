#pragma once
#include <cstdint>
namespace fieldviz {
// Task-safe status publisher; never draws or calls LVGL from the caller.
bool print(std::int16_t line, const char* format, ...);
// Multi-line result panel over the readout column; "" hides it again.
void report(const char* format, ...);
}

#include <cstdio>
#include <string>
#include <cstdint>

namespace pros {
namespace lcd {

using lcd_btn_cb_fn_t = void (*)();

namespace {
bool lcd_initialized = false;
}  // namespace

void initialize() {
  lcd_initialized = true;
}

bool is_initialized() {
  return lcd_initialized;
}

void shutdown() {
  lcd_initialized = false;
}

bool clear() {
  // Compatibility stub for older EZ-Template symbols.
  return true;
}

bool clear_line(std::int16_t) {
  // Compatibility stub for older EZ-Template symbols.
  return true;
}

void register_btn0_cb(lcd_btn_cb_fn_t) {
  // Compatibility stub for older EZ-Template symbols.
}

void register_btn2_cb(lcd_btn_cb_fn_t) {
  // Compatibility stub for older EZ-Template symbols.
}

bool set_text(std::int16_t, const char*) {
  // Compatibility stub for older EZ-Template symbols.
  return true;
}

bool set_text(std::int16_t, std::string text) {
  // Compatibility stub for older EZ-Template symbols.
  (void)text;
  return true;
}

bool set_text(std::int16_t, int) {
  // Compatibility stub for older EZ-Template symbols.
  return true;
}

}  // namespace lcd
}  // namespace pros

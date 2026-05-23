// TimeSync (NTP) + NeoFx (WS2812 event animations).
// Both are small infrastructure modules used by the wireless features.

#include <Arduino.h>
#include <WiFi.h>
#include <time.h>
#include <Adafruit_NeoPixel.h>

#include "config.h"
#include "shared.h"
#include "SettingsStore.h"

// ─────────────────────────────────────────────────────────────────────────────
namespace TimeSync {

static bool s_set = false;

bool tryNtp(uint32_t timeoutMs) {
  if (WiFi.status() != WL_CONNECTED) return false;
  // UTC; NTP pool servers. configTime(int, int, ...) takes gmt + dst offsets.
  configTime(0, 0, "pool.ntp.org", "time.google.com", "time.cloudflare.com");
  uint32_t t0 = millis();
  while ((millis() - t0) < timeoutMs) {
    time_t now = time(nullptr);
    if (now > 1700000000) {       // any time after 2023-11
      s_set = true;
      return true;
    }
    delay(100);
  }
  return false;
}

bool isSet() {
  if (s_set) return true;
  time_t now = time(nullptr);
  return now > 1700000000;
}

void utcNow(char* out, size_t cap) {
  if (!isSet()) { snprintf(out, cap, "--:--:--"); return; }
  time_t now = time(nullptr);
  struct tm t; gmtime_r(&now, &t);
  snprintf(out, cap, "%02d:%02d:%02d", t.tm_hour, t.tm_min, t.tm_sec);
}

void utcDateNow(char* out, size_t cap) {
  if (!isSet()) { snprintf(out, cap, "--/--/--"); return; }
  time_t now = time(nullptr);
  struct tm t; gmtime_r(&now, &t);
  snprintf(out, cap, "%02d/%02d/%02d", t.tm_mday, t.tm_mon + 1, t.tm_year % 100);
}

uint32_t epoch() {
  if (!isSet()) return 0;
  return (uint32_t)time(nullptr);
}

}  // namespace TimeSync

// ─────────────────────────────────────────────────────────────────────────────
// NeoFx — 4-LED WS2812 strip on GPIO 1.
namespace NeoFx {

#ifndef NEOPIXEL_PIN
#define NEOPIXEL_PIN 1
#endif
#ifndef NEOPIXEL_COUNT
#define NEOPIXEL_COUNT 4
#endif

static Adafruit_NeoPixel strip(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
static bool     s_started     = false;
static bool     s_enabled     = true;
static uint32_t s_moodColor   = 0x100018;   // soft Arasaka magenta default
static uint32_t s_phaseStartMs= 0;
static uint32_t s_phaseDurMs  = 0;
static Event    s_activeEvt   = Event::BootSplash;
static bool     s_eventActive = false;

static void allOff() {
  for (int i = 0; i < NEOPIXEL_COUNT; i++) strip.setPixelColor(i, 0);
  strip.show();
}

void begin() {
  if (s_started) return;
  strip.begin();
  strip.setBrightness(60);   // keep modest; 4 LEDs at full white draw ~240 mA
  allOff();
  s_started = true;
}

void setEnabled(bool en) {
  s_enabled = en;
  if (!s_enabled && s_started) allOff();
}

void setMoodBreath(uint32_t color) {
  s_moodColor = color;
}

void event(Event e) {
  if (!s_started || !s_enabled) return;
  s_activeEvt   = e;
  s_phaseStartMs= millis();
  s_eventActive = true;
  switch (e) {
    case Event::Capture:      s_phaseDurMs = 900;   break;   // 3 flashes
    case Event::Deauth:       s_phaseDurMs = 1200;  break;   // strobe burst
    case Event::Friend:       s_phaseDurMs = 700;   break;   // pulse
    case Event::GpsSearching: s_phaseDurMs = 0;     break;   // sticky idle pattern
    case Event::GpsFix:       s_phaseDurMs = 1000;  break;
    case Event::BootSplash:   s_phaseDurMs = 1500;  break;
  }
}

// Drive a frame for the active event/idle pattern.
static void drawFrame(uint32_t now) {
  if (!s_eventActive) {
    // Idle: gentle breathing of moodColor.
    float phase = (now % 4000) / 4000.0f;
    float lum   = 0.20f + 0.30f * (0.5f - 0.5f * cosf(phase * 6.28318f));
    uint8_t r = (uint8_t)(((s_moodColor >> 16) & 0xFF) * lum);
    uint8_t g = (uint8_t)(((s_moodColor >>  8) & 0xFF) * lum);
    uint8_t b = (uint8_t)(( s_moodColor        & 0xFF) * lum);
    uint32_t c = strip.Color(r, g, b);
    for (int i = 0; i < NEOPIXEL_COUNT; i++) strip.setPixelColor(i, c);
    strip.show();
    return;
  }

  uint32_t dt = now - s_phaseStartMs;
  if (s_phaseDurMs > 0 && dt >= s_phaseDurMs) {
    s_eventActive = false;
    return;
  }

  switch (s_activeEvt) {
    case Event::Capture: {
      // Triple green flash: 3 cycles of on/off across the strip.
      bool on = ((dt / 150) % 2) == 0;
      uint32_t c = on ? strip.Color(0, 220, 40) : 0;
      for (int i = 0; i < NEOPIXEL_COUNT; i++) strip.setPixelColor(i, c);
      break;
    }
    case Event::Deauth: {
      // Red strobe — alternating LEDs.
      bool flip = ((dt / 80) % 2) == 0;
      for (int i = 0; i < NEOPIXEL_COUNT; i++) {
        bool on = flip ? (i % 2 == 0) : (i % 2 == 1);
        strip.setPixelColor(i, on ? strip.Color(255, 12, 12) : 0);
      }
      break;
    }
    case Event::Friend: {
      // Single blue pulse rising then falling.
      float t = (float)dt / (float)s_phaseDurMs;
      float lum = (t < 0.5f) ? (t * 2.0f) : (1.0f - (t - 0.5f) * 2.0f);
      uint8_t v = (uint8_t)(255.0f * lum);
      uint32_t c = strip.Color(20, 60, v);
      for (int i = 0; i < NEOPIXEL_COUNT; i++) strip.setPixelColor(i, c);
      break;
    }
    case Event::GpsFix: {
      // Single bright white flash that fades to off.
      float t = 1.0f - ((float)dt / (float)s_phaseDurMs);
      uint8_t v = (uint8_t)(255.0f * t);
      uint32_t c = strip.Color(v, v, v);
      for (int i = 0; i < NEOPIXEL_COUNT; i++) strip.setPixelColor(i, c);
      break;
    }
    case Event::GpsSearching: {
      // Travelling amber dot.
      int pos = (dt / 220) % NEOPIXEL_COUNT;
      for (int i = 0; i < NEOPIXEL_COUNT; i++)
        strip.setPixelColor(i, i == pos ? strip.Color(255, 120, 0) : 0);
      break;
    }
    case Event::BootSplash: {
      // Red wave once across the strip.
      float t = (float)dt / (float)s_phaseDurMs;
      int head = (int)(t * (NEOPIXEL_COUNT + 2));
      for (int i = 0; i < NEOPIXEL_COUNT; i++) {
        int dist = abs(i - head);
        uint8_t v = (dist < 2) ? (uint8_t)(220 - dist * 80) : 0;
        strip.setPixelColor(i, strip.Color(v, 0, 0));
      }
      break;
    }
  }
  strip.show();
}

void tick() {
  if (!s_started) return;
  if (!s_enabled) return;
  uint32_t now = millis();
  static uint32_t lastFrame = 0;
  if ((uint32_t)(now - lastFrame) < 40) return;    // ~25 fps cap
  lastFrame = now;
  drawFrame(now);
}

}  // namespace NeoFx

#include "KeyboardUI.h"
#include "Touchscreen.h"
#include "utils.h"

// ─── On-screen keyboard (rev 2) ──────────────────────────────────────────────
// Proper QWERTY layout with SHIFT (caps lock), SYM (symbol layer), real SPACE
// and BACKSPACE keys. Public API in KeyboardUI.h is unchanged — cfg.rows is
// now ignored in favour of internal layouts, but cfg.titleLine1/2, maxLen,
// requireNonEmpty, backLabel/middleLabel/okLabel, and buttonsY are honored.

namespace {

#ifndef FEATURE_TEXT
#define FEATURE_TEXT ORANGE
#endif

static inline uint16_t KB_BG()       { return FEATURE_BG; }
static inline uint16_t KB_SURF()     { return UI_FG; }
static inline uint16_t KB_BORDER()   { return FEATURE_TEXT; }
static inline uint16_t KB_TEXT()     { return WHITE; }
static inline uint16_t KB_WARN()     { return UI_WARN; }
static inline uint16_t KB_ACCENT()   { return FEATURE_TEXT; }
static inline uint16_t KB_ACCENT_BG(){ return 0x6020; }    // dim red highlight
static inline uint16_t KB_FUNC()     { return UI_LINE; }   // function-key surface

constexpr int INPUT_X      = 8;
constexpr int INPUT_Y      = 52;
constexpr int INPUT_W      = 224;
constexpr int INPUT_H      = 26;

constexpr int KEY_W        = 22;
constexpr int KEY_H        = 24;
constexpr int KEY_GAP      = 2;
constexpr int KEY_ROW_DY   = KEY_H + KEY_GAP;
constexpr int KEY_START_Y  = 118;

constexpr unsigned long CURSOR_BLINK_MS = 500;
constexpr unsigned long TAP_FLASH_MS    = 70;
constexpr unsigned long KB_DEBOUNCE_MS     = 90;

// Internal layouts. Letters row-by-row; the digit row is constant.
static const char DIGITS[]      = "1234567890";
static const char ROW1_LOWER[]  = "qwertyuiop";
static const char ROW1_UPPER[]  = "QWERTYUIOP";
static const char ROW1_SYM[]    = "!@#$%^&*()";
static const char ROW2_LOWER[]  = "asdfghjkl";
static const char ROW2_UPPER[]  = "ASDFGHJKL";
static const char ROW2_SYM[]    = "-_+=/\\?.:";  // 9 chars
static const char ROW3_LOWER[]  = "zxcvbnm";
static const char ROW3_UPPER[]  = "ZXCVBNM";
static const char ROW3_SYM[]    = ";'\"[]{}";    // 7 chars

enum class Layer : uint8_t { Lower, Upper, Symbols };
static Layer s_layer = Layer::Lower;

static const char* row1Chars() {
  return s_layer == Layer::Upper ? ROW1_UPPER
       : s_layer == Layer::Symbols ? ROW1_SYM
       : ROW1_LOWER;
}
static const char* row2Chars() {
  return s_layer == Layer::Upper ? ROW2_UPPER
       : s_layer == Layer::Symbols ? ROW2_SYM
       : ROW2_LOWER;
}
static const char* row3Chars() {
  return s_layer == Layer::Upper ? ROW3_UPPER
       : s_layer == Layer::Symbols ? ROW3_SYM
       : ROW3_LOWER;
}

struct KeyRect { int x, y, w, h; };

// Geometry helpers.
static KeyRect digitRect(int col) {
  return { 1 + col * (KEY_W + KEY_GAP), KEY_START_Y + 0 * KEY_ROW_DY, KEY_W, KEY_H };
}
static KeyRect row1Rect(int col) {
  return { 1 + col * (KEY_W + KEY_GAP), KEY_START_Y + 1 * KEY_ROW_DY, KEY_W, KEY_H };
}
static KeyRect row2Rect(int col) {
  // 9 keys, centered.
  int offset = (240 - (9 * KEY_W + 8 * KEY_GAP)) / 2;
  return { offset + col * (KEY_W + KEY_GAP), KEY_START_Y + 2 * KEY_ROW_DY, KEY_W, KEY_H };
}
// Row 3: SHIFT (32) | 7 letters (22) | BACKSPACE (32)
static constexpr int R3_SHIFT_W = 32;
static constexpr int R3_BS_W    = 32;
static KeyRect r3ShiftRect() {
  return { 1, KEY_START_Y + 3 * KEY_ROW_DY, R3_SHIFT_W, KEY_H };
}
static KeyRect r3LetterRect(int col) {
  int x0 = 1 + R3_SHIFT_W + KEY_GAP;
  return { x0 + col * (KEY_W + KEY_GAP), KEY_START_Y + 3 * KEY_ROW_DY, KEY_W, KEY_H };
}
static KeyRect r3BackspaceRect() {
  int x0 = 1 + R3_SHIFT_W + KEY_GAP + 7 * (KEY_W + KEY_GAP);
  return { x0, KEY_START_Y + 3 * KEY_ROW_DY, R3_BS_W, KEY_H };
}
// Row 4: SYM (44) | SPACE (~140) | ENTER (44)
static constexpr int R4_SYM_W   = 44;
static constexpr int R4_ENTER_W = 44;
static KeyRect r4SymRect() {
  return { 1, KEY_START_Y + 4 * KEY_ROW_DY, R4_SYM_W, KEY_H };
}
static KeyRect r4SpaceRect() {
  int x0 = 1 + R4_SYM_W + KEY_GAP;
  int x1 = 240 - 1 - R4_ENTER_W - KEY_GAP;
  return { x0, KEY_START_Y + 4 * KEY_ROW_DY, x1 - x0, KEY_H };
}
static KeyRect r4EnterRect() {
  return { 240 - 1 - R4_ENTER_W, KEY_START_Y + 4 * KEY_ROW_DY, R4_ENTER_W, KEY_H };
}

static bool inRect(const KeyRect& r, int x, int y) {
  return x >= r.x && x <= r.x + r.w && y >= r.y && y <= r.y + r.h;
}

/** Wardriver / FeatureUI expect font 1 + size 1; keyboard uses size 2 for input. */
static void restoreTftAfterOsKeyboard() {
  tft.setTextSize(1);
  tft.setTextFont(1);
  tft.setTextDatum(TL_DATUM);
}

static void drawInputField(const String& value, bool cursorOn) {
  tft.fillRect(INPUT_X, INPUT_Y, INPUT_W, INPUT_H, KB_SURF());
  tft.drawRect(INPUT_X - 1, INPUT_Y - 1, INPUT_W + 2, INPUT_H + 2, KB_BORDER());
  tft.setTextColor(KB_TEXT(), KB_SURF());
  tft.setTextSize(2);
  tft.setTextDatum(TL_DATUM);
  // Truncate from the front so the end (where the cursor lives) is always visible.
  String display = value;
  if (cursorOn) display += "|";
  while (tft.textWidth(display) > INPUT_W - 12 && display.length() > 1) {
    display.remove(0, 1);
  }
  tft.setCursor(INPUT_X + 5, INPUT_Y + 6);
  tft.print(display);
}

static void drawTitles(const OnScreenKeyboardConfig& cfg) {
  tft.setTextSize(1);
  tft.setTextFont(1);
  tft.setTextColor(KB_BORDER(), KB_BG());
  if (cfg.titleLine1 && cfg.titleLine1[0]) {
    tft.fillRect(0, INPUT_Y + INPUT_H + 4, 240, 12, KB_BG());
    tft.setCursor(8, INPUT_Y + INPUT_H + 5);
    tft.print(cfg.titleLine1);
  }
  if (cfg.titleLine2 && cfg.titleLine2[0]) {
    tft.fillRect(0, INPUT_Y + INPUT_H + 18, 240, 12, KB_BG());
    tft.setCursor(8, INPUT_Y + INPUT_H + 19);
    tft.print(cfg.titleLine2);
  }
}

static void drawCharKey(const KeyRect& r, char c, bool highlight) {
  uint16_t bg = highlight ? KB_BORDER() : KB_SURF();
  uint16_t fg = highlight ? KB_SURF()   : KB_TEXT();
  tft.fillRoundRect(r.x, r.y, r.w, r.h, 3, bg);
  tft.drawRoundRect(r.x, r.y, r.w, r.h, 3, KB_BORDER());
  tft.setTextSize(1);
  tft.setTextFont(2);
  tft.setTextColor(fg, bg);
  tft.setTextDatum(MC_DATUM);
  char s[2] = {c, 0};
  tft.drawString(s, r.x + r.w / 2, r.y + r.h / 2);
  tft.setTextDatum(TL_DATUM);
}

static void drawFuncKey(const KeyRect& r, const char* label, bool active) {
  uint16_t bg = active ? KB_ACCENT_BG() : KB_FUNC();
  uint16_t fg = active ? KB_TEXT()      : KB_TEXT();
  tft.fillRoundRect(r.x, r.y, r.w, r.h, 3, bg);
  tft.drawRoundRect(r.x, r.y, r.w, r.h, 3, active ? KB_ACCENT() : KB_BORDER());
  tft.setTextSize(1);
  tft.setTextFont(2);
  tft.setTextColor(fg, bg);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(label, r.x + r.w / 2, r.y + r.h / 2);
  tft.setTextDatum(TL_DATUM);
}

static void drawKeyboardKeys() {
  // Clear keyboard area.
  tft.fillRect(0, KEY_START_Y - 1, 240, 5 * KEY_ROW_DY + 1, KB_BG());

  // Row 0 — digits
  for (int i = 0; i < 10; i++) drawCharKey(digitRect(i), DIGITS[i], false);

  // Row 1 — qwertyuiop
  const char* r1 = row1Chars();
  for (int i = 0; i < 10; i++) drawCharKey(row1Rect(i), r1[i], false);

  // Row 2 — asdfghjkl (or symbols)
  const char* r2 = row2Chars();
  for (int i = 0; i < 9; i++) drawCharKey(row2Rect(i), r2[i], false);

  // Row 3 — SHIFT + zxcvbnm + BS
  drawFuncKey(r3ShiftRect(),  s_layer == Layer::Upper ? "SHFT" : "shft",
              s_layer == Layer::Upper);
  const char* r3 = row3Chars();
  for (int i = 0; i < 7; i++) drawCharKey(r3LetterRect(i), r3[i], false);
  drawFuncKey(r3BackspaceRect(), "<-", false);

  // Row 4 — SYM / SPACE / ENTER
  drawFuncKey(r4SymRect(),
              s_layer == Layer::Symbols ? "abc" : "?123",
              s_layer == Layer::Symbols);
  KeyRect spc = r4SpaceRect();
  tft.fillRoundRect(spc.x, spc.y, spc.w, spc.h, 3, KB_SURF());
  tft.drawRoundRect(spc.x, spc.y, spc.w, spc.h, 3, KB_BORDER());
  tft.setTextSize(1);
  tft.setTextFont(2);
  tft.setTextColor(KB_TEXT(), KB_SURF());
  tft.setTextDatum(MC_DATUM);
  tft.drawString("space", spc.x + spc.w / 2, spc.y + spc.h / 2);
  tft.setTextDatum(TL_DATUM);
  drawFuncKey(r4EnterRect(), "OK", false);
}

static void drawButtons(const OnScreenKeyboardConfig& cfg) {
  const int btnY = cfg.buttonsY;
  if (btnY <= 0) return;

  tft.setTextSize(1);
  tft.setTextFont(2);
  tft.setTextDatum(MC_DATUM);

  // Back (left).
  tft.fillRoundRect(5, btnY, 70, 25, 5, KB_FUNC());
  tft.drawRoundRect(5, btnY, 70, 25, 5, KB_BORDER());
  tft.setTextColor(KB_TEXT(), KB_FUNC());
  tft.drawString(cfg.backLabel ? cfg.backLabel : "Back", 40, btnY + 13);

  // Clear (middle).
  tft.fillRoundRect(85, btnY, 70, 25, 5, KB_FUNC());
  tft.drawRoundRect(85, btnY, 70, 25, 5, KB_BORDER());
  tft.setTextColor(KB_TEXT(), KB_FUNC());
  tft.drawString("Clear", 120, btnY + 13);

  // OK (right).
  tft.fillRoundRect(165, btnY, 70, 25, 5, KB_ACCENT_BG());
  tft.drawRoundRect(165, btnY, 70, 25, 5, KB_ACCENT());
  tft.setTextColor(KB_TEXT(), KB_ACCENT_BG());
  tft.drawString(cfg.okLabel ? cfg.okLabel : "OK", 200, btnY + 13);

  tft.setTextDatum(TL_DATUM);
}

static void showEmptyError(const OnScreenKeyboardConfig& cfg) {
  if (!cfg.emptyErrorMsg || !cfg.emptyErrorMsg[0]) return;
  tft.fillRect(INPUT_X, INPUT_Y + INPUT_H + 2, INPUT_W, 12, KB_BG());
  tft.setTextSize(1);
  tft.setTextFont(1);
  tft.setTextColor(KB_WARN(), KB_BG());
  tft.setCursor(INPUT_X + 4, INPUT_Y + INPUT_H + 3);
  tft.print(cfg.emptyErrorMsg);
}

// Brief invert-flash on key press for tactile feedback.
static void flashKey(const KeyRect& r) {
  tft.fillRoundRect(r.x, r.y, r.w, r.h, 3, KB_BORDER());
  delay(TAP_FLASH_MS);
}

}  // namespace

OnScreenKeyboardResult showOnScreenKeyboard(const OnScreenKeyboardConfig& cfg,
                                            const String& initial) {
  OnScreenKeyboardResult res;
  res.text      = initial;
  res.accepted  = false;
  res.cancelled = false;

  // Reset layer to lowercase for each invocation so behavior is predictable.
  s_layer = Layer::Lower;

  tft.fillRect(0, 37, tft.width(), tft.height() - 37, KB_BG());

  bool cursorOn = true;
  unsigned long lastBlink = millis();

  drawTitles(cfg);
  drawInputField(res.text, cursorOn);
  drawKeyboardKeys();
  drawButtons(cfg);

  unsigned long lastTouchMs = 0;
  bool wasTouched = false;

  while (true) {
    unsigned long now = millis();

    if (now - lastBlink >= CURSOR_BLINK_MS) {
      cursorOn = !cursorOn;
      drawInputField(res.text, cursorOn);
      lastBlink = now;
    }

    int tx, ty;
    bool touched = readTouchXY(tx, ty);

    // Edge-trigger on a fresh tap, with debounce so a held finger doesn't spam.
    if (!touched) { wasTouched = false; delay(8); continue; }
    if (wasTouched && (now - lastTouchMs) < KB_DEBOUNCE_MS) { delay(8); continue; }
    wasTouched = true;
    lastTouchMs = now;

    // --- Bottom Back/Clear/OK row ---
    const int btnY = cfg.buttonsY;
    if (btnY > 0 && ty >= btnY && ty <= btnY + 25) {
      if (tx >= 5 && tx <= 75) {
        res.cancelled = true;
        res.accepted  = false;
        restoreTftAfterOsKeyboard();
        return res;
      }
      if (tx >= 85 && tx <= 155) {
        // Clear the input.
        res.text = "";
        drawInputField(res.text, cursorOn);
        continue;
      }
      if (tx >= 165 && tx <= 235) {
        if (cfg.requireNonEmpty && res.text.length() == 0) {
          showEmptyError(cfg);
          continue;
        }
        res.accepted  = true;
        restoreTftAfterOsKeyboard();
        return res;
      }
    }

    // --- Function keys first (row 3 SHIFT/BS, row 4 SYM/SPACE/ENTER) ---
    KeyRect rShift = r3ShiftRect();
    if (inRect(rShift, tx, ty)) {
      flashKey(rShift);
      s_layer = (s_layer == Layer::Upper) ? Layer::Lower : Layer::Upper;
      drawKeyboardKeys();
      continue;
    }
    KeyRect rBs = r3BackspaceRect();
    if (inRect(rBs, tx, ty)) {
      flashKey(rBs);
      if (res.text.length() > 0) res.text.remove(res.text.length() - 1);
      drawInputField(res.text, cursorOn);
      drawKeyboardKeys();
      continue;
    }
    KeyRect rSym = r4SymRect();
    if (inRect(rSym, tx, ty)) {
      flashKey(rSym);
      s_layer = (s_layer == Layer::Symbols) ? Layer::Lower : Layer::Symbols;
      drawKeyboardKeys();
      continue;
    }
    KeyRect rSpc = r4SpaceRect();
    if (inRect(rSpc, tx, ty)) {
      flashKey(rSpc);
      if (res.text.length() < cfg.maxLen) res.text += ' ';
      drawInputField(res.text, cursorOn);
      drawKeyboardKeys();
      continue;
    }
    KeyRect rEnt = r4EnterRect();
    if (inRect(rEnt, tx, ty)) {
      flashKey(rEnt);
      if (cfg.requireNonEmpty && res.text.length() == 0) {
        showEmptyError(cfg);
        continue;
      }
      res.accepted = true;
      restoreTftAfterOsKeyboard();
      return res;
    }

    // --- Character rows ---
    auto tryHit = [&](const KeyRect& r, char c) -> bool {
      if (!inRect(r, tx, ty)) return false;
      flashKey(r);
      if (res.text.length() < cfg.maxLen) res.text += c;
      drawCharKey(r, c, false);            // restore key visual
      drawInputField(res.text, cursorOn);
      return true;
    };

    bool consumed = false;
    for (int i = 0; i < 10 && !consumed; i++) consumed = tryHit(digitRect(i), DIGITS[i]);
    const char* r1 = row1Chars();
    for (int i = 0; i < 10 && !consumed; i++) consumed = tryHit(row1Rect(i), r1[i]);
    const char* r2 = row2Chars();
    for (int i = 0; i < 9  && !consumed; i++) consumed = tryHit(row2Rect(i), r2[i]);
    const char* r3 = row3Chars();
    for (int i = 0; i < 7  && !consumed; i++) consumed = tryHit(r3LetterRect(i), r3[i]);
    // Auto-pop symbols back to lowercase after one symbol is typed (sticky shift off).
    if (consumed && s_layer == Layer::Symbols) {
      // keep symbols mode — more useful for typing URLs etc.
    }
  }
}

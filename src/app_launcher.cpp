// AppLauncher — Dark-Div's built-in multi-firmware loader.
//
// Lists ESP32 .bin files under /apps/ on the SD card, lets the user pick one,
// writes it to the next OTA slot via the Arduino Update library, sets that
// slot active, and restarts. Same effect as m5launcher / bmorcelli's Launcher
// project, just running inside Dark-Div instead of as a separate firmware.
//
// .bin format is the standard ESP32 application image (starts with 0xE9).
// No special wrapping — drop a "marauder.bin" / "bruce.bin" / etc. into
// /apps/ on the SD and they'll show up.
//
// LEFT button on the list view dumps the currently-running Dark-Div firmware
// to /apps/dark-div.bin so the user can always reflash back.

#include <Arduino.h>
#include <SD.h>
#include <Update.h>
#include <esp_partition.h>
#include <esp_ota_ops.h>

#include "config.h"
#include "shared.h"
#include "utils.h"

namespace AppLauncher {

static constexpr const char* APPS_DIR = "/apps";
static constexpr int MAX_APPS = 24;

struct AppEntry {
  char     name[40];
  uint32_t size;
};

static AppEntry apps[MAX_APPS];
static int      appCount = 0;
static int      selIndex = 0;
static int      scroll   = 0;
static bool     uiDirty  = true;

static const uint16_t L_RED = 0xE0E6;
static const uint16_t L_DIM = 0x6020;
static const uint16_t L_GRN = 0x07C0;

// --- SD scan ---------------------------------------------------------------
static int compareName(const void* a, const void* b) {
  return strcasecmp(((const AppEntry*)a)->name, ((const AppEntry*)b)->name);
}

static void scanApps() {
  appCount = 0;
  if (!isSDCardAvailable()) return;
  if (!SD.exists(APPS_DIR)) SD.mkdir(APPS_DIR);

  File dir = SD.open(APPS_DIR);
  if (!dir || !dir.isDirectory()) return;
  while (appCount < MAX_APPS) {
    File f = dir.openNextFile();
    if (!f) break;
    if (f.isDirectory()) { f.close(); continue; }
    const char* fname = f.name();
    const char* leaf = strrchr(fname, '/');
    leaf = leaf ? leaf + 1 : fname;
    size_t n = strlen(leaf);
    if (n < 5 || strcasecmp(leaf + n - 4, ".bin") != 0) { f.close(); continue; }
    strncpy(apps[appCount].name, leaf, sizeof(apps[appCount].name) - 1);
    apps[appCount].name[sizeof(apps[appCount].name) - 1] = 0;
    apps[appCount].size = (uint32_t)f.size();
    appCount++;
    f.close();
  }
  dir.close();
  if (appCount > 1) qsort(apps, appCount, sizeof(AppEntry), compareName);
}

// --- Drawing ---------------------------------------------------------------
static void drawShell() {
  tft.fillScreen(TFT_BLACK);
  tft.drawFastHLine(0, 0,   240, L_RED);
  tft.drawFastHLine(0, 24,  240, L_DIM);
  tft.drawFastHLine(0, 296, 240, L_DIM);
  tft.drawFastHLine(0, 319, 240, L_RED);
  tft.setTextFont(2);
  tft.setTextSize(1);
  tft.setTextColor(L_RED, TFT_BLACK);
  tft.setCursor(46, 6);
  tft.print("[ APP LAUNCHER ]");
  tft.setTextColor(L_DIM, TFT_BLACK);
  tft.setCursor(8, 301);
  tft.print("UP/DN  SEL=run  L=save  hold=exit");
}

static void status(const char* s, uint16_t color) {
  tft.fillRect(0, 282, 240, 14, TFT_BLACK);
  tft.setTextFont(2);
  tft.setTextSize(1);
  tft.setTextColor(color, TFT_BLACK);
  tft.setCursor(6, 283);
  tft.print(s);
}

static void drawList() {
  tft.fillRect(0, 30, 240, 252, TFT_BLACK);
  tft.setTextFont(2);
  tft.setTextSize(1);
  tft.setTextColor(L_DIM, TFT_BLACK);
  tft.setCursor(8, 32);
  char hdr[40];
  snprintf(hdr, sizeof(hdr), "/apps  (%d found)", appCount);
  tft.print(hdr);

  if (appCount == 0) {
    tft.setCursor(8, 60);
    tft.setTextColor(L_RED, TFT_BLACK);
    tft.print("No .bin files in /apps");
    tft.setCursor(8, 80);
    tft.setTextColor(L_DIM, TFT_BLACK);
    tft.print("Drop ESP32 .bin firmwares into");
    tft.setCursor(8, 96);
    tft.print("/apps/ on the SD card and");
    tft.setCursor(8, 112);
    tft.print("come back. (Press LEFT to save");
    tft.setCursor(8, 128);
    tft.print("the running Dark-Div firmware");
    tft.setCursor(8, 144);
    tft.print("to /apps/dark-div.bin so you");
    tft.setCursor(8, 160);
    tft.print("can always flash back.)");
    return;
  }

  const int Y0 = 52;
  const int rowH = 18;
  const int maxRows = (282 - Y0) / rowH;
  int start = scroll;
  if (selIndex < start) start = selIndex;
  if (selIndex >= start + maxRows) start = selIndex - maxRows + 1;
  if (start < 0) start = 0;
  scroll = start;

  for (int r = 0; r < maxRows && (start + r) < appCount; r++) {
    int i = start + r;
    int y = Y0 + r * rowH;
    bool isSel = (i == selIndex);
    if (isSel) tft.fillRect(2, y - 2, 236, rowH, L_DIM);
    tft.setTextColor(isSel ? WHITE : L_RED, isSel ? L_DIM : TFT_BLACK);
    tft.setCursor(6, y);
    char line[48];
    const AppEntry& a = apps[i];
    if (a.size < 1024)         snprintf(line, sizeof(line), "%-26s %4u B",  a.name, (unsigned)a.size);
    else if (a.size < 1048576) snprintf(line, sizeof(line), "%-26s %4u KB", a.name, (unsigned)(a.size / 1024));
    else                       snprintf(line, sizeof(line), "%-26s %.2f MB", a.name, a.size / 1048576.0f);
    while (tft.textWidth(line) > 228 && strlen(line) > 4) line[strlen(line) - 1] = 0;
    tft.print(line);
  }
}

// --- Save running firmware to SD -------------------------------------------
static bool saveCurrent() {
  const esp_partition_t* running = esp_ota_get_running_partition();
  if (!running) return false;
  if (!isSDCardAvailable()) return false;
  if (!SD.exists(APPS_DIR)) SD.mkdir(APPS_DIR);

  String path = String(APPS_DIR) + "/dark-div.bin";
  if (SD.exists(path)) SD.remove(path);
  File f = SD.open(path, FILE_WRITE);
  if (!f) return false;

  size_t total = running->size;
  uint8_t buf[1024];
  size_t off = 0;
  uint32_t lastDrawMs = 0;
  while (off < total) {
    size_t want = (total - off) > sizeof(buf) ? sizeof(buf) : (total - off);
    esp_err_t err = esp_partition_read(running, off, buf, want);
    if (err != ESP_OK) { f.close(); return false; }
    if (f.write(buf, want) != want) { f.close(); return false; }
    off += want;
    if (millis() - lastDrawMs > 250) {
      lastDrawMs = millis();
      char msg[40];
      snprintf(msg, sizeof(msg), "saving... %lu / %lu KB",
               (unsigned long)(off / 1024), (unsigned long)(total / 1024));
      status(msg, L_DIM);
    }
  }
  f.close();
  return true;
}

// --- Flash + reboot --------------------------------------------------------
static void flashAndReboot(int idx) {
  if (idx < 0 || idx >= appCount) return;
  const AppEntry& a = apps[idx];

  String path = String(APPS_DIR) + "/" + a.name;
  File f = SD.open(path, FILE_READ);
  if (!f) { status("open failed", L_RED); delay(2000); return; }

  int first = f.peek();
  if (first != 0xE9) {
    f.close();
    char msg[48];
    snprintf(msg, sizeof(msg), "bad magic 0x%02X (not ESP32 .bin)", first & 0xFF);
    status(msg, L_RED);
    delay(2500);
    return;
  }

  drawShell();
  tft.setTextFont(2);
  tft.setTextSize(1);
  tft.setTextColor(L_RED, TFT_BLACK);
  tft.setCursor(8, 60);  tft.print("Flashing:");
  tft.setTextColor(WHITE, TFT_BLACK);
  tft.setCursor(8, 80);  tft.print(a.name);
  tft.setTextColor(L_DIM, TFT_BLACK);
  tft.setCursor(8, 110); tft.print("Do not power off.");

  size_t fileSize = f.size();
  if (!Update.begin(fileSize)) {
    char msg[48];
    snprintf(msg, sizeof(msg), "Update.begin: %s", Update.errorString());
    status(msg, L_RED);
    f.close();
    delay(3000);
    return;
  }

  size_t written = 0;
  size_t lastPct = 0;
  uint8_t buf[1024];
  while (f.available()) {
    int n = f.readBytes((char*)buf, sizeof(buf));
    if (n <= 0) break;
    if (Update.write(buf, n) != (size_t)n) {
      char msg[48];
      snprintf(msg, sizeof(msg), "write fail: %s", Update.errorString());
      status(msg, L_RED);
      Update.abort();
      f.close();
      delay(3000);
      return;
    }
    written += n;
    size_t pct = (written * 100) / fileSize;
    if (pct >= lastPct + 5) {
      lastPct = pct;
      char msg[40];
      snprintf(msg, sizeof(msg), "%u%%  %u / %u KB",
               (unsigned)pct,
               (unsigned)(written / 1024),
               (unsigned)(fileSize / 1024));
      status(msg, L_GRN);
    }
  }
  f.close();

  if (!Update.end(true)) {
    char msg[48];
    snprintf(msg, sizeof(msg), "Update.end: %s", Update.errorString());
    status(msg, L_RED);
    delay(3000);
    return;
  }

  status("OK rebooting...", L_GRN);
  delay(1200);
  ESP.restart();
}

// --- Setup / loop / exit ---------------------------------------------------
void setup() {
  selIndex = 0;
  scroll   = 0;
  uiDirty  = true;
  drawShell();
  status("scanning SD...", L_DIM);
  scanApps();
  drawList();
}

void loop() {
  NeoFx::tick();
  if (uiDirty) {
    drawList();
    uiDirty = false;
  }

  static bool upPrev = false, dnPrev = false, lfPrev = false;
  bool up = !pcf.digitalRead(BTN_UP);
  bool dn = !pcf.digitalRead(BTN_DOWN);
  bool lf = !pcf.digitalRead(BTN_LEFT);
  if (up && !upPrev && appCount > 0 && selIndex > 0)              { selIndex--; uiDirty = true; }
  if (dn && !dnPrev && appCount > 0 && selIndex < appCount - 1)   { selIndex++; uiDirty = true; }
  if (lf && !lfPrev) {
    status("saving running firmware...", L_DIM);
    bool ok = saveCurrent();
    status(ok ? "saved /apps/dark-div.bin" : "save failed",
           ok ? L_GRN : L_RED);
    delay(ok ? 800 : 1500);
    if (ok) { scanApps(); uiDirty = true; }
  }
  upPrev = up; dnPrev = dn; lfPrev = lf;

  // SELECT short tap = flash + reboot; long hold = exit.
  if (isSelectShortTapped() && appCount > 0) {
    flashAndReboot(selIndex);
    drawShell();
    drawList();
  }
  if (isSelectHeldLong()) {
    feature_exit_requested = true;
  }

  delay(15);
}

void exit() {
  // Nothing to tear down.
}

}  // namespace AppLauncher

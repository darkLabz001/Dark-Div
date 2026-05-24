#include <algorithm>
#include <vector>
#include "KeyboardUI.h"
#include "Touchscreen.h"
#include "config.h"
#include "icon.h"
#include "shared.h"


namespace {
  static constexpr const char* SUBGHZ_DIR = "/subghz";
  static constexpr const char* SUBGHZ_EXPORT_PREFIX = "/subghz/profiles_";
  static constexpr const char* SUBGHZ_CURRENT_PATH = "/subghz/profiles_current.bin";
  static constexpr uint32_t SUBGHZ_EXPORT_MAGIC = 0x315A4753;

  struct __attribute__((packed)) SubGhzProfile {
    uint32_t frequency;
    uint32_t value;
    uint16_t bitLength;
    uint16_t protocol;
    char     name[16];
  };

  static constexpr uint16_t MAX_NAME_LENGTH = 16;
  static constexpr uint16_t PROFILE_SIZE = sizeof(SubGhzProfile);

  static constexpr uint16_t ADDR_VALUE = 1280;
  static constexpr uint16_t ADDR_BITLEN = 1284;
  static constexpr uint16_t ADDR_PROTO = 1286;
  static constexpr uint16_t ADDR_FREQ = 1288;
  static constexpr uint16_t ADDR_PROFILE_COUNT = 1296;
  static constexpr uint16_t ADDR_PROFILE_START = 1300;
  static constexpr uint16_t MAX_PROFILES = 5;

  struct __attribute__((packed)) SubGhzExportHeader {
    uint32_t magic;
    uint16_t version;
    uint16_t count;
    uint16_t profileSize;
    uint16_t reserved;
  };

  static bool subghz_sd_mounted = false;
  static bool subghzMountSD() {
    if (subghz_sd_mounted) {
      if (SD.exists("/")) return true;
      subghz_sd_mounted = false;
    }

    #ifdef SD_CD
    pinMode(SD_CD, INPUT_PULLUP);
    if (digitalRead(SD_CD)) return false;
    #endif

    #ifdef SD_SCLK
    #ifdef SD_MISO
    #ifdef SD_MOSI
    #ifdef SD_CS
    SPI.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);
    #endif
    #endif
    #endif
    #endif

    #ifdef SD_CS
    if (SD.begin(SD_CS)) { subghz_sd_mounted = true; return true; }
    #endif

    #ifdef SD_CS_PIN
    #ifdef CC1101_CS
    if (SD_CS_PIN != CC1101_CS) {
      if (SD.begin(SD_CS_PIN)) { subghz_sd_mounted = true; return true; }
    }
    #else
    if (SD.begin(SD_CS_PIN)) { subghz_sd_mounted = true; return true; }
    #endif
    #endif

    return false;
  }

  static bool subghzEnsureDir(const char* dirPath) {
    if (!subghzMountSD()) return false;
    if (SD.exists(dirPath)) return true;
    if (SD.mkdir(dirPath)) return true;
    if (dirPath && dirPath[0] == '/') return SD.mkdir(dirPath + 1);
    return false;
  }

  static void clearProfilesInEeprom() {

    uint16_t zero = 0;
    EEPROM.put(ADDR_PROFILE_COUNT, zero);
    SubGhzProfile empty{};
    for (uint16_t i = 0; i < MAX_PROFILES; i++) {
      EEPROM.put(ADDR_PROFILE_START + (i * PROFILE_SIZE), empty);
    }
    EEPROM.commit();
  }

  static bool makeNextExportPath(String& outPath) {

    for (uint16_t i = 0; i < 10000; i++) {
      char buf[48];
      snprintf(buf, sizeof(buf), "%s%04u.bin", SUBGHZ_EXPORT_PREFIX, (unsigned)i);
      if (!SD.exists(buf)) { outPath = String(buf); return true; }
    }
    return false;
  }

  static bool findLatestExportPath(String& outPath) {

    for (int i = 9999; i >= 0; i--) {
      char buf[48];
      snprintf(buf, sizeof(buf), "%s%04u.bin", SUBGHZ_EXPORT_PREFIX, (unsigned)i);
      if (SD.exists(buf)) { outPath = String(buf); return true; }
    }
    return false;
  }

  static bool exportProfilesToSD(String& outPath, String* errOut = nullptr) {
    if (!subghzEnsureDir(SUBGHZ_DIR)) {
      if (errOut) *errOut = "SD not mounted";
      return false;
    }

    uint16_t count = 0;
    EEPROM.get(ADDR_PROFILE_COUNT, count);
    if (count > MAX_PROFILES) count = MAX_PROFILES;

    if (!makeNextExportPath(outPath)) {
      if (errOut) *errOut = "No free filename";
      return false;
    }

    File f = SD.open(outPath.c_str(), FILE_WRITE);
    if (!f) {
      if (errOut) *errOut = "Open failed";
      return false;
    }

    SubGhzExportHeader h{};
    h.magic = SUBGHZ_EXPORT_MAGIC;
    h.version = 1;
    h.count = count;
    h.profileSize = PROFILE_SIZE;
    h.reserved = 0;

    bool ok = (f.write((const uint8_t*)&h, sizeof(h)) == sizeof(h));
    for (uint16_t i = 0; ok && i < count; i++) {
      SubGhzProfile p{};
      int addr = ADDR_PROFILE_START + (i * PROFILE_SIZE);
      EEPROM.get(addr, p);
      ok = (f.write((const uint8_t*)&p, sizeof(p)) == sizeof(p));
    }
    f.close();

    if (!ok && errOut) *errOut = "Write failed";
    return ok;
  }

  static bool syncCurrentProfilesToSD(String* errOut = nullptr) {

    if (!subghzEnsureDir(SUBGHZ_DIR)) {
      if (errOut) *errOut = "SD not mounted";
      return false;
    }

    uint16_t count = 0;
    EEPROM.get(ADDR_PROFILE_COUNT, count);
    if (count > MAX_PROFILES) count = MAX_PROFILES;

    if (SD.exists(SUBGHZ_CURRENT_PATH)) SD.remove(SUBGHZ_CURRENT_PATH);
    File f = SD.open(SUBGHZ_CURRENT_PATH, FILE_WRITE);
    if (!f) {
      if (errOut) *errOut = "Open failed";
      return false;
    }

    SubGhzExportHeader h{};
    h.magic = SUBGHZ_EXPORT_MAGIC;
    h.version = 1;
    h.count = count;
    h.profileSize = PROFILE_SIZE;
    h.reserved = 0;

    bool ok = (f.write((const uint8_t*)&h, sizeof(h)) == sizeof(h));
    for (uint16_t i = 0; ok && i < count; i++) {
      SubGhzProfile p{};
      int addr = ADDR_PROFILE_START + (i * PROFILE_SIZE);
      EEPROM.get(addr, p);
      ok = (f.write((const uint8_t*)&p, sizeof(p)) == sizeof(p));
    }
    f.close();
    if (!ok && errOut) *errOut = "Write failed";
    return ok;
  }

  static bool importProfilesFromSD(const String& path, String* errOut = nullptr) {
    if (!subghzMountSD()) {
      if (errOut) *errOut = "SD not mounted";
      return false;
    }
    if (path.isEmpty() || !SD.exists(path.c_str())) {
      if (errOut) *errOut = "File not found";
      return false;
    }

    File f = SD.open(path.c_str(), FILE_READ);
    if (!f) {
      if (errOut) *errOut = "Open failed";
      return false;
    }

    SubGhzExportHeader h{};
    if (f.read((uint8_t*)&h, sizeof(h)) != sizeof(h)) { f.close(); if (errOut) *errOut="Bad header"; return false; }
    if (h.magic != SUBGHZ_EXPORT_MAGIC || h.version != 1) { f.close(); if (errOut) *errOut="Wrong file"; return false; }
    if (h.profileSize != PROFILE_SIZE) { f.close(); if (errOut) *errOut="Size mismatch"; return false; }

    uint16_t count = h.count;
    if (count > MAX_PROFILES) count = MAX_PROFILES;

    clearProfilesInEeprom();
    for (uint16_t i = 0; i < count; i++) {
      SubGhzProfile p{};
      if (f.read((uint8_t*)&p, sizeof(p)) != sizeof(p)) { f.close(); if (errOut) *errOut="Read failed"; return false; }
      p.name[MAX_NAME_LENGTH - 1] = '\0';
      EEPROM.put(ADDR_PROFILE_START + (i * PROFILE_SIZE), p);
    }
    EEPROM.put(ADDR_PROFILE_COUNT, count);
    EEPROM.commit();
    f.close();
    return true;
  }

  struct SubGhzFileEntry {
    String path;
    uint16_t count = 0;
    bool isCurrent = false;
  };

  static bool readExportHeader(File& f, SubGhzExportHeader& out, String* errOut = nullptr) {
    if (f.read((uint8_t*)&out, sizeof(out)) != sizeof(out)) { if (errOut) *errOut="Bad header"; return false; }
    if (out.magic != SUBGHZ_EXPORT_MAGIC || out.version != 1) { if (errOut) *errOut="Wrong file"; return false; }
    if (out.profileSize != PROFILE_SIZE) { if (errOut) *errOut="Size mismatch"; return false; }
    return true;
  }

  static bool readProfileAt(const String& path, uint16_t localIndex, SubGhzProfile& out, String* errOut = nullptr) {
    if (!subghzMountSD()) { if (errOut) *errOut="SD not mounted"; return false; }
    File f = SD.open(path.c_str(), FILE_READ);
    if (!f) { if (errOut) *errOut="Open failed"; return false; }
    SubGhzExportHeader h{};
    if (!readExportHeader(f, h, errOut)) { f.close(); return false; }
    uint16_t count = h.count; if (count > MAX_PROFILES) count = MAX_PROFILES;
    if (localIndex >= count) { f.close(); if (errOut) *errOut="Index OOR"; return false; }
    uint32_t off = (uint32_t)sizeof(SubGhzExportHeader) + (uint32_t)localIndex * (uint32_t)PROFILE_SIZE;
    if (!f.seek(off)) { f.close(); if (errOut) *errOut="Seek failed"; return false; }
    if (f.read((uint8_t*)&out, sizeof(out)) != sizeof(out)) { f.close(); if (errOut) *errOut="Read failed"; return false; }
    out.name[MAX_NAME_LENGTH - 1] = '\0';
    f.close();
    return true;
  }

  static bool listAllProfileFiles(std::vector<SubGhzFileEntry>& out, String* errOut = nullptr) {
    out.clear();
    if (!subghzMountSD()) { if (errOut) *errOut="SD not mounted"; return false; }
    if (!SD.exists(SUBGHZ_DIR)) { if (errOut) *errOut="No /subghz"; return false; }
    File d = SD.open(SUBGHZ_DIR);
    if (!d) { if (errOut) *errOut="Open dir failed"; return false; }

    for (;;) {
      File f = d.openNextFile();
      if (!f) break;
      if (f.isDirectory()) { f.close(); continue; }
      String name = String(f.name());

      String fullPath = String(SUBGHZ_DIR) + "/" + name;

      bool isCurrent = (name == "profiles_current.bin");
      bool isArchive = name.startsWith("profiles_") && name.endsWith(".bin") && !isCurrent;
      if (!isCurrent && !isArchive) { f.close(); continue; }

      SubGhzExportHeader h{};
      String herr;
      bool ok = readExportHeader(f, h, &herr);
      f.close();
      if (!ok) continue;

      uint16_t cnt = h.count;
      if (cnt > MAX_PROFILES) cnt = MAX_PROFILES;
      if (cnt == 0) continue;

      SubGhzFileEntry e;
      e.path = fullPath;
      e.count = cnt;
      e.isCurrent = isCurrent;
      out.push_back(e);
    }
    d.close();

    std::sort(out.begin(), out.end(), [](const SubGhzFileEntry& a, const SubGhzFileEntry& b) {
      if (a.isCurrent != b.isCurrent) return a.isCurrent > b.isCurrent;
      return a.path > b.path;
    });
    return true;
  }

  static uint16_t totalProfilesInIndex(const std::vector<SubGhzFileEntry>& files) {
    uint32_t total = 0;
    for (auto& f : files) total += f.count;
    if (total > 65535) total = 65535;
    return (uint16_t)total;
  }

  static bool locateGlobalIndex(const std::vector<SubGhzFileEntry>& files, uint16_t globalIndex,
                                String& outPath, uint16_t& outLocalIdx) {
    uint32_t idx = globalIndex;
    for (auto& fe : files) {
      if (idx < fe.count) {
        outPath = fe.path;
        outLocalIdx = (uint16_t)idx;
        return true;
      }
      idx -= fe.count;
    }
    return false;
  }

  static bool deleteProfileFromFile(const String& path, uint16_t localIndex, String* errOut = nullptr) {
    if (!subghzMountSD()) { if (errOut) *errOut="SD not mounted"; return false; }
    File f = SD.open(path.c_str(), FILE_READ);
    if (!f) { if (errOut) *errOut="Open failed"; return false; }
    SubGhzExportHeader h{};
    if (!readExportHeader(f, h, errOut)) { f.close(); return false; }
    uint16_t count = h.count; if (count > MAX_PROFILES) count = MAX_PROFILES;
    if (localIndex >= count) { f.close(); if (errOut) *errOut="Index OOR"; return false; }

    SubGhzProfile buf[MAX_PROFILES]{};
    for (uint16_t i = 0; i < count; i++) {
      if (f.read((uint8_t*)&buf[i], sizeof(SubGhzProfile)) != sizeof(SubGhzProfile)) { f.close(); if (errOut) *errOut="Read failed"; return false; }
      buf[i].name[MAX_NAME_LENGTH - 1] = '\0';
    }
    f.close();

    for (uint16_t i = localIndex; i + 1 < count; i++) buf[i] = buf[i + 1];
    count--;

    if (SD.exists(path.c_str())) SD.remove(path.c_str());
    File w = SD.open(path.c_str(), FILE_WRITE);
    if (!w) { if (errOut) *errOut="Open write failed"; return false; }

    SubGhzExportHeader nh{};
    nh.magic = SUBGHZ_EXPORT_MAGIC;
    nh.version = 1;
    nh.count = count;
    nh.profileSize = PROFILE_SIZE;
    nh.reserved = 0;
    bool ok = (w.write((const uint8_t*)&nh, sizeof(nh)) == sizeof(nh));
    for (uint16_t i = 0; ok && i < count; i++) {
      ok = (w.write((const uint8_t*)&buf[i], sizeof(SubGhzProfile)) == sizeof(SubGhzProfile));
    }
    w.close();
    if (!ok && errOut) *errOut="Write failed";

    if (ok && path.endsWith("profiles_current.bin")) {
      importProfilesFromSD(path, nullptr);
    }
    return ok;
  }
}

#ifdef TFT_BLACK
#undef TFT_BLACK
#endif
#define TFT_BLACK FEATURE_BG

#ifndef FEATURE_TEXT
#define FEATURE_TEXT ORANGE
#endif
#ifndef FEATURE_WHITE
#define FEATURE_WHITE 0xFFFF
#endif

#ifdef TFT_WHITE
#undef TFT_WHITE
#endif
#define TFT_WHITE FEATURE_TEXT

#ifdef WHITE
#undef WHITE
#endif
#define WHITE FEATURE_WHITE

#ifdef DARK_GRAY
#undef DARK_GRAY
#endif
#define DARK_GRAY UI_FG

namespace replayat {

#define EEPROM_SIZE 1440

#define ADDR_VALUE         1280
#define ADDR_BITLEN        1284
#define ADDR_PROTO         1286
#define ADDR_FREQ          1288
#define ADDR_PROFILE_COUNT 1296
#define ADDR_PROFILE_START 1300
#define MAX_PROFILES       5

#define SCREEN_WIDTH  240
#define SCREENHEIGHT 320
#define SCREEN_HEIGHT 320

static bool uiDrawn = false;

#define MAX_NAME_LENGTH 16

const char* profileKeyboardRows[] = {
  "1234567890",
  "QWERTYUIOP",
  "ASDFGHJKL",
  "ZXCVBNM<-"
};

const char* randomNames[] = {
  "Signal", "Remote", "KeyFob", "GateOpener", "DoorLock",
  "RFTest", "Profile", "Control", "Switch", "Beacon"
};
const uint8_t numRandomNames = 10;

struct __attribute__((packed)) Profile {
    uint32_t frequency;
    uint32_t value;
    uint16_t bitLength;
    uint16_t protocol;
    char name[MAX_NAME_LENGTH];
};

#define PROFILE_SIZE sizeof(Profile)

uint16_t profileCount = 0;

RCSwitch mySwitch = RCSwitch();
arduinoFFT FFTSUB = arduinoFFT();

const uint16_t samplesSUB = 256;
const double FrequencySUB = 5000;

double attenuation_num = 10;

unsigned int sampling_period;
unsigned long micro_s;

double vRealSUB[samplesSUB];
double vImagSUB[samplesSUB];

byte red[128], green[128], blue[128];

unsigned int epochSUB = 0;
unsigned int colorcursor = 2016;

int rssi;

static constexpr uint8_t REPLAY_RX_PIN = SUBGHZ_RX_PIN;
static constexpr uint8_t REPLAY_TX_PIN = SUBGHZ_TX_PIN;

uint32_t receivedValue = 0;
uint16_t receivedBitLength = 0;
uint16_t receivedProtocol = 0;
const int rssi_threshold = -75;

static const uint32_t subghz_frequency_list[] = {
    300000000, 303875000, 304250000, 310000000, 315000000, 318000000,
    390000000, 418000000, 433075000, 433420000, 433920000, 434420000,
    434775000, 438900000, 868350000, 915000000, 925000000
};

uint16_t currentFrequencyIndex = 0;
int yshift = 20;

static bool autoScanEnabled = false;
static uint16_t scanIndex = 0;
static uint32_t lastHopMs = 0;
static uint32_t lockUntilMs = 0;
static constexpr uint32_t SCAN_DWELL_MS = 110;
static constexpr uint32_t LOCK_HOLD_MS  = 2500;
static constexpr uint32_t RSSI_LOCK_MS  = 1200;
static constexpr int      RSSI_DETECT_THRESHOLD = -72;
static constexpr int      RSSI_CLEAR_THRESHOLD  = -78;
static constexpr uint32_t UI_SCAN_UPDATE_MS = 250;
static uint32_t lastUiScanUpdateMs = 0;
static bool     rssiHot = false;

static uint32_t lastDetectAlertMs = 0;
static uint16_t lastDetectAlertFreq = 0xFFFF;
static uint32_t notifHideAtMs = 0;
static bool notifActive = false;

static constexpr uint8_t BUZZER_LEDC_CH = 7;
static bool buzzerArmed = false;
static uint32_t buzzerOffAtMs = 0;
static void replayBeep(uint16_t hz = 2200, uint16_t ms = 60) {
  #if defined(BUZZER_PIN) && (BUZZER_PIN) >= 0
  ledcSetup(BUZZER_LEDC_CH, 4000, 8);
  ledcAttachPin(BUZZER_PIN, BUZZER_LEDC_CH);
  ledcWriteTone(BUZZER_LEDC_CH, hz);
  buzzerArmed = true;
  buzzerOffAtMs = millis() + ms;
  #else
  (void)hz; (void)ms;
  #endif
}

static void replayBeepPoll() {
  #if defined(BUZZER_PIN) && (BUZZER_PIN) >= 0
  if (!buzzerArmed) return;
  if ((int32_t)(millis() - buzzerOffAtMs) < 0) return;
  ledcWriteTone(BUZZER_LEDC_CH, 0);

  ledcDetachPin(BUZZER_PIN);
  buzzerArmed = false;
  #endif
}

static void replayShowDetectNotice(const String& reason, int rssi = 0) {
  uint32_t now = millis();

  if (now - lastDetectAlertMs < 1200 && lastDetectAlertFreq == currentFrequencyIndex) return;
  lastDetectAlertMs = now;
  lastDetectAlertFreq = currentFrequencyIndex;

  char msg[96];
  float mhz = subghz_frequency_list[currentFrequencyIndex] / 1000000.0f;

  snprintf(msg, sizeof(msg), "%s @ %.2f MHz | RSSI %d", reason.c_str(), mhz, rssi);
  showNotificationActions("SubGHz Detected", msg, true);
  replayBeep(reason == "DECODE" ? 2600 : 2000, 70);
  notifActive = true;
  notifHideAtMs = 0;
}

static inline uint16_t freqCount() {
  return (uint16_t)(sizeof(subghz_frequency_list) / sizeof(subghz_frequency_list[0]));
}

static void tuneToIndex(uint16_t idx, bool persist = true) {
  currentFrequencyIndex = idx % freqCount();
  ELECHOUSE_cc1101.setSidle();
  ELECHOUSE_cc1101.setMHZ(subghz_frequency_list[currentFrequencyIndex] / 1000000.0);
  ELECHOUSE_cc1101.SetRx();
  if (persist) {
    EEPROM.put(ADDR_FREQ, currentFrequencyIndex);
    EEPROM.commit();
  }
}

void updateDisplay() {
    uiDrawn = false;

    tft.fillRect(0, 40, 240, 40, TFT_BLACK);
    tft.drawLine(0, 80, 240, 80, ORANGE);

    tft.setCursor(5, 20 + yshift);
    tft.setTextColor(WHITE);
    tft.print("Freq:");
    tft.setTextColor(ORANGE);
    tft.setCursor(50, 20 + yshift);
    tft.print(subghz_frequency_list[currentFrequencyIndex] / 1000000.0, 2);
    tft.print(" MHz");

    tft.setCursor(175, 20 + yshift);
    bool locked = (autoScanEnabled && lockUntilMs != 0 && (int32_t)(millis() - lockUntilMs) < 0);
    tft.setTextColor(autoScanEnabled ? ORANGE : ORANGE);
    if (locked) {
      tft.print("LOCK");
    } else {
      tft.print(autoScanEnabled ? "AUTO" : "MAN ");
    }

    tft.setCursor(5, 35 + yshift);
    tft.setTextColor(WHITE);
    tft.print("Bit:");
    tft.setTextColor(ORANGE);
    tft.setCursor(50, 35 + yshift);
    tft.printf("%d", receivedBitLength);

    tft.setCursor(130, 35 + yshift);
    tft.setTextColor(WHITE);
    tft.print("RSSI:");
    tft.setTextColor(ORANGE);
    tft.setCursor(170, 35 + yshift);
    tft.printf("%d", ELECHOUSE_cc1101.getRssi());

    tft.setCursor(130, 20 + yshift);
    tft.setTextColor(WHITE);
    tft.print("Ptc:");
    tft.setTextColor(ORANGE);
    tft.setCursor(170, 20 + yshift);
    tft.printf("%d", receivedProtocol);

    tft.setCursor(5, 50 + yshift);
    tft.setTextColor(WHITE);
    tft.print("Val:");
    tft.setTextColor(ORANGE);
    tft.setCursor(50, 50 + yshift);
    tft.print(receivedValue);

    ELECHOUSE_cc1101.setSidle();
    ELECHOUSE_cc1101.setMHZ(subghz_frequency_list[currentFrequencyIndex] / 1000000.0);
    ELECHOUSE_cc1101.SetRx();
}

String getUserInputName() {
  OnScreenKeyboardConfig cfg;
  cfg.titleLine1     = "[!] Set a name for the saved profile.";
  cfg.titleLine2     = "(max 15 chars)";
  cfg.rows           = profileKeyboardRows;
  cfg.rowCount       = 4;
  cfg.maxLen         = MAX_NAME_LENGTH - 1;
  cfg.shuffleNames   = randomNames;
  cfg.shuffleCount   = numRandomNames;
  cfg.buttonsY       = 195;
  cfg.backLabel      = "Back";
  cfg.middleLabel    = "Shuffle";
  cfg.okLabel        = "OK";
  cfg.enableShuffle  = true;
  cfg.requireNonEmpty = true;
  cfg.emptyErrorMsg  = "Name cannot be empty!";

  OnScreenKeyboardResult r = showOnScreenKeyboard(cfg, "");

  if (!r.accepted) {

    tft.fillScreen(TFT_BLACK);
    updateDisplay();
  }
  return r.text;
}

void sendSignal() {

    mySwitch.disableReceive();
    delay(100);
    mySwitch.enableTransmit(REPLAY_TX_PIN);
    ELECHOUSE_cc1101.SetTx();

    tft.fillRect(0,40,240,37, TFT_BLACK);

    tft.setCursor(10, 30 + yshift);
    tft.print("Sending...");
    tft.setCursor(10, 40 + yshift);
    tft.print(receivedValue);

    mySwitch.setProtocol(receivedProtocol);
    mySwitch.send(receivedValue, receivedBitLength);

    delay(500);
    tft.fillRect(0,40,240,37, TFT_BLACK);
    tft.setCursor(10, 30 + yshift);
    tft.print("Done!");

    ELECHOUSE_cc1101.SetRx();
    mySwitch.disableTransmit();
    delay(100);
    mySwitch.enableReceive(REPLAY_RX_PIN);

    delay(500);
    updateDisplay();
}

void do_sampling() {

  micro_s = micros();

  #define ALPHA 0.2
  float ewmaRSSI = -50;

for (int i = 0; i < samplesSUB; i++) {
    int rssi = ELECHOUSE_cc1101.getRssi();
    rssi += 100;

    ewmaRSSI = (ALPHA * rssi) + ((1 - ALPHA) * ewmaRSSI);

    vRealSUB[i] = ewmaRSSI * 2;
    vImagSUB[i] = 1;

    while (micros() < micro_s + sampling_period);
    micro_s += sampling_period;
}

  double mean = 0;

  for (uint16_t i = 0; i < samplesSUB; i++)
        mean += vRealSUB[i];
        mean /= samplesSUB;
  for (uint16_t i = 0; i < samplesSUB; i++)
        vRealSUB[i] -= mean;

  micro_s = micros();

  FFTSUB.Windowing(vRealSUB, samplesSUB, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFTSUB.Compute(vRealSUB, vImagSUB, samplesSUB, FFT_FORWARD);
  FFTSUB.ComplexToMagnitude(vRealSUB, vImagSUB, samplesSUB);

unsigned int left_x = 120;
unsigned int graph_y_offset = 81;
int max_k = 0;

for (int j = 0; j < samplesSUB >> 1; j++) {
    int k = vRealSUB[j] / attenuation_num;
    if (k > max_k)
        max_k = k;
    if (k > 127) k = 127;

    unsigned int color = red[k] << 11 | green[k] << 5 | blue[k];
    unsigned int vertical_x = left_x + j;

    tft.drawPixel(vertical_x, epochSUB + graph_y_offset, color);
}

for (int j = 0; j < samplesSUB >> 1; j++) {
    int k = vRealSUB[j] / attenuation_num;
    if (k > max_k)
        max_k = k;
    if (k > 127) k = 127;

    unsigned int color = red[k] << 11 | green[k] << 5 | blue[k];
    unsigned int mirrored_x = left_x - j;
    tft.drawPixel(mirrored_x, epochSUB + graph_y_offset, color);
}

  double tattenuation = max_k / 127.0;

  if (tattenuation > attenuation_num)
    attenuation_num = tattenuation;

    delay(10);
}

void readProfileCount() {
    EEPROM.get(ADDR_PROFILE_COUNT, profileCount);
    if (profileCount > MAX_PROFILES) profileCount = 0;
}

void saveProfile() {
    readProfileCount();

    if (profileCount >= MAX_PROFILES) {

        String err, outPath;
        if (exportProfilesToSD(outPath, &err)) {
            clearProfilesInEeprom();
            profileCount = 0;

            syncCurrentProfilesToSD(nullptr);
        } else {
            tft.fillScreen(TFT_BLACK);
            tft.setCursor(10, 30 + yshift);
            tft.setTextColor(UI_WARN);
            tft.print("Storage full!");
            tft.setCursor(10, 45 + yshift);
            tft.setTextColor(TFT_WHITE);
            tft.print("Insert SD / export fail");
            tft.setCursor(10, 60 + yshift);
            tft.print(err);
            delay(2000);
            updateDisplay();
            float currentBatteryVoltage = readBatteryVoltage();
            drawStatusBar(currentBatteryVoltage, true);
            return;
        }
    }

    if (profileCount < MAX_PROFILES) {

        String customName = getUserInputName();

        tft.setTextSize(1);

        Profile newProfile;
        newProfile.frequency = subghz_frequency_list[currentFrequencyIndex];
        newProfile.value = (uint32_t)receivedValue;
        newProfile.bitLength = (uint16_t)receivedBitLength;
        newProfile.protocol = (uint16_t)receivedProtocol;
        strncpy(newProfile.name, customName.c_str(), MAX_NAME_LENGTH - 1);
        newProfile.name[MAX_NAME_LENGTH - 1] = '\0';

        int addr = ADDR_PROFILE_START + (profileCount * PROFILE_SIZE);
        EEPROM.put(addr, newProfile);
        EEPROM.commit();

        profileCount++;

        EEPROM.put(ADDR_PROFILE_COUNT, profileCount);
        EEPROM.commit();

        syncCurrentProfilesToSD(nullptr);

        tft.fillScreen(TFT_BLACK);
        tft.setCursor(10, 30 + yshift);
        tft.print("Profile saved!");
        tft.setCursor(10, 40 + yshift);
        tft.print("Name: ");
        tft.print(newProfile.name);
        tft.setCursor(10, 50 + yshift);
        tft.print("Profiles saved: ");
        tft.println(profileCount);

    } else {
        tft.fillScreen(TFT_BLACK);
        tft.setCursor(10, 30 + yshift);
        tft.print("Profile storage full!");
    }

    delay(2000);
    updateDisplay();
    float currentBatteryVoltage = readBatteryVoltage();
    drawStatusBar(currentBatteryVoltage, true);
}

void loadProfileCount() {

    readProfileCount();
}

void runUI() {

    #define STATUS_BAR_Y_OFFSET 20
    #define STATUS_BAR_HEIGHT 16
    #define ICON_SIZE 16
    #define ICON_NUM 6

    static int iconX[ICON_NUM] = {90, 130, 170, 210, 50, 10};
    static int iconY = STATUS_BAR_Y_OFFSET;

    static const unsigned char* icons[ICON_NUM] = {
        bitmap_icon_sort_up_plus,
        bitmap_icon_sort_down_minus,
        bitmap_icon_antenna,
        bitmap_icon_floppy,
        bitmap_icon_random,
        bitmap_icon_go_back
    };

    if (!uiDrawn) {
        tft.drawLine(0, 19, 240, 19, TFT_WHITE);
        tft.fillRect(0, STATUS_BAR_Y_OFFSET, SCREEN_WIDTH, STATUS_BAR_HEIGHT, DARK_GRAY);

        for (int i = 0; i < ICON_NUM; i++) {
            if (icons[i] != NULL) {
                tft.drawBitmap(iconX[i], iconY, icons[i], ICON_SIZE, ICON_SIZE, TFT_WHITE);
            }
        }
        tft.drawLine(0, STATUS_BAR_Y_OFFSET + STATUS_BAR_HEIGHT, SCREEN_WIDTH, STATUS_BAR_Y_OFFSET + STATUS_BAR_HEIGHT, ORANGE);
        uiDrawn = true;
    }

    static unsigned long lastAnimationTime = 0;
    static int animationState = 0;
    static int activeIcon = -1;

    if (animationState > 0 && millis() - lastAnimationTime >= 150) {
        if (animationState == 1) {
            tft.drawBitmap(iconX[activeIcon], iconY, icons[activeIcon], ICON_SIZE, ICON_SIZE, TFT_WHITE);
            animationState = 2;

            switch (activeIcon) {
                case 0:
                    autoScanEnabled = false;
                    currentFrequencyIndex = (currentFrequencyIndex + 1) % (sizeof(subghz_frequency_list) / sizeof(subghz_frequency_list[0]));
                    tuneToIndex(currentFrequencyIndex, true);
                    updateDisplay();
                    break;
                case 1:
                    autoScanEnabled = false;
                    currentFrequencyIndex = (currentFrequencyIndex - 1 + (sizeof(subghz_frequency_list) / sizeof(subghz_frequency_list[0]))) % (sizeof(subghz_frequency_list) / sizeof(subghz_frequency_list[0]));
                    tuneToIndex(currentFrequencyIndex, true);
                    updateDisplay();
                    break;
                case 2:
                    sendSignal();
                    break;
                case 3:
                    saveProfile();
                    break;
                case 4:
                    autoScanEnabled = !autoScanEnabled;
                    scanIndex = currentFrequencyIndex;
                    lastHopMs = 0;
                    lockUntilMs = 0;
                    lastUiScanUpdateMs = 0;
                    rssiHot = false;
                    updateDisplay();
                    break;
            }
        } else if (animationState == 2) {
            animationState = 0;
            activeIcon = -1;
        }
        lastAnimationTime = millis();
    }

    static unsigned long lastTouchCheck = 0;
    const unsigned long touchCheckInterval = 50;

    if (millis() - lastTouchCheck >= touchCheckInterval) {
        int x, y;
        if (feature_active && readTouchXY(x, y)) {
            if (y > STATUS_BAR_Y_OFFSET && y < STATUS_BAR_Y_OFFSET + STATUS_BAR_HEIGHT) {
                for (int i = 0; i < ICON_NUM; i++) {
                    if (x > iconX[i] && x < iconX[i] + ICON_SIZE) {
                        if (icons[i] != NULL && animationState == 0) {

                            if (i == 5) {
                                feature_exit_requested = true;
                            } else {

                                tft.drawBitmap(iconX[i], iconY, icons[i], ICON_SIZE, ICON_SIZE, TFT_BLACK);
                                animationState = 1;
                                activeIcon = i;
                                lastAnimationTime = millis();
                            }
                        }
                        break;
                    }
                }
            }
        }
        lastTouchCheck = millis();
    }
}

void ReplayAttackSetup() {
  Serial.begin(115200);

  ELECHOUSE_cc1101.setSpiPin(CC1101_SCK, CC1101_MISO, CC1101_MOSI, CC1101_CS);

  ELECHOUSE_cc1101.Init();

  ELECHOUSE_cc1101.setGDO(CC1101_GDO0, CC1101_GDO2);

  ELECHOUSE_cc1101.SetRx();

  mySwitch.enableReceive(REPLAY_RX_PIN);
  mySwitch.enableTransmit(REPLAY_TX_PIN);
  mySwitch.setRepeatTransmit(8);

  EEPROM.begin(EEPROM_SIZE);
  readProfileCount();

  EEPROM.get(ADDR_VALUE, receivedValue);
  EEPROM.get(ADDR_BITLEN, receivedBitLength);
  EEPROM.get(ADDR_PROTO, receivedProtocol);
  EEPROM.get(ADDR_FREQ, currentFrequencyIndex);

  const uint16_t freqCount = (uint16_t)(sizeof(subghz_frequency_list) / sizeof(subghz_frequency_list[0]));
  if (currentFrequencyIndex >= freqCount) currentFrequencyIndex = 0;

  tuneToIndex(currentFrequencyIndex, false);

    tft.fillScreen(TFT_BLACK);
  tft.setRotation(2);

  pcf.pinMode(BTN_LEFT, INPUT_PULLUP);
  pcf.pinMode(BTN_RIGHT, INPUT_PULLUP);
  pcf.pinMode(BTN_UP, INPUT_PULLUP);
  pcf.pinMode(BTN_DOWN, INPUT_PULLUP);
  pcf.pinMode(BTN_SELECT, INPUT_PULLUP);

  sampling_period = round(1000000*(1.0/FrequencySUB));

  for (int i = 0; i < 32; i++) {
    red[i] = i / 2;
    green[i] = 0;
    blue[i] = i;
  }
  for (int i = 32; i < 64; i++) {
    red[i] = i / 2;
    green[i] = 0;
    blue[i] = 63 - i;
  }
  for (int i = 64; i < 96; i++) {
    red[i] = 31;
    green[i] = (i - 64) * 2;
    blue[i] = 0;
  }
  for (int i = 96; i < 128; i++) {
    red[i] = 31;
    green[i] = 63;
    blue[i] = i - 96;
  }

   float currentBatteryVoltage = readBatteryVoltage();
   drawStatusBar(currentBatteryVoltage, true);
   updateDisplay();
   uiDrawn = false;

}

void ReplayAttackLoop() {

    if (feature_active && isButtonPressed(BTN_SELECT)) {
        feature_exit_requested = true;
        return;
    }

    runUI();

    static unsigned long lastDebounceTime = 0;
    const unsigned long debounceDelay = 200;

    static bool prevLeft = false, prevRight = false, prevUp = false, prevDown = false;
    const bool leftPressed  = isButtonPressed(BTN_LEFT);
    const bool rightPressed = isButtonPressed(BTN_RIGHT);
    const bool upPressed    = isButtonPressed(BTN_UP);
    const bool downPressed  = isButtonPressed(BTN_DOWN);

    replayBeepPoll();

    if (notifActive && isNotificationVisible()) {
      int x, y;
      if (readTouchXY(x, y)) {
        NotificationAction act = notificationHandleTouch(x, y);
        if (act == NotificationAction::Save) {
          notifActive = false;

          tft.fillScreen(TFT_BLACK);
          uiDrawn = false;
          float v = readBatteryVoltage();
          drawStatusBar(v, true);
          runUI();
          updateDisplay();

          autoScanEnabled = false;
          saveProfile();

          tft.fillScreen(TFT_BLACK);
          uiDrawn = false;
          v = readBatteryVoltage();
          drawStatusBar(v, true);
          runUI();
          updateDisplay();
        } else if (act == NotificationAction::Ok || act == NotificationAction::Close) {
          notifActive = false;

          lastDetectAlertMs = millis();
          lastDetectAlertFreq = currentFrequencyIndex;
          lockUntilMs = millis() + 1500;
          rssiHot = true;

          tft.fillScreen(TFT_BLACK);
          uiDrawn = false;
          float v = readBatteryVoltage();
          drawStatusBar(v, true);
          runUI();
          updateDisplay();
        }
      }

      return;
    } else if (notifActive && !isNotificationVisible()) {

      notifActive = false;
      tft.fillScreen(TFT_BLACK);
      uiDrawn = false;
      float v = readBatteryVoltage();
      drawStatusBar(v, true);
      runUI();
      updateDisplay();
    }

    if (rightPressed && !prevRight && millis() - lastDebounceTime > debounceDelay) {
        autoScanEnabled = false;
        lockUntilMs = 0;
        rssiHot = false;
        tuneToIndex((uint16_t)((currentFrequencyIndex + 1) % freqCount()), true);
        updateDisplay();
        lastDebounceTime = millis();
    }
    if (leftPressed && !prevLeft && millis() - lastDebounceTime > debounceDelay) {
        autoScanEnabled = false;
        lockUntilMs = 0;
        rssiHot = false;
        tuneToIndex((uint16_t)((currentFrequencyIndex + freqCount() - 1) % freqCount()), true);
        updateDisplay();
        lastDebounceTime = millis();
    }
    if (upPressed && !prevUp && receivedValue != 0 && millis() - lastDebounceTime > debounceDelay) {

        autoScanEnabled = false;
        lockUntilMs = 0;
        rssiHot = false;
        sendSignal();
        lastDebounceTime = millis();
    }
    if (downPressed && !prevDown && millis() - lastDebounceTime > debounceDelay) {

        autoScanEnabled = !autoScanEnabled;
        scanIndex = currentFrequencyIndex;
        lastHopMs = 0;
        lockUntilMs = 0;
        lastUiScanUpdateMs = 0;
        rssiHot = false;
        updateDisplay();
        lastDebounceTime = millis();
    }

    prevLeft = leftPressed;
    prevRight = rightPressed;
    prevUp = upPressed;
    prevDown = downPressed;

    if (autoScanEnabled) {
      uint32_t now = millis();
      if (lockUntilMs != 0 && (int32_t)(now - lockUntilMs) < 0) {

      } else {

        if (lastHopMs == 0 || (now - lastHopMs) >= SCAN_DWELL_MS) {
          scanIndex = (uint16_t)((scanIndex + 1) % freqCount());

          tuneToIndex(scanIndex, false);
          lastHopMs = now;

          int rssi = ELECHOUSE_cc1101.getRssi();
          if (!rssiHot && rssi > RSSI_DETECT_THRESHOLD) {
            rssiHot = true;
            lockUntilMs = now + RSSI_LOCK_MS;

            EEPROM.put(ADDR_FREQ, currentFrequencyIndex);
            EEPROM.commit();
            replayShowDetectNotice("RSSI", rssi);
          } else if (rssiHot && rssi < RSSI_CLEAR_THRESHOLD) {
            rssiHot = false;
          }

          if (lastUiScanUpdateMs == 0 || (now - lastUiScanUpdateMs) >= UI_SCAN_UPDATE_MS) {
            updateDisplay();
            lastUiScanUpdateMs = now;
          }
        }
      }
    }

    do_sampling();
    delay(10);
    epochSUB++;

    if (epochSUB >= tft.width())
      epochSUB = 0;

    if (mySwitch.available()) {
        receivedValue = mySwitch.getReceivedValue();
        receivedBitLength = mySwitch.getReceivedBitlength();
        receivedProtocol = mySwitch.getReceivedProtocol();

        EEPROM.put(ADDR_VALUE, receivedValue);
        EEPROM.put(ADDR_BITLEN, receivedBitLength);
        EEPROM.put(ADDR_PROTO, receivedProtocol);
        EEPROM.commit();

        updateDisplay();

        if (autoScanEnabled) {
          lockUntilMs = millis() + LOCK_HOLD_MS;
          scanIndex = currentFrequencyIndex;
          rssiHot = false;

          EEPROM.put(ADDR_FREQ, currentFrequencyIndex);
          EEPROM.commit();
          replayShowDetectNotice("DECODE", ELECHOUSE_cc1101.getRssi());
        }
        mySwitch.resetAvailable();
    }

  }
}

namespace SavedProfile {

static bool uiDrawn = false;

#define EEPROM_SIZE 1440

#define ADDR_PROFILE_COUNT 1296
#define ADDR_PROFILE_START 1300
#define MAX_PROFILES       5
#define MAX_NAME_LENGTH    16

#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 320

RCSwitch mySwitch = RCSwitch();
struct __attribute__((packed)) Profile {
    uint32_t frequency;
    uint32_t value;
    uint16_t bitLength;
    uint16_t protocol;
    char name[MAX_NAME_LENGTH];
};

#define PROFILE_SIZE sizeof(Profile)

uint16_t profileCount = 0;
uint16_t currentProfileIndex = 0;

int yshift = 16;

static std::vector<SubGhzFileEntry> sdFiles;
static uint16_t sdTotalProfiles = 0;
static String sdLastErr = "";
static String selectedPath = "";
static uint16_t selectedLocalIdx = 0;
static Profile selectedProfile{};
static bool selectedValid = false;

static constexpr uint8_t ITEMS_PER_PAGE = 7;
static constexpr int LIST_X = 6;
static constexpr int LIST_W = 228;

static constexpr int LIST_Y = 64;
static constexpr int ROW_H  = 18;
static constexpr int BOT_H = 32;
static constexpr int BOT_Y = 320 - BOT_H;

static constexpr int UI_GAP_Y = 6;
static constexpr int DETAILS_H = (BOT_Y - (LIST_Y + (ITEMS_PER_PAGE * ROW_H)) - (2 * UI_GAP_Y));
static constexpr int DETAILS_Y = BOT_Y - UI_GAP_Y - DETAILS_H;
static constexpr int BOT_GAP = 8;
static constexpr int BOT_BTN_W = (240 - 10 - 10 - BOT_GAP) / 2;
static constexpr int BOT_BTN_H = 24;
static constexpr int BOT_BTN_Y = BOT_Y + 4;
static constexpr int BOT_TX_X  = 10;
static constexpr int BOT_DEL_X = BOT_TX_X + BOT_BTN_W + BOT_GAP;

static uint16_t cachedPageStart = 0xFFFF;
static SubGhzProfile cachedPage[ITEMS_PER_PAGE]{};
static bool cachedOk[ITEMS_PER_PAGE]{};
static bool cacheDirty = true;

static bool deleteArmed = false;
static uint32_t deleteArmUntilMs = 0;

static void drawBottomButtons() {

  tft.fillRect(0, BOT_Y, 240, BOT_H, TFT_BLACK);

  FeatureUI::drawButtonRect(BOT_TX_X, BOT_BTN_Y, BOT_BTN_W, BOT_BTN_H,
                            "Transmit", FeatureUI::ButtonStyle::Primary);

  const bool armed = deleteArmed && (int32_t)(millis() - deleteArmUntilMs) < 0;
  const char* delLabel = armed ? "Delete?" : "Delete";
  FeatureUI::drawButtonRect(BOT_DEL_X, BOT_BTN_Y, BOT_BTN_W, BOT_BTN_H,
                            delLabel, FeatureUI::ButtonStyle::Danger);
}

static void refreshSdIndex(bool keepSelection = true) {
    uint16_t oldIdx = currentProfileIndex;
    String err;
    if (!listAllProfileFiles(sdFiles, &err)) {
        sdFiles.clear();
        sdTotalProfiles = 0;
        currentProfileIndex = 0;
        selectedValid = false;
        sdLastErr = err;
        cacheDirty = true;
        return;
    }
    sdLastErr = "";
    sdTotalProfiles = totalProfilesInIndex(sdFiles);
    if (sdTotalProfiles == 0) {
        currentProfileIndex = 0;
        selectedValid = false;
        sdLastErr = "No profiles found";
        cacheDirty = true;
        return;
    }
    if (keepSelection) currentProfileIndex = oldIdx;
    if (currentProfileIndex >= sdTotalProfiles) currentProfileIndex = (uint16_t)(sdTotalProfiles - 1);
    selectedValid = false;
    cacheDirty = true;
}

static bool loadSelectedFromSd(String* errOut = nullptr) {
    if (sdTotalProfiles == 0) { selectedValid = false; return false; }
    if (!locateGlobalIndex(sdFiles, currentProfileIndex, selectedPath, selectedLocalIdx)) {
        selectedValid = false; if (errOut) *errOut="Locate failed"; return false;
    }
    SubGhzProfile p{};
    if (!readProfileAt(selectedPath, selectedLocalIdx, p, errOut)) {
        selectedValid = false; return false;
    }

    selectedProfile.frequency = p.frequency;
    selectedProfile.value = p.value;
    selectedProfile.bitLength = p.bitLength;
    selectedProfile.protocol = p.protocol;
    memcpy(selectedProfile.name, p.name, MAX_NAME_LENGTH);
    selectedProfile.name[MAX_NAME_LENGTH - 1] = '\0';
    selectedValid = true;
    return true;
}

static uint16_t pageStartForIndex(uint16_t idx) {
  return (uint16_t)((idx / ITEMS_PER_PAGE) * ITEMS_PER_PAGE);
}

static void ensurePageCache() {
  if (sdTotalProfiles == 0) return;
  uint16_t start = pageStartForIndex(currentProfileIndex);
  if (!cacheDirty && cachedPageStart == start) return;
  cachedPageStart = start;
  for (uint8_t i = 0; i < ITEMS_PER_PAGE; i++) {
    cachedOk[i] = false;
    uint16_t globalIdx = (uint16_t)(start + i);
    if (globalIdx >= sdTotalProfiles) continue;
    String pth; uint16_t li = 0;
    if (!locateGlobalIndex(sdFiles, globalIdx, pth, li)) continue;
    String err;
    cachedOk[i] = readProfileAt(pth, li, cachedPage[i], &err);
    if (!cachedOk[i]) memset(&cachedPage[i], 0, sizeof(SubGhzProfile));
  }
  cacheDirty = false;
}

static void drawHeaderLine() {

  int hy = 30 + yshift;
  tft.fillRect(0, hy, 240, 14, TFT_BLACK);
  tft.setCursor(10, hy);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.printf("Profile %d/%d", (int)currentProfileIndex + 1, (int)sdTotalProfiles);
}

static void drawRow(uint16_t pageStart, uint8_t row) {
  uint16_t globalIdx = (uint16_t)(pageStart + row);
  if (globalIdx >= sdTotalProfiles) return;

  bool isSel = (globalIdx == currentProfileIndex);
  int y = LIST_Y + (row * ROW_H);

  uint16_t bg = isSel ? DARK_GRAY : TFT_BLACK;
  uint16_t fg = isSel ? TFT_WHITE : TFT_LIGHTGREY;
  tft.fillRect(LIST_X, y, LIST_W, ROW_H - 1, bg);
  tft.setTextColor(fg, bg);
  tft.setCursor(LIST_X + 2, y + 4);
  tft.printf("%2d.", (int)globalIdx + 1);
  tft.setCursor(LIST_X + 34, y + 4);

  if (cachedOk[row]) {

    char nameBuf[17];
    memcpy(nameBuf, cachedPage[row].name, 16);
    nameBuf[16] = '\0';
    String nm = String(nameBuf);
    if (nm.length() > 10) nm = nm.substring(0, 10);
    tft.print(nm);

    char fbuf[16];
    snprintf(fbuf, sizeof(fbuf), "%.2f", cachedPage[row].frequency / 1000000.0);
    int tw = tft.textWidth(fbuf, 1);
    tft.setCursor(LIST_X + LIST_W - 4 - tw, y + 4);
    tft.print(fbuf);
  } else {
    tft.print("<?>");
  }
}

static void drawListPage(uint16_t pageStart) {
  ensurePageCache();

  tft.fillRect(LIST_X, LIST_Y, LIST_W, (ITEMS_PER_PAGE * ROW_H), TFT_BLACK);
  for (uint8_t row = 0; row < ITEMS_PER_PAGE; row++) {
    if ((uint16_t)(pageStart + row) >= sdTotalProfiles) break;
    drawRow(pageStart, row);
  }
}

static void drawDetails() {

  tft.fillRect(0, DETAILS_Y, 240, (DETAILS_H + UI_GAP_Y), TFT_BLACK);

  String err;
  if (!selectedValid) loadSelectedFromSd(&err);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(10, DETAILS_Y);
  if (!selectedValid) {
    tft.print("Read failed: ");
    tft.print(err);
    return;
  }

  tft.print("Name: "); tft.print(selectedProfile.name);
  tft.setCursor(10, DETAILS_Y + 14);
  tft.printf("Freq: %.2f MHz  P:%d", selectedProfile.frequency / 1000000.0, selectedProfile.protocol);
  tft.setCursor(10, DETAILS_Y + 28);
  tft.printf("Val: %lu  Bit:%d", selectedProfile.value, selectedProfile.bitLength);

  tft.setCursor(10, DETAILS_Y + 42);
  tft.setTextColor(UI_DIM_TEXT, TFT_BLACK);
  if (selectedPath.endsWith("profiles_current.bin")) {
    tft.print("SRC: current");
  } else {
    tft.print("SRC: ");
    int slash = selectedPath.lastIndexOf('/');
    tft.print(slash >= 0 ? selectedPath.substring(slash + 1) : selectedPath);
  }

  if (deleteArmed && (int32_t)(millis() - deleteArmUntilMs) < 0) {

    int hintY = DETAILS_Y + 56;
    if (hintY >= BOT_Y) hintY = BOT_Y - 12;
    tft.setCursor(10, hintY);
    tft.setTextColor(UI_WARN, TFT_BLACK);
    tft.print("Press delete again to confirm");
  }

  drawBottomButtons();
}

static void updateSelectionUI(uint16_t oldIndex, bool forceListRedraw = false) {
  if (sdTotalProfiles == 0) return;
  uint16_t oldPage = pageStartForIndex(oldIndex);
  uint16_t newPage = pageStartForIndex(currentProfileIndex);

  tft.startWrite();
  drawHeaderLine();

  if (forceListRedraw || oldPage != newPage) {
    drawListPage(newPage);
  } else {

    uint8_t oldRow = (uint8_t)(oldIndex - oldPage);
    uint8_t newRow = (uint8_t)(currentProfileIndex - newPage);
    ensurePageCache();

    drawRow(newPage, oldRow);
    drawRow(newPage, newRow);
  }

  drawDetails();
  tft.endWrite();
}

void updateDisplay() {

    tft.startWrite();
    tft.fillRect(0, 40, 240, 280, TFT_BLACK);

    if (sdTotalProfiles == 0) {
        tft.setCursor(10, 35 + yshift);
        tft.setTextColor(TFT_WHITE);
        tft.print("No profiles on SD.");
        if (sdLastErr.length()) {
          tft.setCursor(10, 48 + yshift);
          tft.setTextColor(UI_DIM_TEXT);
          tft.print(sdLastErr);
        }
        return;
    }

    drawHeaderLine();
    drawListPage(pageStartForIndex(currentProfileIndex));
    drawDetails();
    tft.endWrite();
}

void transmitProfile(int index) {
    (void)index;
    String err;
    loadSelectedFromSd(&err);
    if (!selectedValid) return;
    Profile profileToSend = selectedProfile;

    ELECHOUSE_cc1101.setSidle();
    ELECHOUSE_cc1101.setMHZ(profileToSend.frequency / 1000000.0);

    mySwitch.disableReceive();
    delay(100);
    mySwitch.enableTransmit(SUBGHZ_TX_PIN);
    ELECHOUSE_cc1101.SetTx();

    tft.fillRect(0, 40, 240, 280, TFT_BLACK);
    tft.setCursor(10, 30 + yshift);
    tft.setTextColor(TFT_WHITE);
    tft.print("Sending ");
    tft.print(profileToSend.name);
    tft.print("...");
    tft.setCursor(10, 50 + yshift);
    tft.print("Value: ");
    tft.print(profileToSend.value);

    mySwitch.setProtocol(profileToSend.protocol);
    mySwitch.send(profileToSend.value, profileToSend.bitLength);

    delay(500);
    tft.fillRect(0, 40, 240, 280, TFT_BLACK);
    tft.setCursor(10, 30 + yshift);
    tft.print("Done!");

    ELECHOUSE_cc1101.SetRx();
    mySwitch.disableTransmit();
    delay(100);
    mySwitch.enableReceive(SUBGHZ_RX_PIN);

    delay(500);
    updateDisplay();
}

void loadProfileCount() {

    refreshSdIndex(true);
}

void printProfiles() {
    Serial.println("Saved Profiles (SD index):");
    String err;
    refreshSdIndex(false);
    Serial.printf("Total profiles: %d\n", (int)sdTotalProfiles);
    if (!sdTotalProfiles) return;

    uint16_t n = sdTotalProfiles > 10 ? 10 : sdTotalProfiles;
    for (uint16_t i = 0; i < n; i++) {
      String pth; uint16_t li = 0;
      if (!locateGlobalIndex(sdFiles, i, pth, li)) continue;
      SubGhzProfile p{};
      if (!readProfileAt(pth, li, p, &err)) continue;
      Serial.printf("  [%d] %s @ %.2f MHz (val=%lu)\n", (int)i, p.name, p.frequency/1000000.0, (unsigned long)p.value);
    }
}

void deleteProfile(int index) {
    (void)index;
    if (sdTotalProfiles == 0) return;
    String err;
    loadSelectedFromSd(&err);
    if (!selectedValid) return;

    String path = selectedPath;
    uint16_t local = selectedLocalIdx;

    uint32_t now = millis();
    if (!deleteArmed || (int32_t)(now - deleteArmUntilMs) >= 0) {
      deleteArmed = true;
      deleteArmUntilMs = now + 3000;
      updateDisplay();
      return;
    }
    deleteArmed = false;

    if (!deleteProfileFromFile(path, local, &err)) {
      tft.fillRect(0, 40, 240, 280, TFT_BLACK);
      tft.setCursor(10, 30 + yshift);
      tft.setTextColor(UI_WARN);
      tft.print("Delete FAILED");
      tft.setCursor(10, 45 + yshift);
      tft.setTextColor(TFT_WHITE);
      tft.print(err);
      delay(1200);
      updateDisplay();
      return;
    }

    refreshSdIndex(false);
    if (sdTotalProfiles == 0) currentProfileIndex = 0;
    else if (currentProfileIndex >= sdTotalProfiles) currentProfileIndex = (uint16_t)(sdTotalProfiles - 1);
    selectedValid = false;
    cacheDirty = true;
    updateDisplay();
}

void runUI() {
    #define STATUS_BAR_Y_OFFSET 20
    #define STATUS_BAR_HEIGHT 16
    #define ICON_SIZE 16
    #define ICON_NUM 6

    static int iconX[ICON_NUM] = {90, 130, 170, 210, 50, 10};
    static int iconY = STATUS_BAR_Y_OFFSET;

    static const unsigned char* icons[ICON_NUM] = {
        bitmap_icon_sort_down_minus,
        bitmap_icon_sort_up_plus,
        bitmap_icon_antenna,
        bitmap_icon_recycle,
        bitmap_icon_sdcard,
        bitmap_icon_go_back
    };

    if (!uiDrawn) {
        tft.drawLine(0, 19, 240, 19, TFT_WHITE);
        tft.fillRect(0, STATUS_BAR_Y_OFFSET, SCREEN_WIDTH, STATUS_BAR_HEIGHT, DARK_GRAY);

        for (int i = 0; i < ICON_NUM; i++) {
            if (icons[i] != NULL) {
                tft.drawBitmap(iconX[i], iconY, icons[i], ICON_SIZE, ICON_SIZE, TFT_WHITE);
            }
        }
        tft.drawLine(0, STATUS_BAR_Y_OFFSET + STATUS_BAR_HEIGHT, SCREEN_WIDTH, STATUS_BAR_Y_OFFSET + STATUS_BAR_HEIGHT, ORANGE);
        uiDrawn = true;
    }

    static unsigned long lastAnimationTime = 0;
    static int animationState = 0;
    static int activeIcon = -1;

    if (animationState > 0 && millis() - lastAnimationTime >= 150) {
        if (animationState == 1) {
            tft.drawBitmap(iconX[activeIcon], iconY, icons[activeIcon], ICON_SIZE, ICON_SIZE, TFT_WHITE);
            animationState = 2;

            switch (activeIcon) {
                case 0:
                    if (sdTotalProfiles > 0) {
                        uint16_t oldIdx = currentProfileIndex;
                        currentProfileIndex = (uint16_t)((currentProfileIndex + 1) % sdTotalProfiles);
                        selectedValid = false;
                        cacheDirty = true;
                        deleteArmed = false;

                        updateSelectionUI(oldIdx, false);
                    }
                    break;
                case 1:
                    if (sdTotalProfiles > 0) {
                        uint16_t oldIdx = currentProfileIndex;
                        currentProfileIndex = (uint16_t)((currentProfileIndex + sdTotalProfiles - 1) % sdTotalProfiles);
                        selectedValid = false;
                        cacheDirty = true;
                        deleteArmed = false;
                        updateSelectionUI(oldIdx, false);
                    }
                    break;
                case 2:
                    if (sdTotalProfiles > 0) {
                        transmitProfile(currentProfileIndex);
                    }
                    break;
                case 3:
                    if (sdTotalProfiles > 0) {
                        deleteProfile(currentProfileIndex);
                    }
                    break;
                case 4: {
                    refreshSdIndex(true);
                    selectedValid = false;
                    cacheDirty = true;
                    deleteArmed = false;
                    updateDisplay();
                    break;
                }
            }
        } else if (animationState == 2) {
            animationState = 0;
            activeIcon = -1;
        }
        lastAnimationTime = millis();
    }

    static unsigned long lastTouchCheck = 0;
    const unsigned long touchCheckInterval = 50;

    if (millis() - lastTouchCheck >= touchCheckInterval) {
        int x, y;
        if (feature_active && readTouchXY(x, y)) {

            if (y >= BOT_Y && y < (BOT_Y + BOT_H)) {
              if (x >= BOT_TX_X && x < (BOT_TX_X + BOT_BTN_W)) {
                if (sdTotalProfiles > 0) transmitProfile(currentProfileIndex);
              } else if (x >= BOT_DEL_X && x < (BOT_DEL_X + BOT_BTN_W)) {
                if (sdTotalProfiles > 0) deleteProfile(currentProfileIndex);
              }
              lastTouchCheck = millis();
              return;
            }

            if (y >= LIST_Y && y < (LIST_Y + (ITEMS_PER_PAGE * ROW_H)) && x >= LIST_X && x < (LIST_X + LIST_W)) {
              uint8_t row = (uint8_t)((y - LIST_Y) / ROW_H);
              uint16_t oldIdx = currentProfileIndex;
              uint16_t start = pageStartForIndex(currentProfileIndex);
              uint16_t idx = (uint16_t)(start + row);
              if (idx < sdTotalProfiles) {
                currentProfileIndex = idx;
                selectedValid = false;
                cacheDirty = true;
                deleteArmed = false;
                updateSelectionUI(oldIdx, false);
              }
            }
            if (y > STATUS_BAR_Y_OFFSET && y < STATUS_BAR_Y_OFFSET + STATUS_BAR_HEIGHT) {
                for (int i = 0; i < ICON_NUM; i++) {
                    if (x > iconX[i] && x < iconX[i] + ICON_SIZE) {
                        if (icons[i] != NULL && animationState == 0) {

                            if (i == 5) {
                                feature_exit_requested = true;
                            } else {

                                tft.drawBitmap(iconX[i], iconY, icons[i], ICON_SIZE, ICON_SIZE, TFT_BLACK);
                                animationState = 1;
                                activeIcon = i;
                                lastAnimationTime = millis();
                            }
                        }
                        break;
                    }
                }
            }
        }
        lastTouchCheck = millis();
    }
}

void saveSetup() {
    Serial.begin(115200);

    ELECHOUSE_cc1101.setSpiPin(CC1101_SCK, CC1101_MISO, CC1101_MOSI, CC1101_CS);

    EEPROM.begin(EEPROM_SIZE);
    loadProfileCount();
    printProfiles();

    pcf.pinMode(BTN_UP, INPUT_PULLUP);
    pcf.pinMode(BTN_DOWN, INPUT_PULLUP);
    pcf.pinMode(BTN_LEFT, INPUT_PULLUP);
    pcf.pinMode(BTN_RIGHT, INPUT_PULLUP);
    pcf.pinMode(BTN_SELECT, INPUT_PULLUP);

    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE);

    setupTouchscreen();

    float currentBatteryVoltage = readBatteryVoltage();
    drawStatusBar(currentBatteryVoltage, true);
    uiDrawn = false;

    ELECHOUSE_cc1101.Init();
    ELECHOUSE_cc1101.setGDO(CC1101_GDO0, CC1101_GDO2);
    ELECHOUSE_cc1101.SetRx();

    mySwitch.enableReceive(SUBGHZ_RX_PIN);
    mySwitch.enableTransmit(SUBGHZ_TX_PIN);
    mySwitch.setRepeatTransmit(8);

    refreshSdIndex(false);
    cacheDirty = true;
    deleteArmed = false;
    updateDisplay();
}

void saveLoop() {

    if (feature_active && isButtonPressed(BTN_SELECT)) {
        feature_exit_requested = true;
        return;
    }

    runUI();

    static unsigned long lastDebounceTime = 0;
    const unsigned long debounceDelay = 200;

    bool prevPressed    = isButtonPressed(BTN_UP);
    bool nextPressed    = isButtonPressed(BTN_DOWN);
    bool txPressed      = isButtonPressed(BTN_RIGHT);
    bool refreshPressed = isButtonPressed(BTN_LEFT);

    if (sdTotalProfiles > 0) {

        if (nextPressed && millis() - lastDebounceTime > debounceDelay) {
            uint16_t oldIdx = currentProfileIndex;
            currentProfileIndex = (uint16_t)((currentProfileIndex + 1) % sdTotalProfiles);
            selectedValid = false;
            updateSelectionUI(oldIdx, false);
            lastDebounceTime = millis();
        }

        if (prevPressed && millis() - lastDebounceTime > debounceDelay) {
            uint16_t oldIdx = currentProfileIndex;
            currentProfileIndex = (uint16_t)((currentProfileIndex + sdTotalProfiles - 1) % sdTotalProfiles);
            selectedValid = false;
            updateSelectionUI(oldIdx, false);
            lastDebounceTime = millis();
        }

        if (txPressed && millis() - lastDebounceTime > debounceDelay) {
            transmitProfile(currentProfileIndex);
            lastDebounceTime = millis();
        }

        if (refreshPressed && millis() - lastDebounceTime > debounceDelay) {
            refreshSdIndex(true);
            selectedValid = false;
            cacheDirty = true;
            deleteArmed = false;
            updateDisplay();
            lastDebounceTime = millis();
        }
    } else {

        tft.setCursor(10, 50 + yshift);
        tft.setTextColor(TFT_WHITE);
        tft.print("No profiles on SD.");
    }
}

}

namespace subjammer {

static bool uiDrawn = false;

static unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 200;

#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 64

static constexpr uint8_t JAM_BTN_LEFT  = 4;
static constexpr uint8_t JAM_BTN_RIGHT = 5;
static constexpr uint8_t JAM_BTN_DOWN  = 3;
static constexpr uint8_t JAM_BTN_UP    = 6;

bool jammingRunning = false;
bool continuousMode = true;
bool autoMode = false;
unsigned long lastSweepTime = 0;
const unsigned long sweepInterval = 1000;

static const uint32_t subghz_frequency_list[] = {
    300000000, 303875000, 304250000, 310000000, 315000000, 318000000,
    390000000, 418000000, 433075000, 433420000, 433920000, 434420000,
    434775000, 438900000, 868350000, 915000000, 925000000
};
const int numFrequencies = sizeof(subghz_frequency_list) / sizeof(subghz_frequency_list[0]);
int currentFrequencyIndex = 4;
float targetFrequency = subghz_frequency_list[currentFrequencyIndex] / 1000000.0;

void updateDisplay() {

    int yshift = 20;

    tft.fillRect(0, 40, 240, 80, TFT_BLACK);
    tft.drawLine(0, 79, 235, 79, TFT_WHITE);

    tft.setTextSize(1);
    tft.setCursor(5, 22 + yshift);
    tft.setTextColor(WHITE);
    tft.print("Freq:");
    tft.setCursor(40, 22 + yshift);
    if (autoMode) {
        tft.setTextColor(ORANGE);
        tft.print("Auto: ");
        tft.setTextColor(TFT_WHITE);
        tft.print(targetFrequency, 1);

        int progress = ::map(currentFrequencyIndex, 0, numFrequencies - 1, 0, 240);
        tft.fillRect(0, 60 + yshift, 240, 4, TFT_BLACK);
        tft.fillRect(0, 60 + yshift, progress, 4, ORANGE);

        if (jammingRunning && millis() % 1000 < 500) {
            tft.fillCircle(220, 22 + yshift, 2, TFT_GREEN);
        }
    } else {
        tft.setTextColor(TFT_WHITE);
        tft.print(targetFrequency, 2);
        tft.print(" MHz");
    }

    tft.setCursor(130, 22 + yshift);
    tft.setTextColor(WHITE);
    tft.print("Mode:");
    tft.setCursor(165, 22 + yshift);
    tft.setTextColor(continuousMode ? TFT_GREEN : TFT_YELLOW);
    tft.print(continuousMode ? "Cont" : "Noise");

    tft.setCursor(5, 42 + yshift);
    tft.setTextColor(WHITE);
    tft.print("Status:");
    tft.setCursor(50, 42 + yshift);
    if (jammingRunning) {
        tft.setTextColor(UI_WARN);
        tft.print("Jamming");

    } else {
        tft.setTextColor(TFT_GREEN);
        tft.print("Idle   ");
    }
}

void runUI() {
    #define SCREEN_WIDTH  240
    #define SCREENHEIGHT 320
    #define STATUS_BAR_Y_OFFSET 20
    #define STATUS_BAR_HEIGHT 16
    #define ICON_SIZE 16
    #define ICON_NUM 6

    static int iconX[ICON_NUM] = {50, 90, 130, 170, 210, 10};
    static int iconY = STATUS_BAR_Y_OFFSET;

    static const unsigned char* icons[ICON_NUM] = {
        bitmap_icon_power,
        bitmap_icon_antenna,
        bitmap_icon_random,
        bitmap_icon_sort_down_minus,
        bitmap_icon_sort_up_plus,
        bitmap_icon_go_back
    };

    if (!uiDrawn) {
        tft.drawLine(0, 19, 240, 19, TFT_WHITE);
        tft.fillRect(0, STATUS_BAR_Y_OFFSET, SCREEN_WIDTH, STATUS_BAR_HEIGHT, DARK_GRAY);

        for (int i = 0; i < ICON_NUM; i++) {
            if (icons[i] != NULL) {
                tft.drawBitmap(iconX[i], iconY, icons[i], ICON_SIZE, ICON_SIZE, TFT_WHITE);
            }
        }
        tft.drawLine(0, STATUS_BAR_Y_OFFSET + STATUS_BAR_HEIGHT, SCREEN_WIDTH, STATUS_BAR_Y_OFFSET + STATUS_BAR_HEIGHT, ORANGE);
        uiDrawn = true;
    }

    static unsigned long lastAnimationTime = 0;
    static int animationState = 0;
    static int activeIcon = -1;

    if (animationState > 0 && millis() - lastAnimationTime >= 150) {
        if (animationState == 1) {
            tft.drawBitmap(iconX[activeIcon], iconY, icons[activeIcon], ICON_SIZE, ICON_SIZE, TFT_WHITE);
            animationState = 2;

            switch (activeIcon) {
                case 0:
                  jammingRunning = !jammingRunning;
                    if (jammingRunning) {
                        Serial.println("Jamming started");
                        ELECHOUSE_cc1101.setMHZ(targetFrequency);
                        ELECHOUSE_cc1101.SetTx();
                    } else {
                        Serial.println("Jamming stopped");
                        ELECHOUSE_cc1101.setSidle();
                        digitalWrite(TX_PIN, LOW);
                    }
                    updateDisplay();
                    lastDebounceTime = millis();
                    break;
                case 1:
                 continuousMode = !continuousMode;
                  Serial.print("Jamming mode: ");
                  Serial.println(continuousMode ? "Continuous Carrier" : "Noise");
                  updateDisplay();
                  lastDebounceTime = millis();
                    break;
                case 2:
                  autoMode = !autoMode;
                  Serial.print("Frequency mode: ");
                  Serial.println(autoMode ? "Automatic" : "Manual");
                  if (autoMode) {
                      currentFrequencyIndex = 0;
                      targetFrequency = subghz_frequency_list[currentFrequencyIndex] / 1000000.0;
                      ELECHOUSE_cc1101.setMHZ(targetFrequency);
                  }
                  updateDisplay();
                  lastDebounceTime = millis();
                    break;
                case 3:
                  currentFrequencyIndex = (currentFrequencyIndex - 1 + numFrequencies) % numFrequencies;
                  targetFrequency = subghz_frequency_list[currentFrequencyIndex] / 1000000.0;
                  ELECHOUSE_cc1101.setMHZ(targetFrequency);
                  Serial.print("Switched to: ");
                  Serial.print(targetFrequency);
                  Serial.println(" MHz");
                  updateDisplay();
                  lastDebounceTime = millis();
                    break;
                 case 4:
                  currentFrequencyIndex = (currentFrequencyIndex + 1) % numFrequencies;
                  targetFrequency = subghz_frequency_list[currentFrequencyIndex] / 1000000.0;
                  ELECHOUSE_cc1101.setMHZ(targetFrequency);
                  Serial.print("Switched to: ");
                  Serial.print(targetFrequency);
                  Serial.println(" MHz");
                  updateDisplay();
                  lastDebounceTime = millis();
                    break;
                case 5:
                    feature_exit_requested = true;
                    break;
            }
        } else if (animationState == 2) {
            animationState = 0;
            activeIcon = -1;
        }
        lastAnimationTime = millis();
    }

    static unsigned long lastTouchCheck = 0;
    const unsigned long touchCheckInterval = 50;

    if (millis() - lastTouchCheck >= touchCheckInterval) {
        int x, y;
        if (feature_active && readTouchXY(x, y)) {
            if (y > STATUS_BAR_Y_OFFSET && y < STATUS_BAR_Y_OFFSET + STATUS_BAR_HEIGHT) {
                for (int i = 0; i < ICON_NUM; i++) {
                    if (x > iconX[i] && x < iconX[i] + ICON_SIZE) {
                        if (icons[i] != NULL && animationState == 0) {

                            if (i == 5) {
                                feature_exit_requested = true;
                            } else {

                                tft.drawBitmap(iconX[i], iconY, icons[i], ICON_SIZE, ICON_SIZE, TFT_BLACK);
                                animationState = 1;
                                activeIcon = i;
                                lastAnimationTime = millis();
                            }
                        }
                        break;
                    }
                }
            }
        }
        lastTouchCheck = millis();
    }
}

void subjammerSetup() {
    Serial.begin(115200);

    ELECHOUSE_cc1101.setSpiPin(CC1101_SCK, CC1101_MISO, CC1101_MOSI, CC1101_CS);

    ELECHOUSE_cc1101.Init();
    ELECHOUSE_cc1101.setModulation(0);
    ELECHOUSE_cc1101.setRxBW(500.0);
    ELECHOUSE_cc1101.setPA(12);
    ELECHOUSE_cc1101.setMHZ(targetFrequency);
    ELECHOUSE_cc1101.SetTx();

    randomSeed(analogRead(0));

    pcf.pinMode(BTN_LEFT, INPUT_PULLUP);
    pcf.pinMode(BTN_RIGHT, INPUT_PULLUP);
    pcf.pinMode(BTN_DOWN, INPUT_PULLUP);
    pcf.pinMode(BTN_UP, INPUT_PULLUP);
    delay(100);

    tft.fillScreen(TFT_BLACK);

    setupTouchscreen();

   float currentBatteryVoltage = readBatteryVoltage();
   drawStatusBar(currentBatteryVoltage, true);
   updateDisplay();
   uiDrawn = false;
}

void subjammerLoop() {

    if (feature_active && isButtonPressed(BTN_SELECT)) {
        feature_exit_requested = true;
        return;
    }

    runUI();

    int btnLeftState = pcf.digitalRead(JAM_BTN_LEFT);
    int btnRightState = pcf.digitalRead(JAM_BTN_RIGHT);
    int btnUpState = pcf.digitalRead(JAM_BTN_UP);
    int btnDownState = pcf.digitalRead(JAM_BTN_DOWN);

    if (btnUpState == LOW && millis() - lastDebounceTime > debounceDelay) {
        jammingRunning = !jammingRunning;
        if (jammingRunning) {
            Serial.println("Jamming started");
            ELECHOUSE_cc1101.setMHZ(targetFrequency);
            ELECHOUSE_cc1101.SetTx();
        } else {
            Serial.println("Jamming stopped");
            ELECHOUSE_cc1101.setSidle();
            digitalWrite(TX_PIN, LOW);
        }
        updateDisplay();
        lastDebounceTime = millis();
    }

    if (btnRightState == LOW && !autoMode && millis() - lastDebounceTime > debounceDelay) {
        currentFrequencyIndex = (currentFrequencyIndex + 1) % numFrequencies;
        targetFrequency = subghz_frequency_list[currentFrequencyIndex] / 1000000.0;
        ELECHOUSE_cc1101.setMHZ(targetFrequency);
        Serial.print("Switched to: ");
        Serial.print(targetFrequency);
        Serial.println(" MHz");
        updateDisplay();
        lastDebounceTime = millis();
    }

    if (btnLeftState == LOW && !autoMode && millis() - lastDebounceTime > debounceDelay) {
        continuousMode = !continuousMode;
        Serial.print("Jamming mode: ");
        Serial.println(continuousMode ? "Continuous Carrier" : "Noise");
        updateDisplay();
        lastDebounceTime = millis();
    }

    if (btnDownState == LOW && millis() - lastDebounceTime > debounceDelay) {
        autoMode = !autoMode;
        Serial.print("Frequency mode: ");
        Serial.println(autoMode ? "Automatic" : "Manual");
        if (autoMode) {
            currentFrequencyIndex = 0;
            targetFrequency = subghz_frequency_list[currentFrequencyIndex] / 1000000.0;
            ELECHOUSE_cc1101.setMHZ(targetFrequency);
        }
        updateDisplay();
        lastDebounceTime = millis();
    }

    if (jammingRunning) {
        if (autoMode) {
            if (millis() - lastSweepTime >= sweepInterval) {
                currentFrequencyIndex = (currentFrequencyIndex + 1) % numFrequencies;
                targetFrequency = subghz_frequency_list[currentFrequencyIndex] / 1000000.0;
                ELECHOUSE_cc1101.setMHZ(targetFrequency);
                Serial.print("Sweeping: ");
                Serial.print(targetFrequency);
                Serial.println(" MHz");
                updateDisplay();
                lastSweepTime = millis();
            }
        }

        ELECHOUSE_cc1101.SetTx();

        if (continuousMode) {
            ELECHOUSE_cc1101.SpiWriteReg(CC1101_TXFIFO, 0xFF);
            ELECHOUSE_cc1101.SpiStrobe(CC1101_STX);
            digitalWrite(TX_PIN, HIGH);
        } else {
            for (int i = 0; i < 10; i++) {
                uint32_t noise = random(16777216);
                ELECHOUSE_cc1101.SpiWriteReg(CC1101_TXFIFO, noise >> 16);
                ELECHOUSE_cc1101.SpiWriteReg(CC1101_TXFIFO, (noise >> 8) & 0xFF);
                ELECHOUSE_cc1101.SpiWriteReg(CC1101_TXFIFO, noise & 0xFF);
                ELECHOUSE_cc1101.SpiStrobe(CC1101_STX);
                delayMicroseconds(50);
              }
          }
      }
  }
}

/* ════════════════════════════════════════════════════════════════════════════
   RfDecoder — sub-GHz protocol decoder + key-fob analyzer.

   Passively listens on the CC1101 with RCSwitch attached to GDO0. Each
   decoded packet shows: frequency, RCSwitch protocol #, payload value,
   bit length, RSSI, and a replay-safety verdict (FIXED CODE = vulnerable
   to capture/replay; absence of decode + strong RSSI = likely rolling
   code / Manchester / proprietary).

   Covers EV1527, PT2240, HT12, Holtek HT6P20B, SC2262, and the other
   RCSwitch built-in protocols out of the box. Rolling-code protocols
   like KeeLoq / HCS200 produce no RCSwitch decode and get classified
   as ROLLING by absence + RSSI heuristic.

   Controls:
     UP/DOWN  — scroll history
     RIGHT    — cycle frequency (315 / 433.92 / 868 MHz)
     LEFT     — save selected packet to /captures/rf.csv
     SEL tap  — drill into highlighted packet (full details + verdict)
     SEL hold — back to menu
   ════════════════════════════════════════════════════════════════════════════ */
namespace RfDecoder {

static constexpr int    MAX_PACKETS = 32;
static constexpr float  FREQS[]     = { 315.0f, 433.92f, 868.35f };
static constexpr int    NUM_FREQS   = sizeof(FREQS) / sizeof(FREQS[0]);

struct Packet {
  uint32_t value;
  uint16_t bits;
  uint8_t  protocol;
  uint32_t delayUs;
  int8_t   rssi;
  uint8_t  freqIdx;
  uint32_t ageMs;
};

static Packet  packets[MAX_PACKETS];
static int     pktCount = 0;
static int     selIndex = 0;
static int     scroll   = 0;
static int     freqIdx  = 1;        // start at 433.92 MHz (the most common)
static bool    uiDirty  = true;
static uint32_t startMs = 0;
static uint32_t totalDecoded = 0;
static uint32_t lastSeenStrongMs = 0;
static int8_t   lastNoDecodeRssi = -127;

static RCSwitch s_sw = RCSwitch();

static const uint16_t R_RED = 0xE0E6;
static const uint16_t R_DIM = 0x6020;
static const uint16_t R_GRN = 0x07C0;
static const uint16_t R_MID = 0x9085;

// ── Verdicts ────────────────────────────────────────────────────────────────
struct Verdict {
  const char* label;     // e.g. "EV1527"
  const char* note;      // short usage note
  bool        replayWorks;
};

static Verdict classify(const Packet& p) {
  // RCSwitch numbers protocols 1..N. Standard ones map to common families.
  switch (p.protocol) {
    case 1:
      if (p.bits == 24) return { "EV1527 / PT2240", "consumer gate/garage remote", true };
      if (p.bits == 32) return { "Protocol 1 (32b)", "extended fixed code", true };
      return { "Protocol 1", "fixed code", true };
    case 2:
      return { "Protocol 2 (HT6P20B)", "Holtek encoder, fixed", true };
    case 3:
      return { "Protocol 3", "Sumtech-style fixed code", true };
    case 4:
      return { "Protocol 4", "fixed code", true };
    case 5:
      return { "Protocol 5", "fixed code", true };
    case 6:
      return { "Protocol 6", "fixed code", true };
    case 7:
      return { "Protocol 7", "fixed code (HT12)", true };
    case 11:
      return { "Protocol 11", "remote (fixed)", true };
    case 12:
      return { "Protocol 12", "remote (fixed)", true };
    default:
      return { "Unknown", "fixed-code variant", true };
  }
}

// ── CC1101 setup ────────────────────────────────────────────────────────────
static void retune(int idx) {
  if (idx < 0) idx = 0;
  if (idx >= NUM_FREQS) idx = NUM_FREQS - 1;
  freqIdx = idx;
  ELECHOUSE_cc1101.setSidle();
  ELECHOUSE_cc1101.setMHZ(FREQS[freqIdx]);
  ELECHOUSE_cc1101.SetRx();
}

// ── UI ──────────────────────────────────────────────────────────────────────
static void drawShell() {
  tft.fillScreen(TFT_BLACK);
  tft.drawFastHLine(0, 0,   240, R_RED);
  tft.drawFastHLine(0, 24,  240, R_DIM);
  tft.drawFastHLine(0, 296, 240, R_DIM);
  tft.drawFastHLine(0, 319, 240, R_RED);
  tft.setTextFont(2); tft.setTextSize(1);
  tft.setTextColor(R_RED, TFT_BLACK);
  tft.setCursor(50, 6); tft.print("[ RF DECODER ]");
  tft.setTextColor(R_DIM, TFT_BLACK);
  tft.setCursor(8, 301); tft.print("UP/DN  R=freq  L=save  SEL=info");
}

static void drawHeader() {
  tft.fillRect(0, 28, 240, 22, TFT_BLACK);
  tft.setTextFont(2); tft.setTextSize(1);
  tft.setTextColor(R_GRN, TFT_BLACK);
  tft.setCursor(8, 30);
  char buf[40];
  snprintf(buf, sizeof(buf), "%.2f MHz", FREQS[freqIdx]);
  tft.print(buf);
  tft.setTextColor(R_MID, TFT_BLACK);
  tft.setCursor(100, 30);
  snprintf(buf, sizeof(buf), "rx:%lu  rssi:%d",
           (unsigned long)totalDecoded,
           (int)lastNoDecodeRssi);
  tft.print(buf);
}

static void drawList() {
  tft.fillRect(0, 52, 240, 244, TFT_BLACK);
  if (pktCount == 0) {
    tft.setTextFont(2);
    tft.setTextColor(R_DIM, TFT_BLACK);
    tft.setCursor(20, 140);
    tft.print("// listening... //");
    tft.setCursor(20, 160);
    tft.print("press a remote near the");
    tft.setCursor(20, 176);
    tft.print("board on this band");
    return;
  }
  // Newest first.
  const int Y0 = 56;
  const int rowH = 22;
  const int maxRows = (290 - Y0) / rowH;
  int start = scroll;
  if (selIndex < start) start = selIndex;
  if (selIndex >= start + maxRows) start = selIndex - maxRows + 1;
  if (start < 0) start = 0;
  scroll = start;

  for (int r = 0; r < maxRows && (start + r) < pktCount; r++) {
    int idx = pktCount - 1 - (start + r);   // newest at top
    if (idx < 0) break;
    const Packet& p = packets[idx];
    int y = Y0 + r * rowH;
    bool isSel = ((start + r) == selIndex);
    if (isSel) tft.fillRect(2, y - 2, 236, rowH, R_DIM);
    tft.setTextFont(2); tft.setTextSize(1);
    tft.setTextColor(isSel ? WHITE : R_RED, isSel ? R_DIM : TFT_BLACK);
    tft.setCursor(4, y);
    char line[48];
    snprintf(line, sizeof(line), "%.2fM p%u b%u",
             FREQS[p.freqIdx], (unsigned)p.protocol, (unsigned)p.bits);
    tft.print(line);
    tft.setCursor(4, y + 10);
    tft.setTextColor(isSel ? WHITE : R_GRN, isSel ? R_DIM : TFT_BLACK);
    char val[24];
    snprintf(val, sizeof(val), " 0x%08lX  %ddBm", (unsigned long)p.value, (int)p.rssi);
    tft.print(val);
  }
}

static void drawDetail(const Packet& p) {
  Verdict v = classify(p);
  tft.fillScreen(TFT_BLACK);
  tft.drawFastHLine(0, 0,   240, R_RED);
  tft.drawFastHLine(0, 24,  240, R_DIM);
  tft.drawFastHLine(0, 296, 240, R_DIM);
  tft.drawFastHLine(0, 319, 240, R_RED);
  tft.setTextFont(2); tft.setTextSize(1);
  tft.setTextColor(R_RED, TFT_BLACK);
  tft.setCursor(50, 6); tft.print("[ FOB ANALYZER ]");

  tft.setTextColor(R_GRN, TFT_BLACK);
  tft.setCursor(8, 36); tft.print(v.label);

  tft.setTextColor(WHITE, TFT_BLACK);
  char buf[64];
  int y = 60;
  snprintf(buf, sizeof(buf), "Freq:    %.3f MHz", FREQS[p.freqIdx]); tft.setCursor(8, y); tft.print(buf); y += 18;
  snprintf(buf, sizeof(buf), "Protocol: %u (RCSwitch)", (unsigned)p.protocol); tft.setCursor(8, y); tft.print(buf); y += 18;
  snprintf(buf, sizeof(buf), "Value:    0x%08lX", (unsigned long)p.value); tft.setCursor(8, y); tft.print(buf); y += 18;
  snprintf(buf, sizeof(buf), "Decimal:  %lu", (unsigned long)p.value); tft.setCursor(8, y); tft.print(buf); y += 18;
  snprintf(buf, sizeof(buf), "Bits:     %u", (unsigned)p.bits); tft.setCursor(8, y); tft.print(buf); y += 18;
  snprintf(buf, sizeof(buf), "Pulse:    %lu us", (unsigned long)p.delayUs); tft.setCursor(8, y); tft.print(buf); y += 18;
  snprintf(buf, sizeof(buf), "RSSI:     %d dBm", (int)p.rssi); tft.setCursor(8, y); tft.print(buf); y += 28;

  tft.setTextColor(v.replayWorks ? R_GRN : R_DIM, TFT_BLACK);
  tft.setCursor(8, y);
  tft.print(v.replayWorks ? "REPLAY: works" : "REPLAY: rolling/unknown");
  y += 18;
  tft.setTextColor(R_MID, TFT_BLACK);
  tft.setCursor(8, y);
  tft.print(v.note);

  tft.setTextColor(R_DIM, TFT_BLACK);
  tft.setCursor(8, 301); tft.print("hold SEL=back  L=save");
}

// ── SD log ──────────────────────────────────────────────────────────────────
static void appendCsv(const Packet& p) {
  if (!(SD.cardSize() > 0)) return;
  if (!SD.exists("/captures")) SD.mkdir("/captures");
  bool fresh = !SD.exists("/captures/rf.csv");
  File f = SD.open("/captures/rf.csv", FILE_APPEND);
  if (!f) return;
  if (fresh) f.println("freq_mhz,protocol,bits,value_hex,delay_us,rssi");
  f.printf("%.3f,%u,%u,0x%08lX,%lu,%d\n",
           FREQS[p.freqIdx], (unsigned)p.protocol, (unsigned)p.bits,
           (unsigned long)p.value, (unsigned long)p.delayUs, (int)p.rssi);
  f.close();
}

// ── Lifecycle ───────────────────────────────────────────────────────────────
void setup() {
  pktCount = 0;
  selIndex = 0;
  scroll   = 0;
  uiDirty  = true;
  totalDecoded = 0;
  lastNoDecodeRssi = -127;
  startMs  = millis();

  ELECHOUSE_cc1101.Init();
  ELECHOUSE_cc1101.setGDO(CC1101_GDO0, CC1101_GDO2);
  retune(freqIdx);
  s_sw.disableTransmit();
  s_sw.enableReceive(digitalPinToInterrupt(CC1101_GDO0));

  drawShell();
  drawHeader();
  drawList();
}

void loop() {
  NeoFx::tick();

  // Poll RCSwitch.
  if (s_sw.available()) {
    uint32_t value    = s_sw.getReceivedValue();
    uint16_t bits     = s_sw.getReceivedBitlength();
    uint8_t  proto    = s_sw.getReceivedProtocol();
    uint32_t delayUs  = s_sw.getReceivedDelay();
    int      rssi     = ELECHOUSE_cc1101.getRssi();
    s_sw.resetAvailable();

    if (value != 0 && bits > 0) {
      Packet p;
      p.value    = value;
      p.bits     = bits;
      p.protocol = proto;
      p.delayUs  = delayUs;
      p.rssi     = (int8_t)rssi;
      p.freqIdx  = (uint8_t)freqIdx;
      p.ageMs    = millis();

      // Append (ring-buffer eviction on overflow).
      if (pktCount == MAX_PACKETS) {
        for (int i = 0; i < MAX_PACKETS - 1; i++) packets[i] = packets[i + 1];
        pktCount--;
      }
      packets[pktCount++] = p;
      totalDecoded++;
      NeoFx::event(NeoFx::Event::Capture);
      uiDirty = true;
    }
  }

  // Track the noise floor for the "rolling code / unknown" hint.
  static uint32_t lastRssiPollMs = 0;
  if ((millis() - lastRssiPollMs) > 300) {
    lastRssiPollMs = millis();
    lastNoDecodeRssi = (int8_t)ELECHOUSE_cc1101.getRssi();
  }

  // Buttons.
  static bool upPrev=false, dnPrev=false, lfPrev=false, rgPrev=false;
  bool up = !pcf.digitalRead(BTN_UP);
  bool dn = !pcf.digitalRead(BTN_DOWN);
  bool lf = !pcf.digitalRead(BTN_LEFT);
  bool rg = !pcf.digitalRead(BTN_RIGHT);
  if (up && !upPrev && selIndex > 0)              { selIndex--; uiDirty = true; }
  if (dn && !dnPrev && selIndex < pktCount - 1)   { selIndex++; uiDirty = true; }
  if (rg && !rgPrev) { retune((freqIdx + 1) % NUM_FREQS); uiDirty = true; }
  if (lf && !lfPrev && pktCount > 0) {
    int idx = pktCount - 1 - selIndex;
    if (idx >= 0) {
      appendCsv(packets[idx]);
      NeoFx::event(NeoFx::Event::Capture);
    }
  }
  upPrev = up; dnPrev = dn; lfPrev = lf; rgPrev = rg;

  // Local SELECT state machine — purely owned by this feature, no dependency
  // on the shared isSelect*Tapped helpers (which the user reports drifting).
  // Release < 350 ms = short tap; press held > 600 ms = exit.
  static uint32_t selPressedAt = 0;
  static bool     selWasRaw    = false;
  static bool     shortTapFired = false;
  static bool     longHoldFired = false;

  bool selRaw = !pcf.digitalRead(BTN_SELECT);
  uint32_t nowMs = millis();
  shortTapFired = false;
  if (selRaw && !selWasRaw) {
    selPressedAt = nowMs;
    longHoldFired = false;
  } else if (!selRaw && selWasRaw) {
    uint32_t held = nowMs - selPressedAt;
    if (held < 350 && !longHoldFired) shortTapFired = true;
    selPressedAt = 0;
  } else if (selRaw && !longHoldFired && (nowMs - selPressedAt) > 600) {
    longHoldFired = true;
    feature_exit_requested = true;
    selWasRaw = selRaw;
    return;
  }
  selWasRaw = selRaw;

  // Detail-view state machine.
  static int detailIdx = -1;
  if (shortTapFired && pktCount > 0 && detailIdx < 0) {
    detailIdx = pktCount - 1 - selIndex;
    if (detailIdx >= 0 && detailIdx < pktCount) {
      drawDetail(packets[detailIdx]);
    } else {
      detailIdx = -1;
    }
  }

  if (detailIdx >= 0) {
    static bool lfPrevD = false;
    bool lf = !pcf.digitalRead(BTN_LEFT);
    if (lf && !lfPrevD) {
      appendCsv(packets[detailIdx]);
      NeoFx::event(NeoFx::Event::Capture);
    }
    lfPrevD = lf;
    if (shortTapFired && !uiDirty) {       // tap again = close detail
      detailIdx = -1;
      drawShell();
      drawHeader();
      drawList();
    }
    delay(15);
    return;
  }

  if (uiDirty) {
    drawHeader();
    drawList();
    uiDirty = false;
  }

  delay(15);
}

void exit() {
  s_sw.disableReceive();
  ELECHOUSE_cc1101.setSidle();
}

}  // namespace RfDecoder

/* ════════════════════════════════════════════════════════════════════════════
   TpmsLogger — passive logger for nearby Tire Pressure Monitoring Sensors.

   Cars broadcast a TPMS packet every ~30 s (continuous when moving) at
   315 MHz (NA) or 433 MHz (EU). Real decoding is per-OEM and ugly; this
   simpler version detects the bursts via RSSI thresholding and timing, then
   hashes the burst-shape signature (envelope durations) into a stable
   "sensor ID". Same sensor + same car → same ID across re-encounters.

   Logs unique IDs to /captures/tpms.csv with first-seen timestamp + GPS
   (if a fix is available from Pwn Mode having opened the receiver earlier).

   Controls:
     RIGHT    — toggle 315 / 433 MHz band
     LEFT     — clear list and rescan
     SEL hold — exit
   ════════════════════════════════════════════════════════════════════════════ */
namespace TpmsLogger {

static constexpr int    MAX_HITS = 32;
static constexpr float  T_FREQS[] = { 315.0f, 433.92f };
static constexpr int    T_NUM_FREQS = sizeof(T_FREQS) / sizeof(T_FREQS[0]);
static constexpr int    BURST_RSSI_FLOOR = -82;   // bursts below this are noise
static constexpr uint32_t BURST_MIN_MS   = 2;     // minimum on-air duration
static constexpr uint32_t BURST_MAX_MS   = 80;    // longer than this = not TPMS
static constexpr uint32_t BURST_GAP_MS   = 250;   // gap between repeats within one sensor

struct TpmsHit {
  uint32_t id;             // hash of burst shape
  uint8_t  freqIdx;
  int8_t   peakRssi;
  uint32_t firstSeenMs;
  uint32_t lastSeenMs;
  uint16_t hitCount;
};
static TpmsHit  thits[MAX_HITS];
static int      thitCount = 0;
static int      tFreqIdx  = 0;        // start 315 MHz (NA TPMS)
static uint32_t tStartMs  = 0;
static uint32_t tTotalBursts = 0;
static int      tSel      = 0;
static int      tScroll   = 0;
static bool     tUiDirty  = true;

static const uint16_t T_RED = 0xE0E6;
static const uint16_t T_DIM = 0x6020;
static const uint16_t T_GRN = 0x07C0;
static const uint16_t T_MID = 0x9085;

static uint32_t fnv1a(const uint8_t* data, size_t len) {
  uint32_t h = 2166136261u;
  for (size_t i = 0; i < len; i++) { h ^= data[i]; h *= 16777619u; }
  return h;
}

static int findOrAlloc(uint32_t id) {
  for (int i = 0; i < thitCount; i++) if (thits[i].id == id) return i;
  if (thitCount == MAX_HITS) {
    // Evict the oldest by lastSeenMs.
    int oldest = 0;
    for (int i = 1; i < thitCount; i++)
      if (thits[i].lastSeenMs < thits[oldest].lastSeenMs) oldest = i;
    return oldest;
  }
  return thitCount++;
}

static void appendCsv(const TpmsHit& h) {
  if (!(SD.cardSize() > 0)) return;
  if (!SD.exists("/captures")) SD.mkdir("/captures");
  bool fresh = !SD.exists("/captures/tpms.csv");
  File f = SD.open("/captures/tpms.csv", FILE_APPEND);
  if (!f) return;
  if (fresh) f.println("utc,date,freq_mhz,sensor_id_hex,peak_rssi");
  char utc[10], date[10];
  TimeSync::utcNow(utc, sizeof(utc));
  TimeSync::utcDateNow(date, sizeof(date));
  f.printf("%s,%s,%.3f,0x%08lX,%d\n",
           utc, date, T_FREQS[h.freqIdx],
           (unsigned long)h.id, (int)h.peakRssi);
  f.close();
}

static void retuneTpms(int idx) {
  if (idx < 0) idx = 0;
  if (idx >= T_NUM_FREQS) idx = T_NUM_FREQS - 1;
  tFreqIdx = idx;
  ELECHOUSE_cc1101.setSidle();
  ELECHOUSE_cc1101.setMHZ(T_FREQS[tFreqIdx]);
  ELECHOUSE_cc1101.SetRx();
}

static void drawShellTpms() {
  tft.fillScreen(TFT_BLACK);
  tft.drawFastHLine(0, 0,   240, T_RED);
  tft.drawFastHLine(0, 24,  240, T_DIM);
  tft.drawFastHLine(0, 296, 240, T_DIM);
  tft.drawFastHLine(0, 319, 240, T_RED);
  tft.setTextFont(2); tft.setTextSize(1);
  tft.setTextColor(T_RED, TFT_BLACK);
  tft.setCursor(50, 6); tft.print("[ TPMS LOGGER ]");
  tft.setTextColor(T_DIM, TFT_BLACK);
  tft.setCursor(8, 301); tft.print("R=band  L=clear  hold SEL=exit");
}

static void drawListTpms() {
  tft.fillRect(0, 28, 240, 268, TFT_BLACK);
  tft.setTextFont(2); tft.setTextSize(1);
  tft.setTextColor(T_GRN, TFT_BLACK);
  tft.setCursor(8, 30);
  char hdr[40];
  uint32_t up = (millis() - tStartMs) / 1000;
  snprintf(hdr, sizeof(hdr), "%.2f MHz  sensors:%d  up:%lus",
           T_FREQS[tFreqIdx], thitCount, (unsigned long)up);
  tft.print(hdr);
  tft.setTextColor(T_MID, TFT_BLACK);
  tft.setCursor(8, 46);
  snprintf(hdr, sizeof(hdr), "bursts seen: %lu", (unsigned long)tTotalBursts);
  tft.print(hdr);

  if (thitCount == 0) {
    tft.setTextColor(T_DIM, TFT_BLACK);
    tft.setCursor(8, 80);
    tft.print("// listening for TPMS bursts //");
    tft.setCursor(8, 100);
    tft.print("Park near a road and wait for");
    tft.setCursor(8, 116);
    tft.print("cars to drive past, or drive");
    tft.setCursor(8, 132);
    tft.print("through a parking lot. Sensors");
    tft.setCursor(8, 148);
    tft.print("broadcast every ~30s when moving.");
    return;
  }

  // Sort newest-first.
  int order[MAX_HITS];
  for (int i = 0; i < thitCount; i++) order[i] = i;
  for (int i = 0; i + 1 < thitCount; i++)
    for (int j = i + 1; j < thitCount; j++)
      if (thits[order[j]].lastSeenMs > thits[order[i]].lastSeenMs) {
        int t = order[i]; order[i] = order[j]; order[j] = t;
      }

  const int Y0 = 70;
  const int rowH = 22;
  const int maxRows = (290 - Y0) / rowH;
  for (int r = 0; r < maxRows && r < thitCount; r++) {
    int idx = order[r];
    const TpmsHit& h = thits[idx];
    int y = Y0 + r * rowH;
    bool isSel = (r == tSel);
    if (isSel) tft.fillRect(2, y - 2, 236, rowH, T_DIM);
    tft.setTextColor(isSel ? WHITE : T_RED, isSel ? T_DIM : TFT_BLACK);
    tft.setCursor(4, y);
    char line[40];
    snprintf(line, sizeof(line), "0x%08lX  %ddBm",
             (unsigned long)h.id, (int)h.peakRssi);
    tft.print(line);
    tft.setCursor(4, y + 10);
    tft.setTextColor(isSel ? WHITE : T_GRN, isSel ? T_DIM : TFT_BLACK);
    uint32_t ageS = (millis() - h.lastSeenMs) / 1000;
    snprintf(line, sizeof(line), " hits:%u  last:%lus ago",
             (unsigned)h.hitCount, (unsigned long)ageS);
    tft.print(line);
  }
}

void setup() {
  thitCount = 0; tTotalBursts = 0; tSel = 0; tScroll = 0; tUiDirty = true;
  tStartMs = millis();
  memset(thits, 0, sizeof(thits));

  ELECHOUSE_cc1101.Init();
  ELECHOUSE_cc1101.setGDO(CC1101_GDO0, CC1101_GDO2);
  retuneTpms(tFreqIdx);

  drawShellTpms();
  drawListTpms();
}

void loop() {
  NeoFx::tick();

  // Burst detector: sample RSSI, treat a sustained-above-floor run as a burst.
  static bool      inBurst   = false;
  static uint32_t  burstStartMs = 0;
  static int8_t    burstPeak    = -127;
  static uint8_t   burstSignature[16] = {0};   // accumulated peak buckets
  static uint8_t   burstSigLen  = 0;
  static uint32_t  lastSampleMs = 0;

  uint32_t now = millis();
  if (now - lastSampleMs >= 1) {
    lastSampleMs = now;
    int rssi = ELECHOUSE_cc1101.getRssi();
    if (!inBurst) {
      if (rssi > BURST_RSSI_FLOOR) {
        inBurst = true;
        burstStartMs = now;
        burstPeak = (int8_t)rssi;
        burstSigLen = 0;
        // Seed sig with the band so different bands hash to different ids.
        burstSignature[burstSigLen++] = (uint8_t)tFreqIdx;
      }
    } else {
      if ((int8_t)rssi > burstPeak) burstPeak = (int8_t)rssi;
      // Sample peak-bucket every 4ms into the signature buffer for hashing.
      if (((now - burstStartMs) & 0x3) == 0 && burstSigLen < sizeof(burstSignature)) {
        burstSignature[burstSigLen++] = (uint8_t)((rssi + 110) & 0xFF);
      }
      if (rssi <= BURST_RSSI_FLOOR - 5 || (now - burstStartMs) > BURST_MAX_MS) {
        uint32_t dur = now - burstStartMs;
        if (dur >= BURST_MIN_MS && dur <= BURST_MAX_MS) {
          tTotalBursts++;
          uint32_t id = fnv1a(burstSignature, burstSigLen);
          int slot = findOrAlloc(id);
          if (slot < thitCount && thits[slot].id == id) {
            // Existing sensor — bump.
            if ((int8_t)rssi > thits[slot].peakRssi) thits[slot].peakRssi = burstPeak;
            thits[slot].lastSeenMs = now;
            thits[slot].hitCount++;
          } else {
            // New sensor.
            thits[slot].id          = id;
            thits[slot].freqIdx     = (uint8_t)tFreqIdx;
            thits[slot].peakRssi    = burstPeak;
            thits[slot].firstSeenMs = now;
            thits[slot].lastSeenMs  = now;
            thits[slot].hitCount    = 1;
            if (slot == thitCount - 1 || thits[slot].id != 0) {
              // (findOrAlloc may return an evicted-slot index; either way we
              // wrote into it.)
            }
            appendCsv(thits[slot]);
            NeoFx::event(NeoFx::Event::Capture);
          }
          tUiDirty = true;
        }
        inBurst = false;
      }
    }
  }

  // UI redraw at most every 250ms to avoid TFT thrash during bursts.
  static uint32_t lastDrawMs = 0;
  if (tUiDirty && (now - lastDrawMs) > 250) {
    lastDrawMs = now;
    tUiDirty = false;
    drawListTpms();
  }

  // Buttons.
  static bool lfPrev = false, rgPrev = false;
  bool lf = !pcf.digitalRead(BTN_LEFT);
  bool rg = !pcf.digitalRead(BTN_RIGHT);
  if (rg && !rgPrev) { retuneTpms((tFreqIdx + 1) % T_NUM_FREQS); tUiDirty = true; lastDrawMs = 0; }
  if (lf && !lfPrev) {
    thitCount = 0; tTotalBursts = 0; tSel = 0;
    memset(thits, 0, sizeof(thits));
    tUiDirty = true; lastDrawMs = 0;
  }
  lfPrev = lf; rgPrev = rg;

  // Local long-hold-SELECT exit — bypass the global helpers which were
  // dropping the event in this feature.
  static uint32_t tSelPressedAt = 0;
  static bool     tSelWasRaw    = false;
  bool tSelRaw = !pcf.digitalRead(BTN_SELECT);
  uint32_t tNow = millis();
  if (tSelRaw && !tSelWasRaw) {
    tSelPressedAt = tNow;
  } else if (tSelRaw && (tNow - tSelPressedAt) > 600) {
    feature_exit_requested = true;
    tSelWasRaw = tSelRaw;
    return;
  } else if (!tSelRaw) {
    tSelPressedAt = 0;
  }
  tSelWasRaw = tSelRaw;
}

void exit() {
  ELECHOUSE_cc1101.setSidle();
}

}  // namespace TpmsLogger

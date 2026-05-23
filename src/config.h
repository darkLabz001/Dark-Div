#pragma once
// config.h — merged from bleconfig.h, wificonfig.h, subconfig.h

/* ───────────── Common includes ───────────── */
#include <Arduino.h>
#include <TFT_eSPI.h>
#include <PCF8574.h>
#include <XPT2046_Touchscreen.h>
#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Preferences.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <RCSwitch.h>
#include <SD.h>
#include <Update.h>
#include <ESPmDNS.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_wifi_types.h>
#include <esp_system.h>
#include <esp_event.h>
#include <esp_event_loop.h>
#include <nvs_flash.h>

// Use NimBLE for all BLE functionality, via the BleCompat shim which
// exposes BLEDevice/BLEScan/etc as aliases to NimBLE types.
#include "BleCompat.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"

#include "arduinoFFT.h"
#include "ELECHOUSE_CC1101_SRC_DRV.h"
#include "utils.h"
#include "shared.h"

using namespace std;

/* ───────────── Common externs ───────────── */
extern TFT_eSPI tft;
extern PCF8574 pcf;

/* ───────────── BLE namespaces ───────────── */
namespace BleJammer {
  void blejamSetup();
  void blejamLoop();
  void exit();
}
namespace BleSpoofer {
  void spooferSetup();
  void spooferLoop();
  void exit();
}
namespace SourApple {
  void sourappleSetup();
  void sourappleLoop();
  void exit();
}
namespace BleScan {
  void bleScanSetup();
  void bleScanLoop();
  void exit();
  // Background BLE scanner: runs periodically in its own task, caching results.
  void startBackgroundScanner();
  // Last known BLE device count from background/foreground scans.
  int  getLastCount();
}
namespace Scanner {
  void scannerSetup();
  void scannerLoop();
}
namespace ProtoKill {
  void prokillSetup();
  void prokillLoop();
}
namespace BleSniffer {
  void blesnifferSetup();
  void blesnifferLoop();
  void exit();
}

/* ───────────── SubGHz namespaces ───────────── */
namespace replayat {
  void ReplayAttackSetup();
  void ReplayAttackLoop();
}
namespace SavedProfile {
  void saveSetup();
  void saveLoop();
}
namespace subjammer {
  void subjammerSetup();
  void subjammerLoop();
}

/* ───────────── WiFi namespaces ───────────── */
namespace PacketMonitor {
  void ptmSetup();
  void ptmLoop();
}
namespace BeaconSpammer {
  void beaconSpamSetup();
  void beaconSpamLoop();
}
namespace DeauthDetect {
  void deauthdetectSetup();
  void deauthdetectLoop();
}
namespace WifiScan {
  void wifiscanSetup();
  void wifiscanLoop();
  // Background WiFi scanner: runs when no feature is active, caching results.
  void startBackgroundScanner();
  // Last known WiFi network count from background/foreground scans.
  int  getLastCount();
}
namespace CaptivePortal {
  void cportalSetup();
  void cportalLoop();
}
namespace Deauther {
  void deautherSetup();
  void deautherLoop();
}
namespace ProbeRequestFlood {
  void probeRequestFloodSetup();
  void probeRequestFloodLoop();
}
namespace FirmwareUpdate {
  void updateSetup();
  void updateLoop();
}
namespace HandshakeCapture {
  void hscapSetup();
  void hscapLoop();
  void hscapExit();
}
namespace OtaGithub {
  // Run the full "fetch latest release from github.com/darkLabz001/Dark-Div
  // and flash it" flow. Blocks until done (success -> reboot, failure ->
  // returns after showing an error). Prompts for WiFi credentials via the
  // on-screen keyboard the first time it's invoked; persists them in NVS.
  void run();
}
namespace WifiSetup {
  // Interactive WiFi scan + connect flow. Lists nearby SSIDs, lets the user
  // pick one (UP/DOWN to navigate, RIGHT or touch to select), prompts for
  // the password via the on-screen keyboard, stores credentials in NVS, and
  // attempts a connection. Credentials are shared with OtaGithub::run().
  void run();
}
namespace WebDashboard {
  // Small HTTP server on port 80 serving a single-page web UI plus JSON API
  // endpoints (status, pwn state, captures browser, SD browser, downloads).
  // Reuses the WiFi creds saved by WifiSetup / OtaGithub.
  void setup();
  void loop();
  void exit();
}
namespace PwnMode {
  // Autonomous channel-hopping handshake hunter. Reuses HandshakeCapture's
  // EAPOL/PMKID parser but on every BSSID, not a single target, and walks
  // channels 1-13 on a 5 s dwell. Periodic broadcast deauths provoke fresh
  // handshakes. Saves to /captures/handshakes.22000 just like the targeted
  // capture mode. RIGHT = manual deauth on current channel, SELECT = exit.
  void pwnSetup();
  void pwnLoop();
  void pwnExit();
}

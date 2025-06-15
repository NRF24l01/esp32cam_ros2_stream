#include "config.h"
#include <FS.h>
#include <SPIFFS.h>

CamConfig camConfig;

static const char* CONFIG_FILE = "/camconfig.txt";

// Парсинг строки "ssid|password|host|port"
bool CamConfig::parseAndSet(const String& line, String& err) {
  int p1 = line.indexOf('|');
  int p2 = line.indexOf('|', p1+1);
  int p3 = line.indexOf('|', p2+1);
  if (p1 < 0 || p2 < 0 || p3 < 0) { err = "Invalid format"; return false; }
  String s = line.substring(0, p1);
  String p = line.substring(p1+1, p2);
  String h = line.substring(p2+1, p3);
  String portStr = line.substring(p3+1);
  uint16_t port = portStr.toInt();
  if (!port) { err = "Port invalid"; return false; }
  ssid = s; password = p; udp_host = h; udp_port = port;
  return true;
}

void CamConfig::load() {
  if (!SPIFFS.begin(true)) return;
  File f = SPIFFS.open(CONFIG_FILE, "r");
  if (!f) return;
  String l = f.readStringUntil('\n');
  String dummy;
  parseAndSet(l, dummy);
  f.close();
}

void CamConfig::save() {
  if (!SPIFFS.begin(true)) return;
  File f = SPIFFS.open(CONFIG_FILE, "w");
  if (!f) return;
  f.println(ssid + "|" + password + "|" + udp_host + "|" + String(udp_port));
  f.close();
}

String CamConfig::toString() const {
  return ssid + "|" + password + "|" + udp_host + "|" + String(udp_port);
}

String CamConfig::toJSON() const {
  return "{\"ssid\":\"" + ssid + "\",\"password\":\"" + password + "\",\"udp_host\":\"" + udp_host +
    "\",\"udp_port\":" + String(udp_port) + "}";
}
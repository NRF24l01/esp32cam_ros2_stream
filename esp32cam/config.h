#pragma once
#include <Arduino.h>
struct CamConfig {
  String ssid;
  String password;

  void load();
  void save();
  bool parseAndSet(const String& line, String& err);
  String toString() const;
  String toJSON() const;
};
extern CamConfig camConfig;
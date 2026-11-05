#pragma once
#include "Arduino.h"
class Preferences {
public:
  bool begin(const char* ns, bool readOnly = false) { (void)ns; (void)readOnly; return true; }
  float getFloat(const char* key, float def = 0.0f) { (void)key; return def; }
  void  putFloat(const char* key, float v) { (void)key; (void)v; }
  void  end() {}
  void  clear() {}
};
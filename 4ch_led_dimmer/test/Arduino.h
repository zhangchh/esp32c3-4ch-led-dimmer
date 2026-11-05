#pragma once
// ---- Minimal Arduino mock for host-side compile/behaviour testing ----
#include <cstdint>
#include <cstdio>
#include <cstdarg>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <string>

#define F(x) (x)
#define HIGH 1
#define LOW  0
#define INPUT        0
#define OUTPUT       1
#define INPUT_PULLUP 2

enum adc_attenuation_t { ADC_0db = 0, ADC_2_5db, ADC_6db, ADC_11db };

// ---- mock state (defined in mock_impl.cpp) ----
extern float    g_mockMv[32];        // simulated wiper voltage per pin
extern uint32_t g_mockDuty[32];      // last duty written per pin
extern int      g_mockDutyWrites;    // write counter
extern unsigned long g_mockMillis;
extern int      g_mockBootPressed;
extern std::string g_mockSerialIn;

class String {
public:
  std::string s;
  String() {}
  String(const char* p) : s(p) {}
  String(int v) : s(std::to_string(v)) {}
  String& operator+=(const String& o) { s += o.s; return *this; }
  friend String operator+(const String& a, const String& b) { String r(a); r.s += b.s; return r; }
  operator const char*() const { return s.c_str(); }
};

class Print {
public:
  void begin(unsigned long) {}
  void print(const char* s) { fputs(s, stdout); }
  void print(char c) { fputc(c, stdout); }
  void print(int v) { printf("%d", v); }
  void print(unsigned int v) { printf("%u", v); }
  void print(long v) { printf("%ld", v); }
  void print(unsigned long v) { printf("%lu", v); }
  void print(float v) { printf("%f", (double)v); }
  void print(double v) { printf("%f", v); }
  void println() { putchar('\n'); }
  void println(const char* s) { fputs(s, stdout); putchar('\n'); }
  void println(int v) { printf("%d\n", v); }
  void println(unsigned int v) { printf("%u\n", v); }
  void println(unsigned long v) { printf("%lu\n", v); }
  void println(float v) { printf("%f\n", (double)v); }
  void println(double v) { printf("%f\n", v); }
  int printf(const char* fmt, ...) {
    va_list ap; va_start(ap, fmt);
    int r = vprintf(fmt, ap);
    va_end(ap);
    return r;
  }
  int available() { return (int)g_mockSerialIn.size(); }
  int read() { if (g_mockSerialIn.empty()) return -1; int c = (unsigned char)g_mockSerialIn[0]; g_mockSerialIn.erase(0,1); return c; }
};

extern Print Serial;
extern Print USBSerial;

// ---- GPIO / ADC / LEDC ----
unsigned long millis();
void delay(unsigned long ms);
void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t value);
int  digitalRead(uint8_t pin);
void analogReadResolution(int bits);
void analogSetAttenuation(adc_attenuation_t att);
int  analogRead(uint8_t pin);
uint32_t analogReadMilliVolts(uint8_t pin);

// LEDC: 3.x API (pin based) and 2.x API (channel based) share this signature
bool ledcAttach(uint8_t pin, uint32_t freq, uint8_t resolution);
void ledcSetup(uint8_t channel, uint32_t freq, uint8_t resolution);
void ledcAttachPin(uint8_t pin, uint8_t channel);
void ledcWrite(uint8_t pin_or_channel, uint32_t duty);
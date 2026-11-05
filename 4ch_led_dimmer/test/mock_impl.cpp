#include "Arduino.h"

Print Serial;
Print USBSerial;

float    g_mockMv[32] = {0};
uint32_t g_mockDuty[32] = {0};
int      g_mockDutyWrites = 0;
unsigned long g_mockMillis = 0;
int      g_mockBootPressed = 0;
std::string g_mockSerialIn;

static int g_mockPinMode[32] = {0};
static uint8_t g_mockChannelPin[16] = {0};
static bool g_mockUsedPinApi = false;

unsigned long millis() { return ++g_mockMillis; }
void delay(unsigned long ms) { g_mockMillis += (ms ? ms : 1); }
void pinMode(uint8_t pin, uint8_t mode) { if (pin < 32) g_mockPinMode[pin] = mode; }
void digitalWrite(uint8_t pin, uint8_t value) { (void)pin; (void)value; }
int  digitalRead(uint8_t pin) { (void)pin; return g_mockBootPressed ? LOW : HIGH; }
void analogReadResolution(int bits) { (void)bits; }
void analogSetAttenuation(adc_attenuation_t att) { (void)att; }

int analogRead(uint8_t pin) {
  float mv = (pin < 32) ? g_mockMv[pin] : 0.0f;
  if (mv > 2500.0f) mv = 2500.0f;
  if (mv < 0.0f) mv = 0.0f;
  return (int)(mv / 2500.0f * 4095.0f + 0.5f);
}

uint32_t analogReadMilliVolts(uint8_t pin) {
  float mv = (pin < 32) ? g_mockMv[pin] : 0.0f;
  if (mv > 2500.0f) mv = 2500.0f;   // 11dB saturation
  if (mv < 0.0f) mv = 0.0f;
  return (uint32_t)(mv + 0.5f);
}

bool ledcAttach(uint8_t pin, uint32_t freq, uint8_t res) { (void)freq; (void)res; g_mockUsedPinApi = true; return true; }
void ledcSetup(uint8_t ch, uint32_t freq, uint8_t res) { (void)ch; (void)freq; (void)res; }
void ledcAttachPin(uint8_t pin, uint8_t ch) { if (ch < 16) g_mockChannelPin[ch] = pin; }
void ledcWrite(uint8_t pin_or_ch, uint32_t duty) {
  uint8_t target = g_mockUsedPinApi ? pin_or_ch : (pin_or_ch < 16 ? g_mockChannelPin[pin_or_ch] : pin_or_ch);
  if (target < 32) g_mockDuty[target] = duty;
  g_mockDutyWrites++;
}
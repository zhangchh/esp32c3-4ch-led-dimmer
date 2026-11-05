/* =====================================================================
 *  4ch_led_dimmer.ino
 *  4 路 LED 灯带 PWM 调光 —— ESP32-C3 SuperMini
 *  ---------------------------------------------------------------------
 *  硬件依据：docs/4-channel-led-pwm-wiring-v3.html
 *
 *  每一路的信号链：
 *    10kΩ 电位器（3V3 — GND）wiper ──(100nF 对地)──> GPIO0 / 1 / 3 / 4  (ADC1)
 *    GPIO21 / 7 / 6 / 5 ──> UCC27524 INA(pin2) / INB(pin4)，非反相
 *                        ──> UCC27524 OUTA(pin7) / OUTB(pin5)
 *                        ──(47Ω)──> IRLZ44N 栅极（另有 100kΩ 下拉到地）
 *    IRLZ44N 源极 ──> GND，漏极 ──> 灯带负极；灯带正极 ──> +12V
 *
 *  UCC27524 是【非反相】栅极驱动器，所以占空比直接等于亮度：
 *      0%   → MOSFET 关断 → 灯带熄灭
 *      100% → MOSFET 全导通 → 灯带最亮
 *  固件在此基础上叠加：软件 gamma 校正、EMA 平滑、斜率限制、两点标定。
 *
 *  目标框架：Arduino-ESP32 core >= 2.0.6（3.x 已适配）
 * ===================================================================== */

#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <Preferences.h>

#if defined(ARDUINO_ARCH_ESP32) && __has_include("core_version.h")
#include "core_version.h"
#endif

/* ---------- Arduino-ESP32 版本适配 ---------- */
// 3.x 的 LEDC API 以“引脚”为参数；2.x 以“通道”为参数
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
#define LEDC_USES_PIN 1
#endif

// 串口端口选择。
// ESP32-C3 的 GPIO21 既是 CH1 的 PWM 脚，也是 UART0 的 TX —— 二者只能二选一。
// 核心在 ARDUINO_USB_CDC_ON_BOOT=1 时会把 Serial 重定向到原生 USB
//   (USB_MODE=1 → HWCDCSerial，即 C3 板子上那个 USB-Serial-JTAG)
// 因此必须开启 "USB CDC On Boot"，否则 Serial = Serial0 = UART0，会和 CH1 抢 GPIO21。
#if defined(ARDUINO_USB_CDC_ON_BOOT) && (ARDUINO_USB_CDC_ON_BOOT == 1)
#define LEDCTL_PORT Serial          // 已是原生 USB，放心用
#else
#define LEDCTL_PORT Serial0         // UART0：与 GPIO21 冲突，见下方 #warning
#endif

#if defined(ARDUINO_ARCH_ESP32) && (!defined(ARDUINO_USB_CDC_ON_BOOT) || (ARDUINO_USB_CDC_ON_BOOT == 0))
#warning "USB CDC On Boot is DISABLED: Serial falls back to UART0 whose TX is GPIO21 = CH1 PWM. Enable 'USB CDC On Boot' in Tools."
#endif

/* ========================= 1. 引脚与常量 ========================= */

static const uint8_t kChCount = 4;

// 电位器 wiper → ESP32-C3 ADC1_CH0 / CH1 / CH3 / CH4
static const int kPotPin[kChCount] = { 0, 1, 3, 4 };

// ESP32-C3 PWM → UCC27524 输入（pin2 = A 路，pin4 = B 路）
static const int kPwmPin[kChCount] = { 21, 7, 6, 5 };

// PWM 参数：恒频 4 kHz / 12 bit（4096 级）
static const uint32_t kPwmFreqHz     = 4000;
static const uint8_t  kPwmResolution = 12;
static const uint32_t kPwmDutyMax    = (1u << kPwmResolution) - 1;

// ADC 参数：12 bit，11dB 衰减（有效量程 0 ~ 2.5 V）
static const int   kAdcBits   = 12;
static const float kAdcSatMv  = 2500.0f;   // 11dB 衰减下的线性上限
static const float kAdcSatEps = 15.0f;     // 饱和判定余量 (mV)

// 控制参数
static const uint8_t kLoopMs       = 8;        // 主循环周期 ≈125 Hz
static const float   kAdcEmaAlpha  = 0.25f;    // ADC 一阶低通系数
static const float   kDutyEmaAlpha = 0.35f;    // 亮度平滑系数（抑制抖动/闪烁）
static const float   kGammaDefault = 2.20f;    // 线性电位器 → 人眼线性
static const float   kMinOnDuty    = 0.02f;    // 低于该占空比完全熄灭
static const uint8_t kBootBtnPin   = 9;        // ESP32-C3 BOOT 按钮（低电平按下）
static const char    kNvsNamespace[] = "led4ch";

/* ========================= 2. 状态量 ========================= */

struct Channel {
  float    minMv;        // 标定：旋钮最小位置时的 wiper 电压
  float    fullMv;       // 标定：认为 100% 亮度时的 wiper 电压
  float    mvEma;        // 平滑后的 wiper 电压
  float    level;        // 归一化旋钮位置 0..1
  float    duty;         // 实际占空比 0..1
  bool     emaReady;
  uint8_t  ledcChannel;
  uint32_t lastDuty;
};

static Channel     g_ch[kChCount];
static Preferences g_prefs;
static float       g_gamma    = kGammaDefault;
static bool        g_forceOff = false;
static char        g_line[64];
static uint8_t     g_lineLen  = 0;

/* ========================= 3. 前置声明 ========================= */

static void printHelp();
static void printStatus();
static void printCalibration();

/* ========================= 4. 底层读写 ========================= */

static float readWiperMv(int pin) {
  return (float)analogReadMilliVolts(pin);
}

static void attachPwm(uint8_t ch) {
#if defined(LEDC_USES_PIN)
  if (!ledcAttach(kPwmPin[ch], kPwmFreqHz, kPwmResolution)) {
    LEDCTL_PORT.printf("错误：IO%d 无法绑定到 LEDC，请换一个可作 PWM 的引脚。\r\n", kPwmPin[ch]);
  }
#else
  g_ch[ch].ledcChannel = ch;                 // 2.x：0..3 号 LEDC 通道
  ledcSetup(g_ch[ch].ledcChannel, kPwmFreqHz, kPwmResolution);
  ledcAttachPin(kPwmPin[ch], g_ch[ch].ledcChannel);
#endif
}

static void writeDuty(uint8_t ch, float duty) {
  if (duty < 0.0f) duty = 0.0f;
  if (duty > 1.0f) duty = 1.0f;

  const uint32_t raw = (uint32_t)lroundf(duty * (float)kPwmDutyMax);
  if (raw == g_ch[ch].lastDuty) return;       // 没变化就不写，省一点总线开销
  g_ch[ch].lastDuty = raw;

#if defined(LEDC_USES_PIN)
  ledcWrite(kPwmPin[ch], raw);
#else
  ledcWrite(g_ch[ch].ledcChannel, raw);
#endif
}

/* ========================= 5. 标定的存取 ========================= */

static void loadCalibration() {
  char key[12];
  for (uint8_t i = 0; i < kChCount; ++i) {
    snprintf(key, sizeof(key), "cmin%u", (unsigned)i);
    g_ch[i].minMv  = g_prefs.getFloat(key, 0.0f);
    snprintf(key, sizeof(key), "cfull%u", (unsigned)i);
    g_ch[i].fullMv = g_prefs.getFloat(key, kAdcSatMv);
    if (g_ch[i].fullMv <= g_ch[i].minMv + 50.0f) {   // 标定值不合理 → 回退默认
      g_ch[i].minMv  = 0.0f;
      g_ch[i].fullMv = kAdcSatMv;
    }
  }
}

static void saveCalibration() {
  char key[12];
  for (uint8_t i = 0; i < kChCount; ++i) {
    snprintf(key, sizeof(key), "cmin%u", (unsigned)i);
    g_prefs.putFloat(key, g_ch[i].minMv);
    snprintf(key, sizeof(key), "cfull%u", (unsigned)i);
    g_prefs.putFloat(key, g_ch[i].fullMv);
  }
}

static void resetCalibration() {
  for (uint8_t i = 0; i < kChCount; ++i) {
    g_ch[i].minMv  = 0.0f;
    g_ch[i].fullMv = kAdcSatMv;
  }
  saveCalibration();
}

/* ========================= 6. 主控制循环 ========================= */

static void updateChannels() {
  for (uint8_t i = 0; i < kChCount; ++i) {
    const float mv = readWiperMv(kPotPin[i]);

    // ADC 一阶低通，抑制电位器接触噪声
    if (!g_ch[i].emaReady) {
      g_ch[i].mvEma    = mv;
      g_ch[i].emaReady = true;
    } else {
      g_ch[i].mvEma += (mv - g_ch[i].mvEma) * kAdcEmaAlpha;
    }

    // 标定区间内线性映射到 0..1
    const float span = (g_ch[i].fullMv - g_ch[i].minMv > 1.0f)
                       ? (g_ch[i].fullMv - g_ch[i].minMv) : kAdcSatMv;
    float level = (g_ch[i].mvEma - g_ch[i].minMv) / span;
    if (level < 0.0f) level = 0.0f;
    if (level > 1.0f) level = 1.0f;
    g_ch[i].level = level;

    // gamma 校正：线性(B 型)电位器 → 人眼感知线性
    float target = 0.0f;
    if (!g_forceOff) {
      target = powf(level, g_gamma);
      if (target < kMinOnDuty) target = 0.0f;
    }

    // 斜率限制：旋钮快速转动 / ADC 抖动时，亮度平滑跟随
    g_ch[i].duty += (target - g_ch[i].duty) * kDutyEmaAlpha;
    if (g_ch[i].duty < kMinOnDuty) g_ch[i].duty = 0.0f;
    if (g_ch[i].duty > 1.0f)      g_ch[i].duty = 1.0f;

    writeDuty(i, g_ch[i].duty);
  }
}

/* ========================= 7. 标定采样 ========================= */

static void sortFloats(float *a, uint8_t n) {
  for (uint8_t i = 1; i < n; ++i) {
    const float v = a[i];
    uint8_t j = i;
    while (j > 0 && a[j - 1] > v) { a[j] = a[j - 1]; --j; }
    a[j] = v;
  }
}

// 采样约 2.4 秒，取分位数（最小取 5%，最大取 95%），对噪声鲁棒
static void captureExtremes(bool wantMax, float outMv[kChCount]) {
  static const uint8_t kSamples = 120;
  static float buf[kChCount][kSamples];
  static float sorted[kSamples];

  for (uint8_t i = 0; i < kChCount; ++i) {
    for (uint8_t s = 0; s < kSamples; ++s) {
      buf[i][s] = readWiperMv(kPotPin[i]);
      delay(5);
    }
  }

  for (uint8_t i = 0; i < kChCount; ++i) {
    uint8_t n = 0;
    for (uint8_t s = 0; s < kSamples; ++s) {
      // 求最大值时剔除完全饱和的样本（ADC 在 2.5 V 以上读数封顶）
      if (wantMax && buf[i][s] >= kAdcSatMv - kAdcSatEps) continue;
      sorted[n++] = buf[i][s];
    }

    if (n == 0) {                      // 全程饱和：用饱和电压兜底
      float mx = 0.0f;
      for (uint8_t s = 0; s < kSamples; ++s) if (buf[i][s] > mx) mx = buf[i][s];
      outMv[i] = mx;
      continue;
    }

    sortFloats(sorted, n);
    if (wantMax) outMv[i] = sorted[(uint8_t)((n * 19) / 20)];   // 95 分位
    else         outMv[i] = sorted[(uint8_t)(n / 20)];            // 5 分位
  }
}

static bool waitEnter(uint32_t timeoutMs = 60000) {
  LEDCTL_PORT.println(F("(等待回车，60 秒超时)"));
  const uint32_t t0 = millis();
  while ((uint32_t)(millis() - t0) < timeoutMs) {
    if (LEDCTL_PORT.available()) {
      const int c = LEDCTL_PORT.read();
      if (c == '\n' || c == '\r') return true;
    }
    delay(10);
  }
  return false;
}

static void runCalWizard() {
  float v[kChCount];

  LEDCTL_PORT.println();
  LEDCTL_PORT.println(F("========== 标定向导 =========="));
  LEDCTL_PORT.println(F("步骤 1/2：把 4 个旋钮全部转到【最小】位置，然后按回车"));
  if (!waitEnter()) { LEDCTL_PORT.println(F("标定已取消。")); return; }
  captureExtremes(false, v);
  for (uint8_t i = 0; i < kChCount; ++i) g_ch[i].minMv = v[i];

  LEDCTL_PORT.println(F("步骤 2/2：把 4 个旋钮全部转到【最大】位置，然后按回车"));
  if (!waitEnter()) { LEDCTL_PORT.println(F("标定已取消。")); return; }
  captureExtremes(true, v);
  for (uint8_t i = 0; i < kChCount; ++i) g_ch[i].fullMv = v[i];

  bool ok = true;
  for (uint8_t i = 0; i < kChCount; ++i) {
    if (g_ch[i].fullMv <= g_ch[i].minMv + 50.0f) {
      LEDCTL_PORT.printf("警告：CH%u 标定区间过小，已回退默认值。\r\n", (unsigned)(i + 1));
      g_ch[i].minMv  = 0.0f;
      g_ch[i].fullMv = kAdcSatMv;
      ok = false;
    }
  }
  if (!ok) { saveCalibration(); return; }

  saveCalibration();
  printCalibration();
  LEDCTL_PORT.println(F("标定完成，已写入 NVS。"));
  LEDCTL_PORT.println(F("提示：ADC 在约 2.5 V 饱和，3.3 V 电位器顶部约 1/4 行程是死区（见 README）。"));
}

static void captureOne(bool wantMax) {
  float v[kChCount];
  captureExtremes(wantMax, v);
  for (uint8_t i = 0; i < kChCount; ++i) {
    if (wantMax) g_ch[i].fullMv = v[i];
    else         g_ch[i].minMv  = v[i];
    if (g_ch[i].fullMv <= g_ch[i].minMv + 50.0f) {
      g_ch[i].minMv  = 0.0f;
      g_ch[i].fullMv = kAdcSatMv;
    }
  }
  saveCalibration();
  printCalibration();
}

/* ========================= 8. 串口命令行 ========================= */

static void printHelp() {
  LEDCTL_PORT.println(F("命令："));
  LEDCTL_PORT.println(F("  status      显示各路电压 / 亮度 / 占空比"));
  LEDCTL_PORT.println(F("  cal         两步标定向导（最小 -> 最大）"));
  LEDCTL_PORT.println(F("  cal min     只采集最小值"));
  LEDCTL_PORT.println(F("  cal max     只采集最大值"));
  LEDCTL_PORT.println(F("  cal show    显示当前标定"));
  LEDCTL_PORT.println(F("  cal reset   恢复默认标定"));
  LEDCTL_PORT.println(F("  off / on    强制全灭 / 恢复旋钮控制"));
  LEDCTL_PORT.println(F("  gamma <x>   临时设置 gamma 指数（如 gamma 2.2）"));
}

static void printStatus() {
  LEDCTL_PORT.println(F("通道  电位器   PWM    wiper电压   旋钮位置   占空比"));
  for (uint8_t i = 0; i < kChCount; ++i) {
    LEDCTL_PORT.printf(" CH%u   IO%-3d   IO%-3d  %8.1f mV   %6.3f    %6.2f%%\r\n",
                       (unsigned)(i + 1), kPotPin[i], kPwmPin[i],
                       g_ch[i].mvEma, g_ch[i].level, g_ch[i].duty * 100.0f);
  }
  if (g_forceOff) LEDCTL_PORT.println(F("[当前处于强制关闭状态，输入 on 恢复]"));
}

static void printCalibration() {
  LEDCTL_PORT.println(F("通道   min(mV)   full(mV)   区间(mV)"));
  for (uint8_t i = 0; i < kChCount; ++i) {
    LEDCTL_PORT.printf(" CH%u   %8.1f   %9.1f   %9.1f\r\n",
                       (unsigned)(i + 1), g_ch[i].minMv, g_ch[i].fullMv,
                       g_ch[i].fullMv - g_ch[i].minMv);
  }
}

static void handleCommand(const char *cmd) {
  if (strcmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0) {
    printHelp();
  } else if (strcmp(cmd, "status") == 0) {
    printStatus();
  } else if (strcmp(cmd, "cal") == 0) {
    runCalWizard();
  } else if (strcmp(cmd, "cal reset") == 0) {
    resetCalibration();
    LEDCTL_PORT.println(F("已恢复默认标定。"));
    printCalibration();
  } else if (strcmp(cmd, "cal show") == 0) {
    printCalibration();
  } else if (strcmp(cmd, "cal min") == 0) {
    captureOne(false);
  } else if (strcmp(cmd, "cal max") == 0) {
    captureOne(true);
  } else if (strcmp(cmd, "off") == 0) {
    g_forceOff = true;
    LEDCTL_PORT.println(F("已强制关闭全部输出（旋钮不再起作用）。"));
  } else if (strcmp(cmd, "on") == 0) {
    g_forceOff = false;
    LEDCTL_PORT.println(F("已恢复由旋钮控制。"));
  } else if (strncmp(cmd, "gamma ", 6) == 0) {
    const float gm = (float)atof(cmd + 6);
    if (gm >= 1.0f && gm <= 5.0f) {
      g_gamma = gm;
      LEDCTL_PORT.printf("gamma = %.2f （仅本次运行有效，重启后恢复默认）\r\n", g_gamma);
    } else {
      LEDCTL_PORT.println(F("gamma 取值范围 1.0 ~ 5.0"));
    }
  } else {
    LEDCTL_PORT.println(F("未知命令，输入 help 查看用法"));
  }
}

static void serviceSerial() {
  while (LEDCTL_PORT.available()) {
    const int c = LEDCTL_PORT.read();
    if (c == '\r' || c == '\n') {
      if (g_lineLen > 0) {
        g_line[g_lineLen] = '\0';
        handleCommand(g_line);
        g_lineLen = 0;
      }
    } else if (g_lineLen < sizeof(g_line) - 1) {
      g_line[g_lineLen++] = (char)c;
    }
  }
}

/* ========================= 9. setup / loop ========================= */

void setup() {
  LEDCTL_PORT.begin(115200);

  g_prefs.begin(kNvsNamespace, false);
  loadCalibration();

  analogReadResolution(kAdcBits);
  analogSetAttenuation(ADC_11db);          // 0 ~ 2.5 V 有效量程

  for (uint8_t i = 0; i < kChCount; ++i) {
    // 先当普通输出拉低，避免 ESP32 复位期间 UCC27524 的输入悬空而误开 MOSFET
    pinMode(kPwmPin[i], OUTPUT);
    digitalWrite(kPwmPin[i], LOW);
    g_ch[i].lastDuty = 0xFFFFFFFFu;         // 保证首次写入 0
    attachPwm(i);
    writeDuty(i, 0.0f);
  }

  // 按住 BOOT 上电 → 直接进入标定向导
  pinMode(kBootBtnPin, INPUT_PULLUP);
  delay(50);
  const bool bootHeld = (digitalRead(kBootBtnPin) == LOW);

  LEDCTL_PORT.println();
  LEDCTL_PORT.println(F("========== 4ch_led_dimmer ready =========="));
  LEDCTL_PORT.printf("PWM: %lu Hz / %u bit   gamma: %.2f\r\n",
                     (unsigned long)kPwmFreqHz, (unsigned)kPwmResolution, g_gamma);
  printHelp();

  if (bootHeld) {
    LEDCTL_PORT.println(F("[检测到 BOOT 按住] 进入标定向导"));
    runCalWizard();
  }
}

void loop() {
  serviceSerial();
  updateChannels();
  delay(kLoopMs);
}

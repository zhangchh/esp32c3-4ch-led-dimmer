// Host-side behavioural test harness for 4ch_led_dimmer.ino
#include "Arduino.h"
#include <cstdio>
#include <cmath>
#include <cstring>
#include <string>

extern void setup();
extern void loop();

static int g_pass = 0, g_fail = 0;

static void check(const char* name, bool cond, const char* detail = "") {
  if (cond) { g_pass++; printf("  [PASS] %s\n", name); }
  else      { g_fail++; printf("  [FAIL] %s   %s\n", name, detail); }
}

static void settle(int n = 150) { for (int i = 0; i < n; ++i) loop(); }

static void setPots(float a, float b, float c, float d) {
  g_mockMv[0] = a; g_mockMv[1] = b; g_mockMv[3] = c; g_mockMv[4] = d;
}

// expected raw duty for a given wiper voltage with default calibration (0..2500 mV)
static uint32_t expectedDuty(float mv) {
  double level = mv / 2500.0;
  if (level < 0) level = 0;
  if (level > 1) level = 1;
  return (uint32_t)lround(pow(level, 2.2) * 4095.0);
}

int main() {
  setup();

  printf("\n=== T1  全部旋钮归零 -> 四路输出必须为 0 ===\n");
  setPots(0, 0, 0, 0);
  settle();
  check("CH1(IO21)=0", g_mockDuty[21] == 0);
  check("CH2(IO7) =0", g_mockDuty[7]  == 0);
  check("CH3(IO6) =0", g_mockDuty[6]  == 0);
  check("CH4(IO5) =0", g_mockDuty[5]  == 0);

  printf("\n=== T2  全部旋钮拉满 (2500 mV) -> 四路输出必须为 4095 ===\n");
  setPots(2500, 2500, 2500, 2500);
  settle();
  check("CH1=4095", g_mockDuty[21] == 4095, "actual");
  check("CH2=4095", g_mockDuty[7]  == 4095);
  check("CH3=4095", g_mockDuty[6]  == 4095);
  check("CH4=4095", g_mockDuty[5]  == 4095);

  printf("\n=== T3  半程 (1250 mV) -> gamma 2.2 期望 %u ===\n", expectedDuty(1250));
  setPots(1250, 1250, 1250, 1250);
  settle();
  {
    uint32_t e = expectedDuty(1250);
    check("CH1 gamma 曲线正确", abs((int)g_mockDuty[21] - (int)e) <= 2);
    check("CH2 gamma 曲线正确", abs((int)g_mockDuty[7]  - (int)e) <= 2);
    check("CH3 gamma 曲线正确", abs((int)g_mockDuty[6]  - (int)e) <= 2);
    check("CH4 gamma 曲线正确", abs((int)g_mockDuty[5]  - (int)e) <= 2);
  }

  printf("\n=== T4  引脚映射: 电位器 IO0/1/3/4 -> PWM IO21/7/6/5 ===\n");
  setPots(2500, 0, 0, 0); settle();
  check("IO0 -> IO21", g_mockDuty[21] == 4095 && g_mockDuty[7] == 0 && g_mockDuty[6] == 0 && g_mockDuty[5] == 0);
  setPots(0, 2500, 0, 0); settle();
  check("IO1 -> IO7",  g_mockDuty[7]  == 4095 && g_mockDuty[21] == 0 && g_mockDuty[6] == 0 && g_mockDuty[5] == 0);
  setPots(0, 0, 2500, 0); settle();
  check("IO3 -> IO6",  g_mockDuty[6]  == 4095 && g_mockDuty[21] == 0 && g_mockDuty[7]  == 0 && g_mockDuty[5] == 0);
  setPots(0, 0, 0, 2500); settle();
  check("IO4 -> IO5",  g_mockDuty[5]  == 4095 && g_mockDuty[21] == 0 && g_mockDuty[7]  == 0 && g_mockDuty[6] == 0);

  printf("\n=== T5  四路互相独立 ===\n");
  setPots(2500, 0, 1250, 0); settle();
  check("CH1 满 / CH2 灭 / CH3 半 / CH4 灭",
        g_mockDuty[21] == 4095 && g_mockDuty[7] == 0 &&
        abs((int)g_mockDuty[6] - (int)expectedDuty(1250)) <= 2 && g_mockDuty[5] == 0);

  printf("\n=== T6  ADC 饱和: wiper 超过 2.5 V 仍应为 100%% (不超调) ===\n");
  setPots(3000, 3000, 3000, 3000); settle();
  check("3.0 V -> 4095 且不溢出", g_mockDuty[21] == 4095 && g_mockDuty[5] == 4095);

  printf("\n=== T7  亮度单调性 (0 -> 100%% 共 21 个点) ===\n");
  {
    bool mono = true;
    uint32_t prev = 0;
    for (int i = 0; i <= 20; ++i) {
      float mv = (float)i * 2500.0f / 20.0f;
      setPots(mv, mv, mv, mv);
      settle();
      uint32_t d = g_mockDuty[21];
      if (d < prev) { mono = false; printf("      非单调 @%.0f mV: %u < %u\n", mv, d, prev); }
      prev = d;
    }
    check("占空比随旋钮单调递增", mono);
  }

  printf("\n=== T8  串口命令 off / on ===\n");
  setPots(2500, 2500, 2500, 2500); settle();
  g_mockSerialIn = "off\n"; loop();
  settle();
  check("off 后全部为 0", g_mockDuty[21] == 0 && g_mockDuty[7] == 0 && g_mockDuty[6] == 0 && g_mockDuty[5] == 0);
  g_mockSerialIn = "on\n"; loop();
  settle();
  check("on 后恢复 4095", g_mockDuty[21] == 4095 && g_mockDuty[7] == 4095);

  printf("\n=== T9  标定命令 (cal min / cal max) ===\n");
  setPots(50, 50, 50, 50);
  g_mockSerialIn = "cal min\n"; loop();     // 采集最小值 ~50 mV
  setPots(2500, 2500, 2500, 2500);
  g_mockSerialIn = "cal max\n"; loop();     // 采集最大值
  setPots(1250, 1250, 1250, 1250);
  settle();
  {
    // 标定后区间变成 50..2500 => level = (1250-50)/(2500-50) = 0.4898
    double level = (1250.0 - 50.0) / (2500.0 - 50.0);
    uint32_t e = (uint32_t)lround(pow(level, 2.2) * 4095.0);
    printf("      标定后期望占空比 = %u\n", e);
    check("标定后映射重新归一化", abs((int)g_mockDuty[21] - (int)e) <= 3);
  }
  setPots(50, 50, 50, 50); settle();
  check("标定后最小点仍为 0", g_mockDuty[21] == 0);

  printf("\n=== T10  status / help / 未知命令 不崩溃 ===\n");
  g_mockSerialIn = "status\n"; loop();
  g_mockSerialIn = "help\n";   loop();
  g_mockSerialIn = "garbage\n"; loop();
  g_mockSerialIn = "gamma 1.0\n"; loop();
  settle();
  check("命令处理后仍在运行", true);

  printf("\n----------------------------------------\n");
  printf("  通过 %d 项，失败 %d 项\n", g_pass, g_fail);
  printf("----------------------------------------\n");
  return g_fail == 0 ? 0 : 1;
}
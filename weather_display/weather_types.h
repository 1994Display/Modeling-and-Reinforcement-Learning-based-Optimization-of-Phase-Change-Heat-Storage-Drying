/**
 * weather_types.h
 * 自定义类型定义（独立头文件，避免 Arduino 自动函数原型在类型定义前
 * 引用这些类型导致 "does not name a type" 编译错误）
 */
#ifndef WEATHER_TYPES_H
#define WEATHER_TYPES_H

#include <stdint.h>

// 天气类别
enum WeatherCat { CAT_UNKNOWN, CAT_SUNNY, CAT_CLOUDY, CAT_OVERCAST,
                  CAT_RAIN, CAT_SNOW, CAT_STORM, CAT_FOG };

// 天气数据（vvhan API 解析结果）
struct WeatherData {
  bool   valid = false;
  char   wea_utf8[24] = {0};   // 天气描述 UTF-8
  char   win_utf8[16] = {0};   // 风向 UTF-8
  int    tem = 0;              // 当前温度
  int    hum = 0;              // 湿度 %
  int    tmax = 0;
  int    tmin = 0;
  int    win_level = 0;        // 风级（0=微风）
  bool   has_tomorrow = false;
  char   tmr_wea_utf8[24] = {0};
  int    tmr_tmax = 0;
  int    tmr_tmin = 0;
  int    week = 0;             // 0~6 (星期X -> 0~6)
  unsigned long fetch_ms = 0;

  // 小时级降水预报（Open-Meteo，逐小时降水概率）
  bool   rain_ok = false;       // 小时预报是否获取成功
  int    rain_start_hour = -1;  // 从当前时刻起首个降水小时 0~23；-1 = 当前之后无雨
  int    rain_end_hour   = -1;  // 该段连续降雨的结束小时（含）；-1 = 持续到深夜
  int    rain_start_prob = 0;   // 降雨开始小时对应的降水概率 %
  bool   rain_any_today = false;// 今天全天是否有雨
  char   realtime_wea[16] = {0};// 实时天气（Open-Meteo 当前小时 weather_code 映射）

  // 未来天气变化预测（Open-Meteo 逐小时 weather_code）
  int    fc_change_hour = -1;    // 距当前几小时后天气变化；-1=未来无变化
  char   fc_change_wea[16] = {0};// 变化后的天气（如"多云"）

  // 明天天气变化预测（Open-Meteo 明天 0~23 时）
  int    tmr_fc_hour = -1;       // 明天几点天气变化；-1=明天无变化
  char   tmr_fc_wea[16] = {0};   // 明天变化后的天气（如"多云"）
  char   tmr_wea_start[16] = {0};// 明天起始天气（0 时）
};

// 干燥建议（文字 + 颜色）
struct Advice { const uint8_t* text; uint16_t color; };

// AI 干燥建议（esp32_controller 返回的建议码 -> 文字 + 颜色）
struct AiAdvice { const uint8_t* text; uint16_t color; };

#endif // WEATHER_TYPES_H
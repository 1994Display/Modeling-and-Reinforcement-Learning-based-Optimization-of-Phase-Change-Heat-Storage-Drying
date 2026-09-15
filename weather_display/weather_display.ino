/**
 * ============================================================================
 *  太阳能相变粮食干燥 - 联网气象监测系统
 *  硬件：ESP32-S3-N16R8 + 2.8寸 SPI TFT (ILI9341, 240x320) 带 GT24L24A2W16 字库
 *  平台：Arduino IDE + ESP32-S3 Dev Module（零第三方库依赖）
 *
 *  功能：
 *    1. WiFi 联网 -> 获取实时天气与预报 (默认免 Key 的 vvhan API)
 *    2. 在 TFT 上显示城市/天气/气温/湿度/风速/风向/最高最低温/明日预报/时间
 *    3. 根据气象数据给出太阳能粮食干燥作业建议
 *    4. 通过字库芯片读取 GBK 点阵显示中文（16x16 汉字 + 8x16 / 16x32 ASCII）
 *
 *  模块引脚（ESP32-S3 -> 2.8寸带字库TFT，对照模块原理图）：
 *    GPIO 11 (MOSI) -> 模块 SDI
 *    GPIO 13 (MISO) -> 模块 SDO  (字库数据输出，无字库版本可不接)
 *    GPIO 12 (SCK)  -> 模块 SCL
 *    GPIO 10 (CS)   -> 模块 CS   (低电平=液晶，高电平=字库，复用同一脚)
 *    GPIO  9 (DC)   -> 模块 RS/DC
 *    GPIO 14 (RST)  -> 模块 RST
 *    GPIO 21 (BLK)  -> 模块 BLK  (背光；勿用 GPIO45/46，见注释)
 *    GPIO  0 (BOOT) -> 手动按键：按下立即刷新天气（可选）
 *
 *  Arduino IDE 库：
 *    - 仅需 ESP32 内置的 WiFi / HTTPClient / SPI / Time（零第三方依赖）
 *    - JSON 解析使用内置极简提取器，无需安装 ArduinoJson
 * ============================================================================
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include <SPI.h>
#include <Wire.h>
#include <time.h>
// 注意：本程序不再依赖 ArduinoJson 库，内置极简 JSON 提取器（见 fetchWeather）
#include "weather_types.h"   // 自定义类型（必须置于顶部，供 Arduino 自动原型使用）

// 前向声明（drawLightDischarging 定义在文件后部，被 recvAdviceFromController / drawLightLabels 前向调用）
void drawLightDischarging();

// ---- 无线光照接收（来自 ESP32-C3 + GY-30 节点，最多 4 路） ----
#define UDP_PORT       8266    // 与发送端一致
#define LIGHT_TIMEOUT  10000   // 超过 10 秒收不到数据视为离线 (ms)
#define LIGHT_NODES    4       // 支持 4 个 C3 节点（序号 1~4）
WiFiUDP udpRx;
bool   udpReady      = false;  // UDP 仅在 WiFi 连接后初始化（必须在 WiFi 之后 begin！）
bool   lightOnline[LIGHT_NODES] = {false};
float  wirelessLux[LIGHT_NODES] = {-1.0, -1.0, -1.0, -1.0};
unsigned long lastLightMs[LIGHT_NODES] = {0, 0, 0, 0};

// ---- UART 有线发送（四个集热器光照 → esp32_controller） ----
// 接线：本板 GPIO19(TX) → 对方 GPIO19(RX)，本板 GPIO20(RX) ← 对方 GPIO20(TX)，GND 共地
// 注意：本板 GPIO19/20 为普通空闲引脚；日志走 GPIO43/44(CH343)，无需改 USB CDC 设置
#define UART_LINK_BAUD   115200
#define UART_TX_PIN      19
#define UART_RX_PIN      20
// 每 2 秒发送一次四个集热器光照，格式: LUX:1234;2345;3456;4567\n
#define UART_SEND_INTERVAL_MS 2000
unsigned long lastUartSendMs = 0;

// ---- UART 接收：esp32_controller 发回的 AI 干燥建议 ----
// 协议：每行 ADV:建议码\n（建议码 0~N，见 adviceFromCode）
String uartRecvBuf = "";         // 行缓冲
bool   aiAdviceValid = false;    // 是否已收到建议
uint8_t aiAdviceCode = 0;        // 当前建议码
unsigned long lastAdviceMs = 0;  // 建议更新时间（用于跟随天气刷新的展示逻辑）
int    g_discharging = -1;       // 当前放热集热器编号 1~4（-1=未知，来自控制器 DIS 消息）
float  g_remainSec = -1.0f;      // 干燥剩余时间 s（来自控制器 RMN，-1=未知）
unsigned long g_remainMs = 0;    // 收到 RMN 的时间戳（判断是否过期）
unsigned long g_controllerMs = 0;  // 最近收到控制器任何消息的时间戳（在线判定）
int    ctrlFan    = 0;   // 控制器风机实际功率 %（来自 STT 上报）
int    ctrlPump   = 0;   // 控制器水泵实际流量 %（来自 STT 上报）
int    ctrlHeater = 0;   // 控制器加热状态 0/1（来自 STT 上报）
int    ctrlDamper = 0;   // 控制器风门角度 °（来自 STT 上报）
int    ctrlStage  = -1;  // COLLECT 当前阶段码（来自 STA 上报；-1=非 COLLECT）
int    ctrlMode   = -1;  // 系统模式 0=COLLECT 1=AUTO 2=MANUAL（来自 SYS 上报）
float  ctrlLux    = -1.0f; // 控制器 BH1750 实时光照 lx（来自 LUX 上报；-1=无效）
int    ctrlFault  = -1;    // 控制器设备故障 0=正常 1=故障（来自 FLT 上报；-1=未知）
float  g_drySec   = -1.0f; // 本次干燥耗时 s（来自 DRT 上报；-1=未知）

// ---- Web 服务器（平板远程监控，零第三方库，独立不影响其他功能） ----
WebServer webServer(80);
bool      webReady = false;      // Web 服务器初始化成功标志（失败则完全不影响其他功能）

// ---- 联网光照（Open-Meteo 短波辐射，随天气获取，1 分钟刷新显示） ----
float  localLux      = -1.0;   // 当前光照强度 (lx)
float  luxNextH      = -1.0;   // 下一小时光照 (lx，用于趋势)
float  luxPeakH      = -1.0;   // 今日光照峰值 (lx)
int    luxPeakHour   = -1;     // 峰值所在小时
unsigned long lastLocalLuxMs = 0;

/* =========================================================================
 *                          1. 用户配置区
 * ========================================================================= */

// ---- WiFi ----
const char* WIFI_SSID     = "cdwwsbz";
const char* WIFI_PASSWORD = "blwrd233";

// ---- 固定 IP（自适应网段）----
// 连接成功后把 IP 最后一段固定为该值，前 3 段跟随热点实际网段。
// 这样网址固定且热点网段变化时不会断网。设为 0 = 关闭固定 IP，走纯 DHCP。
// 注意：200 不要落在热点 DHCP 常用分配段内，避免与其他设备冲突。
#define FIX_IP_LAST   200

// ---- 自带热点（平板直连，网址固定 http://192.168.4.1，不依赖手机热点）----
// 注意：ESP32 热点密码必须 ≥8 位，否则自动变成开放热点（无密码）。
#define AP_SSID      "GymDisplay"
#define AP_PASSWORD  "12211221"

// ---- 自动定位（IP 定位到手机当前所在城市） ----
// 1 = 开启：联网后自动查询公共 IP 归属地，匹配到城市表后显示该城市天气
// 0 = 关闭：始终使用 CITY_INDEX 指定城市
#define AUTO_LOCATE  1

// ---- 高德地图 IP 定位（Web服务 API key，免费） ----
// 在 https://lbs.amap.com/ 注册后，控制台 -> 应用管理 -> 创建 Web服务 key
// 高德 IP 定位只能精确到"市级"，返回 rectangle 经纬度范围；
// 用其中心点做逆地理编码可尽力到区县（移动 IP 精度有限，仅供参考）。
const char* AMAP_KEY = "fdc9c4f8c9633dcf4781dcb348becbdf";
// 城市 -> 固定区县：定位到该城市时强制使用该区县的天气（见 CITY_DISTRICT_PINS 表）

// 天气 API（itboy，纯 HTTP、免 Key）：http://t.weather.itboy.net/api/weather/city/<城市代码>
const char* WEATHER_HOST = "http://t.weather.itboy.net";
const char* WEATHER_PATH = "/api/weather/city/";

// IP 定位说明：
//   主源 myip.ipip.net（纯文本），备用 cip.cc（纯文本），见 locateByText()
// 默认城市：通过 CITY_INDEX 选取（0=北京，1=上海，2=广州 …）
// code 为"中国天气网城市代码"，用于 itboy 天气接口
// 如需新增城市，按下面格式补一行；GBK 用于字库显示
struct CityEntry { const char* name; const char* code;
                   uint8_t gbk[12]; uint8_t gbk_len;   // 去 const：允许默认构造 & 拷贝
                   float lat, lon; };   // 经纬度：用于 Open-Meteo 逐小时降水预报
const CityEntry CITIES[] = {
  {"北京", "101010100", {0xB1,0xB1,0xBE,0xA9,0x00}, 5, 39.904f, 116.407f},
  {"上海", "101020100", {0xC9,0xCF,0xBA,0xA3,0x00}, 5, 31.230f, 121.474f},
  {"广州", "101280101", {0xB9,0xE3,0xD6,0xDD,0x00}, 5, 23.129f, 113.264f},
  {"深圳", "101280601", {0xC9,0xEE,0xDB,0xDA,0x00}, 5, 22.543f, 114.058f},
  {"杭州", "101210101", {0xBA,0xBC,0xD6,0xDD,0x00}, 5, 30.274f, 120.155f},
  {"南京", "101190101", {0xC4,0xCF,0xBE,0xA9,0x00}, 5, 32.060f, 118.797f},
  {"武汉", "101200101", {0xCE,0xE4,0xBA,0xBA,0x00}, 5, 30.593f, 114.305f},
  {"成都", "101270101", {0xB3,0xC9,0xB6,0xBC,0x00}, 5, 30.573f, 104.067f},
  {"重庆", "101040100", {0xD6,0xD8,0xC7,0xEC,0x00}, 5, 29.563f, 106.551f},
  {"西安", "101110101", {0xCE,0xF7,0xB0,0xB2,0x00}, 5, 34.341f, 108.940f},
  {"天津", "101030100", {0xCC,0xEC,0xBD,0xF2,0x00}, 5, 39.085f, 117.201f},
  {"苏州", "101190401", {0xCB,0xD5,0xD6,0xDD,0x00}, 5, 31.299f, 120.585f},
  {"长沙", "101250101", {0xB3,0xA4,0xC9,0xB3,0x00}, 5, 28.228f, 112.939f},
  {"青岛", "101120201", {0xC7,0xE0,0xB5,0xBA,0x00}, 5, 36.067f, 120.383f},
  {"厦门", "101230201", {0xCF,0xC3,0xC3,0xC5,0x00}, 5, 24.480f, 118.089f},
  {"福州", "101230101", {0xB8,0xA3,0xD6,0xDD,0x00}, 5, 26.074f, 119.297f},
  {"济南", "101120101", {0xBC,0xC3,0xC4,0xCF,0x00}, 5, 36.651f, 117.120f},
  {"合肥", "101220101", {0xBA,0xCF,0xB7,0xCA,0x00}, 5, 31.821f, 117.227f},
  {"郑州", "101180101", {0xD6,0xA3,0xD6,0xDD,0x00}, 5, 34.747f, 113.625f},
  {"昆明", "101290101", {0xC0,0xA5,0xC3,0xF7,0x00}, 5, 24.880f, 102.833f},
  {"哈尔滨","101050101", {0xB9,0xFE,0xB6,0xFB,0xB1,0xF5,0x00}, 7, 45.803f, 126.535f},
  {"长春", "101060101", {0xB3,0xA4,0xB4,0xBA,0x00}, 5, 43.817f, 125.324f},
  {"沈阳", "101070101", {0xC9,0xF2,0xD1,0xF4,0x00}, 5, 41.806f, 123.432f},
  {"大连", "101070201", {0xB4,0xF3,0xC1,0xAC,0x00}, 5, 38.914f, 121.615f},
  {"南宁", "101300101", {0xC4,0xCF,0xC4,0xFE,0x00}, 5, 22.817f, 108.366f},
  {"海口", "101310101", {0xBA,0xA3,0xBF,0xDA,0x00}, 5, 20.044f, 110.199f},
  {"三亚", "101310201", {0xC8,0xFD,0xD1,0xC7,0x00}, 5, 18.253f, 109.512f},
  {"兰州", "101160101", {0xC0,0xBC,0xD6,0xDD,0x00}, 5, 36.061f, 103.834f},
  {"贵阳", "101260101", {0xB9,0xF3,0xD1,0xF4,0x00}, 5, 26.647f, 106.630f},
  {"太原", "101100101", {0xCC,0xAB,0xD4,0xAD,0x00}, 5, 37.870f, 112.549f},
  {"呼和浩特","101080101", {0xBA,0xF4,0xBA,0xCD,0xBA,0xC6,0xCC,0xD8,0x00}, 9, 40.842f, 111.750f},
  {"石家庄", "101090101", {0xCA,0xAF,0xBC,0xD2,0xD7,0xAF,0x00}, 7, 38.043f, 114.515f},
  {"乌鲁木齐","101130101", {0xCE,0xDA,0xC2,0xB3,0xC4,0xBE,0xC6,0xEB,0x00}, 9, 43.826f, 87.617f},
  {"拉萨", "101140101", {0xC0,0xAD,0xC8,0xF8,0x00}, 5, 29.650f, 91.100f},
  {"西宁", "101150101", {0xCE,0xF7,0xC4,0xFE,0x00}, 5, 36.617f, 101.778f},
  {"银川", "101170101", {0xD2,0xF8,0xB4,0xA8,0x00}, 5, 38.487f, 106.231f},
  {"南昌", "101240101", {0xC4,0xCF,0xB2,0xFD,0x00}, 5, 28.682f, 115.858f},
  {"佛山", "101280800", {0xB7,0xF0,0xC9,0xBD,0x00}, 5, 23.022f, 113.122f},
  {"香港", "101320101", {0xCF,0xE3,0xB8,0xDB,0x00}, 5, 22.320f, 114.175f},
  {"澳门", "101330101", {0xB0,0xC4,0xC3,0xC5,0x00}, 5, 22.199f, 113.549f},
  {"台北", "101340101", {0xCC,0xA8,0xB1,0xB1,0x00}, 5, 25.033f, 121.565f},
  {"保定", "101090201", {0xB1,0xA3,0xB6,0xA8,0x00}, 5, 38.874f, 115.465f},
  {"唐山", "101090501", {0xCC,0xC6,0xC9,0xBD,0x00}, 5, 39.630f, 118.180f},
  {"邯郸", "101091001", {0xBA,0xAA,0xB5,0xA6,0x00}, 5, 36.626f, 114.539f},
  {"廊坊", "101090601", {0xC0,0xC8,0xB7,0xBB,0x00}, 5, 39.538f, 116.684f},
  {"沧州", "101090701", {0xB2,0xD7,0xD6,0xDD,0x00}, 5, 38.304f, 116.839f},
  {"邢台", "101090901", {0xD0,0xCF,0xCC,0xA8,0x00}, 5, 37.070f, 114.504f},
  {"秦皇岛", "101091101", {0xC7,0xD8,0xBB,0xCA,0xB5,0xBA,0x00}, 7, 39.942f, 119.600f},
  {"承德", "101090402", {0xB3,0xD0,0xB5,0xC2,0x00}, 5, 40.951f, 117.963f},
  {"张家口", "101090301", {0xD5,0xC5,0xBC,0xD2,0xBF,0xDA,0x00}, 7, 40.825f, 114.886f},
};
const uint8_t CITY_INDEX = 42;  // 默认城市：42=唐山（唐山工业职业技术大学所在地，IP 定位失败时的兜底）
const CityEntry* city = &CITIES[CITY_INDEX];

// ---- 区县表（高德 regeo 匹配到区县后，用区县天气代码） ----
// 唐山 11 区县 + 成都 14 区县（itboy 区县代码已实测/核对）
struct DistrictEntry {
  const char* name;          // 区县名（UTF-8，用于匹配高德 regeo district）
  const char* code;          // itboy 区县天气代码
  const uint8_t gbk[12];     // GBK 字模
  uint8_t gbk_len;
  float lat, lon;            // 经纬度（Open-Meteo 降水用）
};
const DistrictEntry DISTRICTS[] = {
  // ---- 唐山市（11 区县） ----
  {"丰南区", "101090502", {0xB7,0xE1,0xC4,0xCF,0xC7,0xF8,0x00}, 7, 39.33f, 118.10f},
  {"丰润区", "101090503", {0xB7,0xE1,0xC8,0xF3,0xC7,0xF8,0x00}, 7, 39.83f, 118.16f},
  {"滦州市", "101090504", {0xC2,0xD0,0xD6,0xDD,0xCA,0xD0,0x00}, 7, 39.74f, 118.70f},
  {"滦南县", "101090505", {0xC2,0xD0,0xC4,0xCF,0xCF,0xD8,0x00}, 7, 39.50f, 118.68f},
  {"乐亭县", "101090506", {0xC0,0xD6,0xCD,0xA4,0xCF,0xD8,0x00}, 7, 39.42f, 118.91f},
  {"迁西县", "101090507", {0xC7,0xA8,0xCE,0xF7,0xCF,0xD8,0x00}, 7, 40.14f, 118.31f},
  {"玉田县", "101090508", {0xD3,0xF1,0xCC,0xEF,0xCF,0xD8,0x00}, 7, 39.90f, 117.74f},
  {"曹妃甸区","101090509", {0xB2,0xDC,0xE5,0xFA,0xB5,0xE9,0x00}, 7, 39.27f, 118.46f},   // 屏幕显示"曹妃甸"（不带区）
  {"遵化市", "101090510", {0xD7,0xF1,0xBB,0xAF,0xCA,0xD0,0x00}, 7, 40.19f, 117.97f},
  {"迁安市", "101090511", {0xC7,0xA8,0xB0,0xB2,0xCA,0xD0,0x00}, 7, 40.00f, 118.70f},
  // 唐山主城区（无独立天气代码，共用唐山市 101090501）
  {"路南区", "101090501", {0xC2,0xB7,0xC4,0xCF,0xC7,0xF8,0x00}, 7, 39.63f, 118.15f},
  {"路北区", "101090501", {0xC2,0xB7,0xB1,0xB1,0xC7,0xF8,0x00}, 7, 39.65f, 118.20f},
  {"古冶区", "101090501", {0xB9,0xC5,0xD2,0xB1,0xC7,0xF8,0x00}, 7, 39.73f, 118.44f},
  {"开平区", "101090501", {0xBF,0xAA,0xC6,0xBD,0xC7,0xF8,0x00}, 7, 39.67f, 118.26f},
  // ---- 成都市（14 区县） ----
  {"龙泉驿区","101270102", {0xC1,0xFA,0xC8,0xAA,0xE6,0xE4,0xC7,0xF8,0x00}, 9, 30.56f, 104.27f},
  {"新都区", "101270103", {0xD0,0xC2,0xB6,0xBC,0xC7,0xF8,0x00}, 7, 30.82f, 104.16f},
  {"温江区", "101270104", {0xCE,0xC2,0xBD,0xAD,0xC7,0xF8,0x00}, 7, 30.68f, 103.85f},
  {"金堂县", "101270105", {0xBD,0xF0,0xCC,0xC3,0xCF,0xD8,0x00}, 7, 30.86f, 104.41f},
  {"双流区", "101270106", {0xCB,0xAB,0xC1,0xF7,0xC7,0xF8,0x00}, 7, 30.57f, 103.92f},
  {"郫都区", "101270107", {0xDB,0xAF,0xB6,0xBC,0xC7,0xF8,0x00}, 7, 30.80f, 103.90f},
  {"大邑县", "101270108", {0xB4,0xF3,0xD2,0xD8,0xCF,0xD8,0x00}, 7, 30.59f, 103.51f},
  {"蒲江县", "101270109", {0xC6,0xD1,0xBD,0xAD,0xCF,0xD8,0x00}, 7, 30.19f, 103.51f},
  {"新津区", "101270110", {0xD0,0xC2,0xBD,0xF2,0xC7,0xF8,0x00}, 7, 30.41f, 103.81f},
  {"都江堰市","101270111", {0xB6,0xBC,0xBD,0xAD,0xD1,0xDF,0xCA,0xD0,0x00}, 9, 31.00f, 103.65f},
  {"彭州市", "101270112", {0xC5,0xED,0xD6,0xDD,0xCA,0xD0,0x00}, 7, 30.99f, 103.94f},
  {"邛崃市", "101270113", {0xDA,0xF6,0xE1,0xC1,0xCA,0xD0,0x00}, 7, 30.41f, 103.46f},
  {"崇州市", "101270114", {0xB3,0xE7,0xD6,0xDD,0xCA,0xD0,0x00}, 7, 30.63f, 103.67f},
  {"武侯区", "101270119", {0xCE,0xE4,0xBA,0xEE,0xC7,0xF8,0x00}, 7, 30.64f, 104.04f},
  // 成都主城区（无独立天气代码，共用成都市 101270101）
  {"青羊区", "101270101", {0xC7,0xE0,0xD1,0xF2,0xC7,0xF8,0x00}, 7, 30.66f, 104.06f},
  {"锦江区", "101270101", {0xBD,0xF5,0xBD,0xAD,0xC7,0xF8,0x00}, 7, 30.66f, 104.09f},
  {"金牛区", "101270101", {0xBD,0xF0,0xC5,0xA3,0xC7,0xF8,0x00}, 7, 30.69f, 104.05f},
  {"成华区", "101270101", {0xB3,0xC9,0xBB,0xAA,0xC7,0xF8,0x00}, 7, 30.66f, 104.10f},
  {"青白江区","101270101", {0xC7,0xE0,0xB0,0xD7,0xBD,0xAD,0xC7,0xF8,0x00}, 9, 30.88f, 104.24f},
  {"简阳市", "101270101", {0xBC,0xF2,0xD1,0xF4,0xCA,0xD0,0x00}, 7, 30.41f, 104.55f},
};
// 自动定位结果（AUTO_LOCATE=1 时使用）——必须先于 setDistrictCity 声明
const CityEntry* g_activeCity = nullptr;   // 定位并匹配到的城市（区县定位时指向 g_districtCity）
const CityEntry* g_dispCity   = nullptr;   // 当前实际显示天气的城市

const DistrictEntry* g_activeDistrict = nullptr;  // 定位/手动指定的区县
CityEntry g_districtCity;                          // 区县对应的"虚拟城市"（供天气获取/显示复用）

// ---- 城市 -> 固定区县映射（定位到该城市时，强制显示该区县的天气） ----
// 例如：定位到"唐山"则显示曹妃甸区天气；定位到"成都"则显示郫都区天气
struct CityDistrictPin { const char* city; const char* district; };
const CityDistrictPin CITY_DISTRICT_PINS[] = {
  {"唐山", "曹妃甸区"},
  {"成都", "郫都区"},
};
#define PIN_COUNT  (sizeof(CITY_DISTRICT_PINS)/sizeof(CITY_DISTRICT_PINS[0]))

// 把某个区县填入"虚拟城市"，供天气获取/显示复用
static void setDistrictCity(const DistrictEntry* d) {
  g_districtCity.name   = d->name;
  g_districtCity.code   = d->code;
  memcpy(g_districtCity.gbk, d->gbk, 12);
  g_districtCity.gbk_len = d->gbk_len;
  g_districtCity.lat    = d->lat;
  g_districtCity.lon    = d->lon;
  g_activeDistrict = d;
  g_activeCity     = &g_districtCity;
}

// 查"城市 -> 固定区县"映射：返回区县表项；未配置返回 nullptr
static const DistrictEntry* findPinnedDistrict(const char* cityName) {
  for (size_t p = 0; p < PIN_COUNT; p++) {
    if (strcmp(CITY_DISTRICT_PINS[p].city, cityName) == 0) {
      for (size_t i = 0; i < sizeof(DISTRICTS)/sizeof(DISTRICTS[0]); i++) {
        if (strcmp(CITY_DISTRICT_PINS[p].district, DISTRICTS[i].name) == 0) return &DISTRICTS[i];
      }
    }
  }
  return nullptr;
}

// 反查：根据区县名找所属城市表项（用于显示"城市+区县"）；未配置返回 nullptr
static const CityEntry* findCityOfDistrict(const char* districtName) {
  for (size_t p = 0; p < PIN_COUNT; p++) {
    if (strcmp(CITY_DISTRICT_PINS[p].district, districtName) == 0) {
      for (size_t i = 0; i < sizeof(CITIES)/sizeof(CITIES[0]); i++) {
        if (strcmp(CITY_DISTRICT_PINS[p].city, CITIES[i].name) == 0) return &CITIES[i];
      }
    }
  }
  return nullptr;
}

// ---- NTP ----
const long  UTC_OFFSET_SEC = 8 * 3600;          // UTC+8
const char* NTP_SERVER1    = "cn.pool.ntp.org";
const char* NTP_SERVER2    = "ntp1.aliyun.com";

// ---- 刷新周期 ----
const unsigned long WEATHER_INTERVAL_MS = 60 * 60 * 1000UL;   // 1 小时
const unsigned long TIME_REFRESH_MS     = 1000;               // 1 秒
const unsigned long LUX_REFRESH_MS      = 60 * 1000UL;        // 当地光照 1 分钟

// ---- GPIO ----
#define PIN_MOSI   11
#define PIN_MISO   13
#define PIN_SCK    12
#define PIN_CS     10
#define PIN_DC      9
#define PIN_RST    14
#define PIN_BLK    21   // 背光（注意：不要用 GPIO45/46！它们是 VDD_SPI strap 引脚，会阻止上传）
#define PIN_BOOT    0    // BOOT 按键：按下立即刷新

/* =========================================================================
 *                2. 字库芯片 GT24L24A2W16 偏移量
 * ========================================================================= */
#define FONT_CMD_READ         0x03
#define ASCII_8x16_ST        0x00080800UL
#define ASCII_16x32_ST       0x00082600UL
#define GBK_16x16_ST         0x0011DD00UL
#define GBK_24x24_ST         0x001DA000UL   // 24x24 汉字区（每字 72 字节）

/* =========================================================================
 *                3. 中文字符串（GBK 字节数组，由 _gbk_gen.py 生成）
 * ========================================================================= */

// ---- 标题与状态 ----
const uint8_t TXT_TITLE[]      = {0xC6,0xF8,0xCF,0xF3,0xBC,0xE0,0xB2,0xE2,0xCF,0xB5,0xCD,0xB3,0x00}; // 气象监测系统
const uint8_t TXT_LOADING[]    = {0xC1,0xAA,0xCD,0xF8,0xBB,0xF1,0xC8,0xA1,0xD6,0xD0,0x00};             // 联网获取中
const uint8_t TXT_CONN_FAIL[]  = {0xC1,0xAC,0xBD,0xD3,0xCA,0xA7,0xB0,0xDC,0x00};                         // 连接失败
const uint8_t TXT_FETCH_FAIL[] = {0xBB,0xF1,0xC8,0xA1,0xCA,0xA7,0xB0,0xDC,0x00};                         // 获取失败
const uint8_t TXT_CHECK_WIFI[] = {0xC7,0xEB,0xBC,0xEC,0xB2,0xE9,0x57,0x69,0x46,0x69,0x00};             // 请检查WiFi
const uint8_t TXT_RETRY[]      = {0xC9,0xD4,0xBA,0xF3,0xD6,0xD8,0xCA,0xD4,0x00};                         // 稍后重试

// ---- 字段标签 ----
const uint8_t TXT_L_CITY[]  = {0xB3,0xC7,0xCA,0xD0,0x00};   // 城市
const uint8_t TXT_L_LOCATED[]={0xB6,0xA8,0xCE,0xBB,0x00};   // 定位
const uint8_t TXT_L_NODE[]  = {0xBC,0xAF,0xC8,0xC8,0xC6,0xF7,0x00};   // 集热器
const uint8_t TXT_HEADER_COLLECTOR[] = {0xBC,0xAF,0xC8,0xC8,0xC6,0xF7,0xC4,0xDA,0xB2,0xBF,0xB9,0xE2,0xD5,0xD5,0x00}; // 集热器内部光照
const uint8_t TXT_L_LUX[]   = {0xB9,0xE2,0xD5,0xD5,0x00};   // 光照
const uint8_t TXT_L_DISCH[] = {0xBA,0xC5,0xD5,0xFD,0xD4,0xDA,0xB7,0xC5,0xC8,0xC8,0xA3,0xA1,0x00}; // 号正在放热！
const uint8_t TXT_UP[]      = {0xC9,0xFD,0x00};             // 升
const uint8_t TXT_FLAT[]    = {0xC6,0xBD,0x00};             // 平
const uint8_t TXT_DOWN[]    = {0xBD,0xB5,0x00};             // 降
const uint8_t TXT_PEAK[]    = {0xB7,0xE5,0x00};             // 峰
const uint8_t TXT_NEXT[]    = {0xCF,0xC2,0x00};             // 下（下一时）
const uint8_t TXT_AFTER[]   = {0xBA,0xF3,0x00};             // 后
const uint8_t TXT_TOW[]     = {0xD7,0xAA,0x00};             // 转
const uint8_t TXT_KEEP[]    = {0xB3,0xD6,0xD0,0xF8,0x00};   // 持续
const uint8_t TXT_TODAY[]   = {0xBD,0xF1,0xC8,0xD5,0x00};   // 今日
const uint8_t TXT_DEGREE[]  = {0xB6,0xC8,0x00};             // 度（备用）
const uint8_t TXT_DEG_SYM[] = {0xA1,0xE3,0x00};             // °（GBK 全角度符号）
const uint8_t TXT_L_TEM[]   = {0xC6,0xF8,0xCE,0xC2,0x00};   // 气温
const uint8_t TXT_L_HUM[]   = {0xCA,0xAA,0xB6,0xC8,0x00};   // 湿度
const uint8_t TXT_L_WIND[]  = {0xB7,0xE7,0xCF,0xF2,0x00};   // 风向
const uint8_t TXT_L_WSPD[]  = {0xB7,0xE7,0xCB,0xD9,0x00};   // 风速
const uint8_t TXT_L_HIGH[]  = {0xD7,0xEE,0xB8,0xDF,0x00};   // 最高
const uint8_t TXT_L_LOW[]   = {0xD7,0xEE,0xB5,0xCD,0x00};   // 最低
const uint8_t TXT_L_TMR[]   = {0xC3,0xF7,0xCC,0xEC,0x00};   // 明天
const uint8_t TXT_MING[]    = {0xC3,0xF7,0x00};             // 明
const uint8_t TXT_DIAN[]    = {0xB5,0xE3,0x00};             // 点
const uint8_t TXT_L_TODAY[] = {0xBD,0xF1,0xCC,0xEC,0x00};   // 今天
const uint8_t TXT_L_TIME[]  = {0xCA,0xB1,0xBC,0xE4,0x00};   // 时间
const uint8_t TXT_L_WEEK[]  = {0xD0,0xC7,0xC6,0xDA,0x00};   // 星期
const uint8_t TXT_L_UPD[]   = {0xB8,0xFC,0xD0,0xC2,0x00};   // 更新
const uint8_t TXT_L_ADV[]   = {0xBD,0xA8,0xD2,0xE9,0x00};   // 建议

// 星期（vvhan 返回数字 1~7 -> "星期三"等完整显示）
const uint8_t TXT_WK[7][8] = {
  {0xD0,0xC7,0xC6,0xDA,0xD2,0xBB,0x00,0x00},   // 星期一
  {0xD0,0xC7,0xC6,0xDA,0xB6,0xFE,0x00,0x00},   // 星期二
  {0xD0,0xC7,0xC6,0xDA,0xC8,0xFD,0x00,0x00},   // 星期三
  {0xD0,0xC7,0xC6,0xDA,0xCB,0xC4,0x00,0x00},   // 星期四
  {0xD0,0xC7,0xC6,0xDA,0xCE,0xE5,0x00,0x00},   // 星期五
  {0xD0,0xC7,0xC6,0xDA,0xC1,0xF9,0x00,0x00},   // 星期六
  {0xD0,0xC7,0xC6,0xDA,0xC8,0xD5,0x00,0x00},   // 星期日
};

// ---- 风向 ----
const uint8_t TXT_DIR_E[]  = {0xB6,0xAB,0x00};                 // 东
const uint8_t TXT_DIR_S[]  = {0xC4,0xCF,0x00};                 // 南
const uint8_t TXT_DIR_W[]  = {0xCE,0xF7,0x00};                 // 西
const uint8_t TXT_DIR_N[]  = {0xB1,0xB1,0x00};                 // 北
const uint8_t TXT_DIR_NE[] = {0xB6,0xAB,0xB1,0xB1,0x00};      // 东北
const uint8_t TXT_DIR_SE[] = {0xB6,0xAB,0xC4,0xCF,0x00};      // 东南
const uint8_t TXT_DIR_NW[] = {0xCE,0xF7,0xB1,0xB1,0x00};      // 西北
const uint8_t TXT_DIR_SW[] = {0xCE,0xF7,0xC4,0xCF,0x00};      // 西南
const uint8_t TXT_CALM[]   = {0xCE,0xA2,0xB7,0xE7,0x00};      // 微风
const uint8_t TXT_LVL[]    = {0xBC,0xB6,0x00};                 // 级

// ---- 天气状况分类 ----
const uint8_t TXT_W_SUNNY[]    = {0xC7,0xE7,0x00};                       // 晴
const uint8_t TXT_W_CLOUDY[]   = {0xB6,0xE0,0xD4,0xC6,0x00};             // 多云
const uint8_t TXT_W_OVERCAST[] = {0xD2,0xF5,0x00};                        // 阴
const uint8_t TXT_W_RAIN[]     = {0xD3,0xEA,0x00};                        // 雨
const uint8_t TXT_W_LRAIN[]    = {0xD0,0xA1,0xD3,0xEA,0x00};             // 小雨
const uint8_t TXT_W_MRAIN[]    = {0xD6,0xD0,0xD3,0xEA,0x00};             // 中雨
const uint8_t TXT_W_HRAIN[]    = {0xB4,0xF3,0xD3,0xEA,0x00};             // 大雨
const uint8_t TXT_W_SHOWER[]   = {0xD5,0xF3,0xD3,0xEA,0x00};             // 阵雨
const uint8_t TXT_W_STORM[]    = {0xB1,0xA9,0xD3,0xEA,0x00};             // 暴雨
const uint8_t TXT_W_THUNDER[]  = {0xC0,0xD7,0xD5,0xF3,0xD3,0xEA,0x00};   // 雷阵雨
const uint8_t TXT_W_SLEET[]    = {0xD3,0xEA,0xBC,0xD0,0xD1,0xA9,0x00};   // 雨夹雪
const uint8_t TXT_W_SNOW[]     = {0xD1,0xA9,0x00};                        // 雪
const uint8_t TXT_W_FOG[]      = {0xCE,0xED,0x00};                        // 雾
const uint8_t TXT_W_HAZE[]     = {0xF6,0xB2,0x00};                        // 霾
const uint8_t TXT_W_SAND[]     = {0xD1,0xEF,0xC9,0xB3,0x00};             // 扬沙
const uint8_t TXT_W_DUST[]     = {0xC9,0xB3,0xB3,0xBE,0xB1,0xA9,0x00};   // 沙尘暴
const uint8_t TXT_W_UNKNOWN[]  = {0xD7,0xA2,0xD2,0xE2,0x00};             // 注意 (未知)

// ---- 干燥建议 ----
const uint8_t TXT_A_GOOD[]  = {0xCA,0xCA,0xD2,0xCB,0xCC,0xAB,0xD1,0xF4,0xC4,0xDC,0xB8,0xC9,0xD4,0xEF,0x00}; // 适宜太阳能干燥
const uint8_t TXT_A_OK[]    = {0xBF,0xC9,0xBD,0xF8,0xD0,0xD0,0xC1,0xC0,0xC9,0xB9,0x00};                      // 可进行晾晒
const uint8_t TXT_A_PRE[]   = {0xBD,0xA8,0xD2,0xE9,0xCC,0xE1,0xC7,0xB0,0xCA,0xD5,0xC1,0xB8,0x00};             // 建议提前收粮
const uint8_t TXT_A_BAD[]   = {0xB2,0xBB,0xD2,0xCB,0xC2,0xB6,0xCC,0xEC,0xB8,0xC9,0xD4,0xEF,0x00};             // 不宜露天干燥
const uint8_t TXT_A_RAIN[]  = {0xD7,0xA2,0xD2,0xE2,0xB7,0xC0,0xD3,0xEA,0x00};                                // 注意防雨
const uint8_t TXT_A_WEAK[]  = {0xCC,0xAB,0xD1,0xF4,0xC4,0xDC,0xB2,0xBB,0xD7,0xE3,0x00};                        // 太阳能不足
const uint8_t TXT_A_HEAT[]  = {0xB8,0xA8,0xD6,0xFA,0xBC,0xD3,0xC8,0xC8,0x00};                                  // 辅助加热
const uint8_t TXT_A_DUST[]  = {0xD7,0xA2,0xD2,0xE2,0xD1,0xEF,0xB3,0xBE,0x00};                                  // 注意扬尘
const uint8_t TXT_A_MOLD[]  = {0xC3,0xB9,0xB1,0xE4,0xB7,0xE7,0xCF,0xD5,0x00};                                  // 霉变风险
const uint8_t TXT_TODAY_W[] = {0xBD,0xF1,0x00};                                                                  // 今
const uint8_t TXT_TOM_W[]   = {0xC3,0xF7,0x00};                                                                  // 明

// ---- AI 干燥建议字模（esp32_controller 返回建议码，此处映射显示） ----
const uint8_t A_ADV_LOAD[]  = {0xBC,0xD3,0xD4,0xD8,0xD6,0xD0,0x00};                                     // 加载中
const uint8_t A_ADV_GOOD[]  = {0xB9,0xE2,0xD5,0xD5,0xB3,0xE4,0xD7,0xE3,0xD5,0xFD,0xB3,0xA3,0xB8,0xC9,0xD4,0xEF,0x00}; // 光照充足正常干燥
const uint8_t A_ADV_FAN[]   = {0xB7,0xE7,0xBB,0xFA,0xBC,0xD3,0xB4,0xF3,0xBC,0xD3,0xCB,0xD9,0xB7,0xC5,0xC8,0xC8,0x00}; // 风机加大加速放热
const uint8_t A_ADV_DAMPER[]= {0xB7,0xE7,0xC3,0xC5,0xBF,0xAA,0xC6,0xF4,0xB8,0xA8,0xD6,0xFA,0xC9,0xA2,0xC8,0xC8,0x00}; // 风门开启辅助散热
const uint8_t A_ADV_PUMP[]  = {0xCB,0xAE,0xB1,0xC3,0xBF,0xAA,0xC6,0xF4,0xB4,0xA2,0xC8,0xC8,0xD1,0xAD,0xBB,0xB7,0x00}; // 水泵开启储热循环
const uint8_t A_ADV_HEAT[]  = {0xBC,0xD3,0xC8,0xC8,0xBF,0xAA,0xC6,0xF4,0xB2,0xB9,0xC8,0xC8,0xB8,0xC9,0xD4,0xEF,0x00}; // 加热开启补热干燥
const uint8_t A_ADV_DISCH[] = {0xC5,0xC5,0xC1,0xB8,0xBF,0xAA,0xC6,0xF4,0xCA,0xCA,0xCA,0xB1,0xB3,0xF6,0xC1,0xCF,0x00}; // 排粮开启适时出料
const uint8_t A_ADV_RAIN[]  = {0xBD,0xB5,0xD3,0xEA,0xD4,0xA4,0xB1,0xA8,0xCC,0xE1,0xC7,0xB0,0xCA,0xD5,0xC1,0xB8,0x00}; // 降雨预报提前收粮
const uint8_t A_ADV_HUM[]   = {0xB8,0xDF,0xCA,0xAA,0xBB,0xB7,0xBE,0xB3,0xD2,0xD6,0xD6,0xC6,0xB7,0xC5,0xC8,0xC8,0x00}; // 高湿环境抑制放热
const uint8_t A_ADV_LOW[]   = {0xB9,0xE2,0xD5,0xD5,0xB2,0xBB,0xD7,0xE3,0xBC,0xF5,0xBB,0xBA,0xB8,0xC9,0xD4,0xEF,0x00}; // 光照不足减缓干燥
const uint8_t A_ADV_HOT[]   = {0xCE,0xC2,0xB6,0xC8,0xB9,0xFD,0xB8,0xDF,0xD7,0xA2,0xD2,0xE2,0xB7,0xC0,0xCA,0xEE,0x00}; // 温度过高注意防暑
const uint8_t A_ADV_NORM[]  = {0xCF,0xB5,0xCD,0xB3,0xD5,0xFD,0xB3,0xA3,0xB1,0xA3,0xB3,0xD6,0xD4,0xCB,0xD0,0xD0,0x00}; // 系统正常保持运行

// ---- 小时级降水预报 ----
const uint8_t TXT_RAIN_NOW[]     = {0xD5,0xFD,0xD4,0xDA,0xBD,0xB5,0xD3,0xEA,0x00};   // 正在降雨
const uint8_t TXT_RAIN_START[]   = {0xC6,0xF0,0xD3,0xEA,0x00};                       // 起雨
const uint8_t TXT_RAIN_WORD[]    = {0xBD,0xB5,0xD3,0xEA,0x00};                       // 降雨
const uint8_t TXT_RAIN_TO[]      = {0xD6,0xC1,0x00};                                 // 至
const uint8_t TXT_RAIN_NONE[]    = {0xBD,0xF1,0xC8,0xD5,0xCE,0xDE,0xD3,0xEA,0x00};   // 今日无雨
const uint8_t TXT_RAIN_TODAY[]   = {0xBD,0xF1,0xC8,0xD5,0xD3,0xD0,0xD3,0xEA,0x00};   // 今日有雨
const uint8_t TXT_RAIN_PROB[]    = {0xB8,0xC5,0xC2,0xCA,0x00};                       // 概率

/* =========================================================================
 *                4. 颜色 / 屏幕参数
 * ========================================================================= */
#define COLOR_BG          0x1082   // 深蓝灰背景
#define COLOR_HEADER_BG   0x041F   // 标题栏深蓝
#define COLOR_TEXT        0xFFFF
#define COLOR_DIM         0xC618
#define COLOR_VALUE       0x07FF   // 数据值青色
#define COLOR_GOOD        0x07E0   // 适宜 绿
#define COLOR_WARN        0xFD20   // 注意 琥珀
#define COLOR_YELLOW      0xFFE0   // 纯黄（放热集热器标识）
#define COLOR_BAD         0xF800   // 不宜 红
#define COLOR_HOT         0xFBE0   // 高温橙
#define COLOR_COLD        0x001F   // 低温蓝
#define COLOR_DIV         0x4208

// 建议码 -> (字模, 颜色)（依赖上面的颜色宏，须置于此）
static AiAdvice adviceFromCode(uint8_t code) {
  switch (code) {
    case 1:  return { A_ADV_GOOD,  COLOR_GOOD };
    case 2:  return { A_ADV_FAN,   COLOR_GOOD };
    case 3:  return { A_ADV_DAMPER,COLOR_GOOD };
    case 4:  return { A_ADV_PUMP,  COLOR_GOOD };
    case 5:  return { A_ADV_HEAT,  COLOR_WARN };
    case 6:  return { A_ADV_DISCH, COLOR_WARN };
    case 7:  return { A_ADV_RAIN,  COLOR_BAD };
    case 8:  return { A_ADV_HUM,   COLOR_WARN };
    case 9:  return { A_ADV_LOW,   COLOR_WARN };
    case 10: return { A_ADV_HOT,   COLOR_BAD };
    case 11: return { A_ADV_NORM,  COLOR_GOOD };
    default: return { A_ADV_LOAD,  COLOR_DIM };
  }
}

#define SCREEN_W  240   // 竖屏物理尺寸：宽 240
#define SCREEN_H  320   // 高 320
#define HEADER_H   28
#define FOOTER_H   40

// 注意：STM32 参考例程用的是 SPI_MODE3（CPOL=1, CPHA=1）
// 此带字库模块的液晶与字库都按 MODE3 驱动，不能用 MODE0，否则字库数据会错位成乱码
// 字库读取时钟放慢到 1MHz，避免个别芯片时序偏紧导致部分点阵读错（显示乱码/糊字）
SPISettings spiFont(1000000, MSBFIRST, SPI_MODE3);
SPISettings spiLcd (40000000, MSBFIRST, SPI_MODE3);

/* =========================================================================
 *                5. ILI9341 SPI 底层驱动
 * ========================================================================= */
static inline void cs_high()         { digitalWrite(PIN_CS, HIGH); }
static inline void lcd_cs_select()   { digitalWrite(PIN_CS, LOW);  }

static void lcd_write_cmd(uint8_t cmd) {
  cs_high();
  SPI.beginTransaction(spiLcd);
  lcd_cs_select();
  digitalWrite(PIN_DC, LOW);
  SPI.write(cmd);
  cs_high();
  SPI.endTransaction();
}

static void lcd_write_data(const uint8_t* buf, uint32_t len) {
  cs_high();
  SPI.beginTransaction(spiLcd);
  lcd_cs_select();
  digitalWrite(PIN_DC, HIGH);
  for (uint32_t i = 0; i < len; i++) SPI.write(buf[i]);
  cs_high();
  SPI.endTransaction();
}
static inline void lcd_write_data8(uint8_t v)   { lcd_write_data(&v, 1); }
static inline void lcd_write_data16(uint16_t v) {
  uint8_t b[2] = { (uint8_t)(v>>8), (uint8_t)v };
  lcd_write_data(b, 2);
}

static void lcd_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
  lcd_write_cmd(0x2A);
  uint8_t b[4] = { (uint8_t)(x0>>8),(uint8_t)x0, (uint8_t)(x1>>8),(uint8_t)x1 };
  lcd_write_data(b, 4);
  lcd_write_cmd(0x2B);
  uint8_t c[4] = { (uint8_t)(y0>>8),(uint8_t)y0, (uint8_t)(y1>>8),(uint8_t)y1 };
  lcd_write_data(c, 4);
  lcd_write_cmd(0x2C);
}

// 像素块绘制（RGB565 双字节流）
static void lcd_write_pixels(uint32_t count, uint16_t color) {
  uint8_t hi = color >> 8, lo = color;
  cs_high();
  SPI.beginTransaction(spiLcd);
  lcd_cs_select();
  digitalWrite(PIN_DC, HIGH);
  for (uint32_t i = 0; i < count; i++) { SPI.write(hi); SPI.write(lo); }
  cs_high();
  SPI.endTransaction();
}

static void lcd_init() {
  pinMode(PIN_CS,   OUTPUT);
  pinMode(PIN_DC,   OUTPUT);
  pinMode(PIN_RST,  OUTPUT);
  pinMode(PIN_BLK,  OUTPUT);
  digitalWrite(PIN_CS,  HIGH);
  digitalWrite(PIN_DC,  HIGH);
  digitalWrite(PIN_BLK, HIGH);

  digitalWrite(PIN_RST, LOW);  delay(100);
  digitalWrite(PIN_RST, HIGH); delay(150);

  lcd_write_cmd(0x01); delay(150);  // soft reset

  // 参考 STM32 例程的 ILI9341 初始化序列
  static const uint8_t init_cmds[][2] = {
    {0xCF, 3},{0xED, 4},{0xE8, 3},{0xCB, 5},{0xF7, 1},{0xEA, 2},
    {0xC0, 1},{0xC1, 1},{0xC5, 2},{0xC7, 1},{0x36, 1},{0x3A, 1},
    {0xB1, 2},{0xB6, 2},{0xF2, 1},{0x26, 1},{0xE0,15},{0xE1,15}
  };
  static const uint8_t init_data[] = {
    0x00,0xD9,0x30,
    0x64,0x03,0x12,0x81,
    0x85,0x10,0x78,
    0x39,0x2C,0x00,0x34,0x02,
    0x20,
    0x00,0x00,
    0x21,
    0x12,
    0x32,0x3C,
    0xC1,
    0x08,
    0x55,
    0x00,0x18,
    0x0A,0xA2,
    0x00,
    0x01,
    0x0F,0x20,0x1E,0x09,0x12,0x0B,0x50,0xBA,0x44,0x09,0x14,0x05,0x23,0x21,0x00,
    0x00,0x19,0x19,0x00,0x12,0x07,0x2D,0x28,0x3F,0x02,0x0A,0x08,0x25,0x2D,0x0F,
  };
  const uint8_t* p = init_data;
  for (uint8_t i = 0; i < sizeof(init_cmds)/sizeof(init_cmds[0]); i++) {
    lcd_write_cmd(init_cmds[i][0]);
    uint8_t n = init_cmds[i][1];
    if (n) { lcd_write_data(p, n); p += n; }
  }
  lcd_write_cmd(0x11); delay(120);   // sleep out
  lcd_write_cmd(0x29);               // display on

  // 竖屏 240x320（0x08：自然列方向，不镜像）
  // 若上下颠倒改 0x88；左右颠倒改 0x48；都反改 0xC8
  lcd_write_cmd(0x36); lcd_write_data8(0x08);
  lcd_set_window(0, 0, SCREEN_W-1, SCREEN_H-1);
}

static void lcd_fill(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
  lcd_set_window(x, y, x+w-1, y+h-1);
  lcd_write_pixels((uint32_t)w * h, color);
}

static void draw_hline(uint16_t x, uint16_t y, uint16_t w, uint16_t color) {
  lcd_fill(x, y, w, 1, color);
}

/* =========================================================================
 *                6. 字库芯片驱动 (GT24L24A2W16)
 *    CS=HIGH 选中字库；CS=LOW 选中液晶。同一引脚，复用。
 * ========================================================================= */

static void font_read(uint8_t* buf, uint32_t addr, uint16_t len) {
  cs_high();
  SPI.beginTransaction(spiFont);
  digitalWrite(PIN_CS, HIGH);      // 选中字库（CS 高）
  delayMicroseconds(2);            // CS 建立稳定时间
  SPI.write(FONT_CMD_READ);
  SPI.write((addr >> 16) & 0xFF);
  SPI.write((addr >>  8) & 0xFF);
  SPI.write(addr & 0xFF);
  for (uint16_t i = 0; i < len; i++) buf[i] = SPI.transfer(0xFF);
  delayMicroseconds(1);
  cs_high();
  SPI.endTransaction();
}

// 读取一个 GBK 汉字 16x16 点阵 (32 字节)
static void font_get_gbk16(const uint8_t* p, uint8_t* out) {
  uint8_t qh = p[0], ql = p[1];
  if (qh < 0x81 || ql < 0x40 || ql == 0xFF || qh == 0xFF) { memset(out,0,32); return; }
  uint32_t foffset = ((uint32_t)190 * (qh - 0x81)
                     + (ql < 0x7F ? ql - 0x40 : ql - 0x41)) * 32UL;
  font_read(out, GBK_16x16_ST + foffset, 32);
}

// 读取一个 ASCII 8x16 (16 字节)
static void font_get_ascii8x16(uint8_t c, uint8_t* out) {
  font_read(out, ASCII_8x16_ST + ((uint32_t)c) * 16UL, 16);
}

// 读取一个 ASCII 16x32 (64 字节)
static void font_get_ascii16x32(uint8_t c, uint8_t* out) {
  font_read(out, ASCII_16x32_ST + ((uint32_t)c) * 64UL, 64);
}

/* =========================================================================
 *                7. 文本渲染
 * ========================================================================= */

// 绘制一个 8x16 ASCII 字符（fg=前景，bg=背景）
// 字库点阵是"列优先"存储：buf[2*c + r/8] 的 bit (7 - r%8) = 第 r 行第 c 列的像素
static void draw_char_ascii(uint16_t x, uint16_t y, uint8_t c,
                            uint16_t fg, uint16_t bg) {
  if (c < 0x20) { lcd_fill(x, y, 8, 16, bg); return; }
  uint8_t buf[16];
  font_get_ascii8x16(c, buf);
  lcd_set_window(x, y, x+7, y+15);
  cs_high();
  SPI.beginTransaction(spiLcd);
  lcd_cs_select();
  digitalWrite(PIN_DC, HIGH);
  uint8_t fhi = fg >> 8, flo = fg & 0xFF;
  uint8_t bhi = bg >> 8, blo = bg & 0xFF;
  // 字库列存储方向与 LCD 窗口一致：LCD 第 c2 列 ← 字库第 c2 列
  for (uint8_t r = 0; r < 16; r++) {
    uint8_t row_byte_off = r >> 3;
    uint8_t row_bit = 7 - (r & 7);
    for (uint8_t c2 = 0; c2 < 8; c2++) {
      uint8_t byte_idx = (c2 << 1) | row_byte_off;
      bool on = (buf[byte_idx] >> row_bit) & 1;
      if (on) { SPI.write(fhi); SPI.write(flo); }
      else    { SPI.write(bhi); SPI.write(blo); }
    }
  }
  cs_high();
  SPI.endTransaction();
}

// 绘制一个 16x32 ASCII 字符（大字，用于温度）
// LCD 第 c2 列 ← 字库第 c2 列
static void draw_char_ascii32(uint16_t x, uint16_t y, uint8_t c,
                              uint16_t fg, uint16_t bg) {
  if (c < 0x20 || c > 0x7E) { lcd_fill(x, y, 16, 32, bg); return; }
  uint8_t buf[64];
  font_get_ascii16x32(c, buf);
  lcd_set_window(x, y, x+15, y+31);
  cs_high();
  SPI.beginTransaction(spiLcd);
  lcd_cs_select();
  digitalWrite(PIN_DC, HIGH);
  uint8_t fhi = fg >> 8, flo = fg & 0xFF;
  uint8_t bhi = bg >> 8, blo = bg & 0xFF;
  for (uint8_t r = 0; r < 32; r++) {
    uint8_t row_byte_off = r >> 3;
    uint8_t row_bit = 7 - (r & 7);
    for (uint8_t c2 = 0; c2 < 16; c2++) {
      uint8_t byte_idx = (c2 << 2) | row_byte_off;
      bool on = (buf[byte_idx] >> row_bit) & 1;
      if (on) { SPI.write(fhi); SPI.write(flo); }
      else    { SPI.write(bhi); SPI.write(blo); }
    }
  }
  cs_high();
  SPI.endTransaction();
}

// 绘制一个 16x16 GBK 汉字
// LCD 第 c 列 ← 字库第 c 列
static void draw_char_gbk16(uint16_t x, uint16_t y, const uint8_t* p,
                            uint16_t fg, uint16_t bg) {
  uint8_t buf[32];
  font_get_gbk16(p, buf);
  lcd_set_window(x, y, x+15, y+15);
  cs_high();
  SPI.beginTransaction(spiLcd);
  lcd_cs_select();
  digitalWrite(PIN_DC, HIGH);
  uint8_t fhi = fg >> 8, flo = fg & 0xFF;
  uint8_t bhi = bg >> 8, blo = bg & 0xFF;
  for (uint8_t r = 0; r < 16; r++) {
    uint8_t row_byte_off = r >> 3;
    uint8_t row_bit = 7 - (r & 7);
    for (uint8_t c = 0; c < 16; c++) {
      uint8_t byte_idx = (c << 1) | row_byte_off;
      bool on = (buf[byte_idx] >> row_bit) & 1;
      if (on) { SPI.write(fhi); SPI.write(flo); }
      else    { SPI.write(bhi); SPI.write(blo); }
    }
  }
  cs_high();
  SPI.endTransaction();
}

// 读取一个 GBK 汉字 24x24 点阵（72 字节：列优先，每列 3 字节）
static void font_get_gbk24(const uint8_t* p, uint8_t* out) {
  uint8_t qh = p[0], ql = p[1];
  if (qh < 0x81 || ql < 0x40 || ql == 0xFF || qh == 0xFF) { memset(out, 0, 72); return; }
  uint32_t foffset = ((uint32_t)190 * (qh - 0x81)
                     + (ql < 0x7F ? ql - 0x40 : ql - 0x41)) * 72UL;
  font_read(out, GBK_24x24_ST + foffset, 72);
}

// 绘制一个 24x24 GBK 汉字（列优先：buf[c*3 + r/8] bit(7-r%8)）
static void draw_char_gbk24(uint16_t x, uint16_t y, const uint8_t* p,
                            uint16_t fg, uint16_t bg) {
  uint8_t buf[72];
  font_get_gbk24(p, buf);
  lcd_set_window(x, y, x+23, y+23);
  cs_high();
  SPI.beginTransaction(spiLcd);
  lcd_cs_select();
  digitalWrite(PIN_DC, HIGH);
  uint8_t fhi = fg >> 8, flo = fg & 0xFF;
  uint8_t bhi = bg >> 8, blo = bg & 0xFF;
  for (uint8_t r = 0; r < 24; r++) {
    uint8_t row_byte_off = r >> 3;
    uint8_t row_bit = 7 - (r & 7);
    for (uint8_t c = 0; c < 24; c++) {
      uint8_t byte_idx = c * 3 + row_byte_off;
      bool on = (buf[byte_idx] >> row_bit) & 1;
      if (on) { SPI.write(fhi); SPI.write(flo); }
      else    { SPI.write(bhi); SPI.write(blo); }
    }
  }
  cs_high();
  SPI.endTransaction();
}

// 绘制 24x24 GBK 字符串（以 0x00 结尾）
static uint16_t draw_string_cn24(uint16_t x, uint16_t y, const uint8_t* s,
                                 uint16_t fg, uint16_t bg = COLOR_BG,
                                 uint16_t max_x = SCREEN_W) {
  while (*s) {
    if (x + 24 > max_x) break;
    if (*s >= 0x80) {
      if (*(s+1) == 0) break;
      draw_char_gbk24(x, y, s, fg, bg);
      s += 2; x += 24;
    } else {
      draw_char_ascii(x + 8, y + 4, *s, fg, bg);  // 24px 串中嵌入 ASCII 粗略居中
      s += 1; x += 12;
    }
  }
  return x;
}

// 绘制 GBK 字符串（以 0x00 结尾）。返回下一字符 x。
static uint16_t draw_string_cn(uint16_t x, uint16_t y, const uint8_t* s,
                               uint16_t fg, uint16_t bg = COLOR_BG,
                               uint16_t max_x = SCREEN_W) {
  while (*s) {
    uint16_t cw = (*s >= 0x80) ? 16 : 8;
    if (x + cw > max_x) break;
    if (*s >= 0x80) {
      if (*(s+1) == 0) break; // 防孤字节
      draw_char_gbk16(x, y, s, fg, bg);
      s += 2; x += 16;
    } else {
      draw_char_ascii(x, y, *s, fg, bg);
      s += 1; x += 8;
    }
  }
  return x;
}

// 居中绘制 GBK 字符串
static void draw_string_cn_centered(uint16_t y, const uint8_t* s,
                                    uint16_t fg, uint16_t bg = COLOR_BG) {
  uint16_t w = 0;
  for (const uint8_t* p = s; *p; ) {
    if (*p >= 0x80) { w += 16; p += 2; } else { w += 8; p++; }
  }
  uint16_t x = (w < SCREEN_W) ? (SCREEN_W - w) / 2 : 0;
  draw_string_cn(x, y, s, fg, bg);
}

// 绘制普通 C 字符串 (UTF-8 / ASCII)，按 ASCII 渲染
static uint16_t draw_string_ascii(uint16_t x, uint16_t y, const char* s,
                                  uint16_t fg, uint16_t bg = COLOR_BG,
                                  uint16_t max_x = SCREEN_W) {
  while (*s) {
    if (x + 8 > max_x) break;
    draw_char_ascii(x, y, (uint8_t)*s, fg, bg);
    s++; x += 8;
  }
  return x;
}

/* =========================================================================
 *                8. 天气数据 & 解析
 *    (enum WeatherCat / struct WeatherData / struct Advice 已移至
 *     weather_types.h，见文件顶部 include)
 * ========================================================================= */
WeatherData g_wx;
int tmpTrend = 0;   // 温度趋势 1=升 -1=降 0=平稳（下一小时 vs 当前小时）
int humTrend = 0;   // 湿度趋势 1=升 -1=降 0=平稳

// ---- 极简 JSON 字符串提取器（替代 ArduinoJson，零依赖） ----
// 在 body[start..] 中寻找 '"key"' 并取出其字符串值（处理 \" 转义）
// 返回 true 表示找到；同名键从 start 之后取首次出现的值。
static bool jsonGetStr(const char* body, size_t start,
                       const char* key, char* out, size_t max) {
  size_t kl = strlen(key);
  for (size_t i = start; body[i]; i++) {
    if (body[i] == '"' && strncmp(body + i + 1, key, kl) == 0 &&
        body[i + 1 + kl] == '"') {
      size_t j = i + 1 + kl;   // 指向键的收尾引号
      j++;                     // 跳过收尾引号
      while (body[j] == ' ' || body[j] == '\t' || body[j] == '\n' || body[j] == '\r') j++;
      if (body[j] != ':') continue;
      j++;
      while (body[j] == ' ' || body[j] == '\t' || body[j] == '\n' || body[j] == '\r') j++;
      if (body[j] != '"') continue;   // 仅处理字符串值
      j++;
      size_t k = 0;
      while (body[j] && body[j] != '"' && k < max - 1) {
        if (body[j] == '\\' && body[j + 1]) {
          if (body[j + 1] == '"')  { out[k++] = '"';  j += 2; }
          else if (body[j + 1] == '\\') { out[k++] = '\\'; j += 2; }
          else { out[k++] = body[j]; j++; }
        } else {
          out[k++] = body[j++];
        }
      }
      out[k] = '\0';
      return true;
    }
  }
  return false;
}

static size_t jsonFind(const char* body, size_t start, const char* needle) {
  size_t nl = strlen(needle);
  for (size_t i = start; body[i]; i++)
    if (strncmp(body + i, needle, nl) == 0) return i;
  return (size_t)-1;
}

// 从字符串中提取第一个整数（"高温 33℃" -> 33, "<3级" -> 3, "微风" -> 0）
static int extractInt(const char* s) {
  while (*s && !(*s >= '0' && *s <= '9')) s++;
  return atoi(s);
}

/* itboy 天气接口返回格式（纯 HTTP，免 Key）：
   {"message":"success...","status":200,"date":"20260809",
    "cityInfo":{"city":"北京市","citykey":"101010100",...},
    "data":{
      "shidu":"56%","pm25":21,"pm10":45,"quality":"良",
      "wendu":"31.8","ganmao":"...",
      "forecast":[
        {"date":"09","high":"高温 30℃","low":"低温 24℃","ymd":"2026-08-09",
         "week":"星期日","sunrise":"05:20","sunset":"19:19","aqi":65,
         "fx":"东南风","fl":"1级","type":"晴","notice":"..."},   // 今天
        {"date":"10",...,"week":"星期一",...},                     // 明天
        ...
      ],
      "yesterday":{...}
    }}
*/
static bool fetchWeatherFor(const CityEntry* c) {
  if (WiFi.status() != WL_CONNECTED) { Serial.println("[fetch] WiFi 未连接"); return false; }
  HTTPClient http;
  String url = String(WEATHER_HOST) + String(WEATHER_PATH) + c->code;
  Serial.print("[fetch] URL: "); Serial.println(url);
  if (!http.begin(url)) {
    Serial.println("[fetch] http.begin 失败（域名解析失败）");
    http.end(); return false;
  }
  http.setTimeout(10000);
  int code = http.GET();
  Serial.printf("[fetch] HTTP 状态码: %d\n", code);
  if (code != 200) {
    if (code < 0)
      Serial.printf("[fetch] 连接错误码: %d （-1=无法连接/DNS, -2=发送失败, -3=解码失败）\n", code);
    http.end(); return false;
  }
  String body = http.getString();
  http.end();
  Serial.printf("[fetch] 响应: %.300s\n", body.c_str());

  // 校验：必须有 "wendu" 字段才认为有效
  char tmp[24];
  if (!jsonGetStr(body.c_str(), 0, "wendu", tmp, sizeof(tmp))) {
    Serial.println("[fetch] 响应中无 wendu 字段，解析失败");
    return false;
  }

  WeatherData w;
  char v[24] = {0};

  // 当前温度 / 湿度
  jsonGetStr(body.c_str(), 0, "wendu", v, sizeof(v));   w.tem = atoi(v);
  jsonGetStr(body.c_str(), 0, "shidu", v, sizeof(v));   w.hum = atoi(v);

  // 星期："星期日" 等
  jsonGetStr(body.c_str(), 0, "week", v, sizeof(v));
  if      (strstr(v,"\xe4\xb8\x80")) w.week = 0;   // 一
  else if (strstr(v,"\xe4\xba\x8c")) w.week = 1;   // 二
  else if (strstr(v,"\xe4\xb8\x89")) w.week = 2;   // 三
  else if (strstr(v,"\xe5\x9b\x9b")) w.week = 3;   // 四
  else if (strstr(v,"\xe4\xba\x94")) w.week = 4;   // 五
  else if (strstr(v,"\xe5\x85\xad")) w.week = 5;   // 六
  else if (strstr(v,"\xe6\x97\xa5")) w.week = 6;   // 日

  // forecast 数组：第 1 个 '{' = 今天，第 2 个 '{' = 明天
  size_t fc = jsonFind(body.c_str(), 0, "forecast");
  if (fc != (size_t)-1) {
    size_t br1 = jsonFind(body.c_str(), fc, "{");
    if (br1 != (size_t)-1) {
      jsonGetStr(body.c_str(), br1, "type", v, sizeof(v));
      strncpy(w.wea_utf8, v, sizeof(w.wea_utf8) - 1);

      jsonGetStr(body.c_str(), br1, "high", v, sizeof(v));  w.tmax = extractInt(v);
      jsonGetStr(body.c_str(), br1, "low",  v, sizeof(v));  w.tmin = extractInt(v);

      jsonGetStr(body.c_str(), br1, "fx", v, sizeof(v));
      strncpy(w.win_utf8, v, sizeof(w.win_utf8) - 1);

      jsonGetStr(body.c_str(), br1, "fl", v, sizeof(v));    w.win_level = extractInt(v);

      // 明天（forecast 内第 2 个 '{'）
      size_t br2 = jsonFind(body.c_str(), br1 + 1, "{");
      if (br2 != (size_t)-1) {
        jsonGetStr(body.c_str(), br2, "type", v, sizeof(v));
        strncpy(w.tmr_wea_utf8, v, sizeof(w.tmr_wea_utf8) - 1);

        jsonGetStr(body.c_str(), br2, "high", v, sizeof(v));  w.tmr_tmax = extractInt(v);
        jsonGetStr(body.c_str(), br2, "low",  v, sizeof(v));  w.tmr_tmin = extractInt(v);
        w.has_tomorrow = true;
      }
    }
  }

  w.valid = true;
  w.fetch_ms = millis();
  g_wx = w;
  g_dispCity = c;             // 记录当前显示天气的城市
  Serial.printf("[fetch] 解析成功（城市: %s）\n", c->name);
  fetchHourlyRain(c);         // 补充当天逐小时降水预报（失败不影响主天气）
  sendWeatherToController();  // 把天气数据发给 esp32_controller（AI 推理用）
  return true;
}

// 通用 IP 定位：GET 指定 URL 的纯文本，用 UTF-8 城市名（CITIES[].name）做子串匹配。
// 网络错误/HTTP 非 200 才重试；成功返回但城市未匹配（城市表缺失）则不重试。
static bool locateByText(const char* url, const char* label, int attempts) {
  if (WiFi.status() != WL_CONNECTED) return false;
  for (int att = 1; att <= attempts; att++) {
    HTTPClient http;
    Serial.printf("[loc] %s 第 %d/%d 次\n", label, att, attempts);
    if (!http.begin(url)) { http.end(); return false; }
    http.setTimeout(8000);
    http.addHeader("User-Agent", "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/120.0.0.0 Safari/537.36");
    int code = http.GET();
    if (code != 200) {
      Serial.printf("[loc] %s HTTP %d\n", label, code);
      http.end();
      if (att < attempts) delay(1500);
      continue;
    }
    String body = http.getString();
    http.end();
    Serial.printf("[loc] %s 返回: %.120s\n", label, body.c_str());
    for (size_t i = 0; i < sizeof(CITIES)/sizeof(CITIES[0]); i++) {
      if (body.indexOf(CITIES[i].name) >= 0) {
        // 该城市配置了固定区县？若有则用固定区县天气
        const DistrictEntry* pin = findPinnedDistrict(CITIES[i].name);
        if (pin) {
          setDistrictCity(pin);
          Serial.printf("[loc] 城市%s固定区县(%s): %s\n", label, CITIES[i].name, pin->name);
          return true;
        }
        g_activeCity = &CITIES[i];
        Serial.printf("[loc] 定位城市(%s): %s\n", label, CITIES[i].name);
        return true;
      }
    }
    Serial.printf("[loc] %s 城市未匹配（城市表无此城市，不重试）\n", label);
    return false;
  }
  Serial.printf("[loc] %s 多次尝试后仍失败\n", label);
  return false;
}

// 高德 IP 定位：请求 v3/ip 得到 province/city/adcode/rectangle，
// 再用 rectangle 中心点做逆地理编码（regeo）拿到 district 区县。
// 匹配逻辑：先匹配区县表 DISTRICTS（更精确），未命中再匹配市级城市表 CITIES。
static bool locateByAmap() {
  if (WiFi.status() != WL_CONNECTED) return false;
  HTTPClient http;
  char url[160];
  snprintf(url, sizeof(url), "http://restapi.amap.com/v3/ip?key=%s", AMAP_KEY);
  Serial.println("[loc] 高德 IP 定位...");
  if (!http.begin(url)) { http.end(); return false; }
  http.setTimeout(8000);
  http.addHeader("User-Agent", "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36");
  int code = http.GET();
  if (code != 200) {
    Serial.printf("[loc] 高德 HTTP %d\n", code);
    http.end(); return false;
  }
  String body = http.getString();
  http.end();
  Serial.printf("[loc] 高德返回: %.200s\n", body.c_str());

  // ---- 解析 rectangle，中心点 regeo 找区县 ----
  char rect[64] = {0};
  jsonGetStr(body.c_str(), 0, "rectangle", rect, sizeof(rect));
  if (rect[0]) {
    // rectangle 格式 "minLon,minLat;maxLon,maxLat"
    double minLon, minLat, maxLon, maxLat;
    if (sscanf(rect, "%lf,%lf;%lf,%lf", &minLon, &minLat, &maxLon, &maxLat) == 4) {
      double clon = (minLon + maxLon) / 2.0;
      double clat = (minLat + maxLat) / 2.0;
      char rurl[200];
      snprintf(rurl, sizeof(rurl),
               "http://restapi.amap.com/v3/geocode/regeo?key=%s&location=%.5f,%.5f",
               AMAP_KEY, clon, clat);
      Serial.printf("[loc] 高德 regeo 中心点: %.4f,%.4f\n", clon, clat);
      if (http.begin(rurl)) {
        http.setTimeout(8000);
        int code2 = http.GET();
        if (code2 == 200) {
          String rbody = http.getString();
          Serial.printf("[loc] regeo 返回: %.150s\n", rbody.c_str());
          // 区县字段
          char district[32] = {0};
          jsonGetStr(rbody.c_str(), 0, "district", district, sizeof(district));
          if (district[0]) {
            for (size_t i = 0; i < sizeof(DISTRICTS)/sizeof(DISTRICTS[0]); i++) {
              if (strcmp(district, DISTRICTS[i].name) == 0) {
                setDistrictCity(&DISTRICTS[i]);
                Serial.printf("[loc] 定位到区县(amap): %s\n", DISTRICTS[i].name);
                http.end();
                return true;
              }
            }
            Serial.printf("[loc] 区县 %s 不在映射表，回退市级定位\n", district);
          }
        }
        http.end();
      }
    }
  }

  // ---- 市级匹配：用高德返回 body 中的城市名子串匹配 CITIES ----
  for (size_t i = 0; i < sizeof(CITIES)/sizeof(CITIES[0]); i++) {
    if (body.indexOf(CITIES[i].name) >= 0) {
      // 该城市配置了固定区县？若有则用固定区县天气
      const DistrictEntry* pin = findPinnedDistrict(CITIES[i].name);
      if (pin) {
        setDistrictCity(pin);
        Serial.printf("[loc] 城市%s固定区县: %s\n", CITIES[i].name, pin->name);
        return true;
      }
      g_activeCity = &CITIES[i];
      Serial.printf("[loc] 定位城市(amap): %s\n", CITIES[i].name);
      return true;
    }
  }
  return false;
}

static bool fetchLocatedCity() {
#if AUTO_LOCATE == 0
  return false;
#endif
  if (WiFi.status() != WL_CONNECTED) return false;

  // 1) 高德 IP 定位（主源，可精确到区）
  if (locateByAmap()) return true;

  // 2) 备用源 myip.ipip.net（纯文本 UTF-8）
  if (locateByText("http://myip.ipip.net", "ipip", 1)) return true;

  // 3) 备用源 cip.cc（纯文本 HTML）
  if (locateByText("http://cip.cc", "cip.cc", 1)) return true;

  return false;
}

// 带重试的天气获取：定位城市优先，失败则回退默认城市
static bool fetchWeather() {
  if (WiFi.status() != WL_CONNECTED) { Serial.println("[fetch] WiFi 未连接"); return false; }
  const CityEntry* order[2];
  int n = 0;
  if (g_activeCity) order[n++] = g_activeCity;          // 定位城市（若匹配成功）
  if (n == 0 || order[0] != city) order[n++] = city;    // 默认城市兜底
  for (int i = 0; i < n; i++) {
    Serial.printf("[fetch] 尝试城市: %s\n", order[i]->name);
    if (fetchWeatherFor(order[i])) return true;
  }
  return false;
}

// 带重试的获取（热点网络不稳定，失败后自动重试几次）
static bool fetchWeatherRetry(int attempts = 4) {
  for (int i = 1; i <= attempts; i++) {
    Serial.printf("[fetch] 第 %d/%d 次尝试\n", i, attempts);
    if (fetchWeather()) return true;
    if (i < attempts) delay(2000);
  }
  Serial.println("[fetch] 多次尝试后仍失败");
  return false;
}

// WMO 天气代码 -> UTF-8 中文（与 weatherGlyph 匹配一致）
static void wmoToUtf8(int wcode, char* out, size_t max) {
  const char* s = "";
  switch (wcode) {
    case 0:  case 1:  s = "\xe6\x99\xb4"; break;                            // 晴
    case 2:           s = "\xe5\xa4\x9a\xe4\xba\x91"; break;               // 多云
    case 3:           s = "\xe9\x98\xb4"; break;                           // 阴
    case 45: case 48: s = "\xe9\x9b\xbe"; break;                           // 雾
    case 51: case 53: case 55: s = "\xe5\xb0\x8f\xe9\x9b\xa8"; break;      // 小雨
    case 56: case 57: case 66: case 67: s = "\xe9\x9b\xa8\xe5\xa4\xb9\xe9\x9b\xaa"; break; // 雨夹雪
    case 61:          s = "\xe5\xb0\x8f\xe9\x9b\xa8"; break;               // 小雨
    case 63:          s = "\xe4\xb8\xad\xe9\x9b\xa8"; break;               // 中雨
    case 65:          s = "\xe5\xa4\xa7\xe9\x9b\xa8"; break;               // 大雨
    case 71: case 73: case 75: case 77: s = "\xe9\x9b\xaa"; break;         // 雪
    case 80: case 81: s = "\xe9\x98\xb5\xe9\x9b\xa8"; break;               // 阵雨
    case 82:          s = "\xe5\xa4\xa7\xe9\x9b\xa8"; break;               // 大雨
    case 85: case 86: s = "\xe9\x98\xb5\xe9\x9b\xaa"; break;               // 阵雪
    case 95: case 96: case 99: s = "\xe9\x9b\xb7\xe9\x98\xb5\xe9\x9b\xa8"; break; // 雷阵雨
    default:          s = ""; break;
  }
  strncpy(out, s, max - 1);
  out[max - 1] = 0;
}

// 前置声明（weatherGlyph 定义在后面，但 fetchHourlyRain 需要在此调用）
static const uint8_t* weatherGlyph(const char* wea_utf8);

// ---- 当地光照强度（Open-Meteo shortwave_radiation W/m² -> lx）----
// 经验换算：1 W/m² ≈ 120 lx（晴空直射约 120 lx/W/m²）
static bool fetchLocalLux(const CityEntry* c) {
  if (WiFi.status() != WL_CONNECTED) return false;
  HTTPClient http;
  char url[192];
  snprintf(url, sizeof(url),
           "http://api.open-meteo.com/v1/forecast?latitude=%.4f&longitude=%.4f"
           "&hourly=shortwave_radiation&forecast_days=1&timezone=Asia%%2FShanghai",
           c->lat, c->lon);
  if (!http.begin(url)) { http.end(); return false; }
  http.setTimeout(8000);
  int code = http.GET();
  if (code != 200) {
    // 偶发失败：1 秒后重试一次
    Serial.printf("[lux] HTTP %d，重试一次\n", code);
    http.end();
    delay(1000);
    if (!http.begin(url)) { http.end(); return false; }
    http.setTimeout(8000);
    code = http.GET();
    if (code != 200) { Serial.printf("[lux] HTTP %d\n", code); http.end(); return false; }
  }
  String body = http.getString();
  http.end();

  size_t sr = jsonFind(body.c_str(), 0, "\"shortwave_radiation\":[");
  if (sr == (size_t)-1) { Serial.println("[lux] 未找到 shortwave_radiation 字段"); return false; }
  sr += strlen("\"shortwave_radiation\":[");
  float rad[24];
  int ridx = 0;
  const char* rp = body.c_str() + sr;
  while (*rp && *rp != ']' && ridx < 24) {
    if (*rp==',' || *rp==' ' || *rp=='\t' || *rp=='\n' || *rp=='\r') { rp++; continue; }
    // 用 strtof 解析小数（如 658.0），strtol 会把小数点后 .0 误判为独立数字导致数组错位
    char* rend;
    float rv = strtof(rp, &rend);
    if (rend == rp) { rp++; continue; }
    rad[ridx++] = rv;
    rp = rend;
  }
  if (ridx <= 0) return false;

  // 当前小时 + 下一小时（本地时间不可用时仅更新峰值）
  struct tm t;
  int curH = -1;
  if (getLocalTime(&t, 0) && t.tm_year > 100) curH = t.tm_hour;
  if (curH >= 0 && curH < ridx) localLux = rad[curH] * 120.0f;
  if (curH >= 0 && curH + 1 < ridx) luxNextH = rad[curH + 1] * 120.0f;
  else if (curH >= 0 && curH + 1 >= ridx) luxNextH = -1.0f;
  // 今日峰值
  float peak = -1.0f;
  for (int i = 0; i < ridx; i++)
    if (rad[i] > peak) { peak = rad[i]; luxPeakHour = i; }
  luxPeakH = (peak >= 0) ? peak * 120.0f : -1.0f;
  Serial.printf("[lux] 当前%.0f lx, 下一时%.0f lx, 峰值%.0f lx @ %d时\n",
                localLux, luxNextH, luxPeakH, luxPeakHour);
  return true;
}

// Open-Meteo 逐小时数据（纯 HTTP、免费、无 Key）：
//  - 降水概率：取第一个 >=50% 的小时作为降雨开始时间
//  - weather_code：当前小时实时天气实况
//  - shortwave_radiation：当地光照强度（W/m²，换算为 lx）
// 注意：Open-Meteo 需经纬度（已内置在城市表 lat/lon 字段）。
static bool fetchHourlyRain(const CityEntry* c) {
  if (WiFi.status() != WL_CONNECTED) return false;
  HTTPClient http;
  char url[224];
  snprintf(url, sizeof(url),
           "http://api.open-meteo.com/v1/forecast?latitude=%.4f&longitude=%.4f"
           "&hourly=precipitation_probability,weather_code,shortwave_radiation,temperature_2m,relativehumidity_2m"
           "&forecast_days=2&timezone=Asia%%2FShanghai",
           c->lat, c->lon);
  Serial.printf("[rain] 请求小时降水: %.4f, %.4f\n", c->lat, c->lon);
  if (!http.begin(url)) { http.end(); return false; }
  http.setTimeout(10000);
  int code = http.GET();
  if (code != 200) {
    Serial.printf("[rain] HTTP %d\n", code);
    http.end(); return false;
  }
  String body = http.getString();
  http.end();
  Serial.printf("[rain] 响应: %.200s\n", body.c_str());
  size_t pp = jsonFind(body.c_str(), 0, "\"precipitation_probability\":[");
  if (pp == (size_t)-1) { Serial.println("[rain] 未找到降水概率字段"); return false; }
  pp += strlen("\"precipitation_probability\":[");
  int prob[24];
  int idx = 0;
  const char* p = body.c_str() + pp;
  while (*p && *p != ']' && idx < 24) {
    if (*p==',' || *p==' ' || *p=='\t' || *p=='\n' || *p=='\r') { p++; continue; }
    char* end;
    long v = strtol(p, &end, 10);
    if (end == p) { p++; continue; }
    prob[idx++] = (int)v;
    p = end;
  }
  // 今天是否预报有雨：以 itboy 当天天气类型为准（与主天气显示一致），
  // 避免把已过去的凌晨降水概率误算为"今日有雨"
  bool todayRain = (strstr(g_wx.wea_utf8,"\xe9\x9b\xa8") || strstr(g_wx.wea_utf8,"\xe9\x9b\xaa"));
  // 从当前时刻起找第一个降水小时（当前小时也算，用于"正在降雨"）
  struct tm t;
  int curH = -1;
  if (getLocalTime(&t, 0) && t.tm_year > 100) curH = t.tm_hour;
  // 实时天气 + 未来天气变化：解析完整 weather_code 数组（今明两天，最多 48 个）
  // 用 weatherGlyph 映射后比较显示文本，找到第一个与当前不同的时刻
  if (curH >= 0 && curH < 24) {
    size_t wc = jsonFind(body.c_str(), 0, "\"weather_code\":[");
    if (wc != (size_t)-1) {
      wc += strlen("\"weather_code\":[");
      int wcodes[48];
      int widx = 0;
      const char* q = body.c_str() + wc;
      while (*q && *q != ']' && widx < 48) {
        if (*q==',' || *q==' ' || *q=='\t' || *q=='\n' || *q=='\r') { q++; continue; }
        char* end2;
        long cv = strtol(q, &end2, 10);
        if (end2 == q) { q++; continue; }
        wcodes[widx++] = (int)cv;
        q = end2;
      }
      // 当前实时天气
      g_wx.realtime_wea[0] = 0;
      if (widx > curH) wmoToUtf8(wcodes[curH], g_wx.realtime_wea, sizeof(g_wx.realtime_wea));
      Serial.printf("[rain] 当前%02d时实时天气: %s\n", curH, g_wx.realtime_wea);
      // 未来变化：找第一个与当前"显示文本"不同的时刻（最多往后看 12 小时）
      const uint8_t* curGlyph = g_wx.realtime_wea[0] ? weatherGlyph(g_wx.realtime_wea) : nullptr;
      g_wx.fc_change_hour = -1;
      g_wx.fc_change_wea[0] = 0;
      if (curGlyph) {
        for (int i = curH + 1; i < widx && i <= curH + 12; i++) {
          char fut[16];
          wmoToUtf8(wcodes[i], fut, sizeof(fut));
          if (!fut[0]) continue;
          if (weatherGlyph(fut) != curGlyph) {   // 显示不同 => 天气变化
            g_wx.fc_change_hour = i - curH;
            strncpy(g_wx.fc_change_wea, fut, sizeof(g_wx.fc_change_wea) - 1);
            break;
          }
        }
      }
      if (g_wx.fc_change_hour > 0)
        Serial.printf("[rain] 未来%d小时后转: %s\n", g_wx.fc_change_hour, g_wx.fc_change_wea);

      // ---- 明天天气变化预测（明天 0~23 时 = 数组 index 24~47） ----
      g_wx.tmr_fc_hour = -1;
      g_wx.tmr_fc_wea[0] = 0;
      g_wx.tmr_wea_start[0] = 0;
      if (widx > 24) {
        wmoToUtf8(wcodes[24], g_wx.tmr_wea_start, sizeof(g_wx.tmr_wea_start));  // 明天 0 时
        const uint8_t* tmr0Glyph = weatherGlyph(g_wx.tmr_wea_start);
        for (int i = 25; i < widx; i++) {
          char fut[16];
          wmoToUtf8(wcodes[i], fut, sizeof(fut));
          if (!fut[0]) continue;
          if (weatherGlyph(fut) != tmr0Glyph) {   // 明天天气变化
            g_wx.tmr_fc_hour = i - 24;            // 明天几点（0~23）
            strncpy(g_wx.tmr_fc_wea, fut, sizeof(g_wx.tmr_fc_wea) - 1);
            break;
          }
        }
        if (g_wx.tmr_fc_hour >= 0)
          Serial.printf("[rain] 明天%d时转: %s\n", g_wx.tmr_fc_hour, g_wx.tmr_fc_wea);
      }
    }
  }

  // ---- 温度/湿度趋势：当前小时 vs 下一小时（供平板显示升/降箭头） ----
  tmpTrend = 0; humTrend = 0;
  size_t tp = jsonFind(body.c_str(), 0, "\"temperature_2m\":[");
  if (tp != (size_t)-1) {
    tp += strlen("\"temperature_2m\":[");
    int tarr[48]; int tIdx = 0;
    const char* q1 = body.c_str() + tp;
    while (*q1 && *q1 != ']' && tIdx < 48) {
      if (*q1==','||*q1==' '||*q1=='\t'||*q1=='\n'||*q1=='\r') { q1++; continue; }
      char* te; float tv = strtof(q1, &te);
      if (te == q1) { q1++; continue; }
      tarr[tIdx++] = (int)tv; q1 = te;
    }
    if (curH >= 0 && curH + 1 < tIdx) {
      if (tarr[curH+1] > tarr[curH]) tmpTrend = 1;
      else if (tarr[curH+1] < tarr[curH]) tmpTrend = -1;
    }
  }
  size_t hp = jsonFind(body.c_str(), 0, "\"relativehumidity_2m\":[");
  if (hp != (size_t)-1) {
    hp += strlen("\"relativehumidity_2m\":[");
    int harr[48]; int hIdx = 0;
    const char* q2 = body.c_str() + hp;
    while (*q2 && *q2 != ']' && hIdx < 48) {
      if (*q2==','||*q2==' '||*q2=='\t'||*q2=='\n'||*q2=='\r') { q2++; continue; }
      char* he; float hv = strtof(q2, &he);
      if (he == q2) { q2++; continue; }
      harr[hIdx++] = (int)hv; q2 = he;
    }
    if (curH >= 0 && curH + 1 < hIdx) {
      if (harr[curH+1] > harr[curH]) humTrend = 1;
      else if (harr[curH+1] < harr[curH]) humTrend = -1;
    }
  }

  if (curH < 0 || curH >= 24) {
    // 本地时间不可用：不猜测具体时刻，避免"预计00:00起雨"误报
    g_wx.rain_ok = true;
    g_wx.rain_any_today = todayRain;
    g_wx.rain_start_hour = -1;
    g_wx.rain_end_hour = -1;
    g_wx.rain_start_prob = 0;
    Serial.printf("[rain] 本地时间不可用，仅类型判断: 今日有雨=%d\n", todayRain);
    return true;
  }
  int next = -1;
  for (int i = curH; i < idx; i++)
    if (prob[i] >= 50) { next = i; break; }
  // 计算该段连续降雨的结束小时（含结束小时；持续到深夜则保留 23，便于显示"至23:00"）
  int endH = -1;
  if (next >= 0) {
    endH = next;
    while (endH + 1 < idx && prob[endH + 1] >= 50) endH++;
  }
  g_wx.rain_ok = true;
  g_wx.rain_any_today = todayRain;
  g_wx.rain_start_hour = next;
  g_wx.rain_end_hour = endH;
  g_wx.rain_start_prob = (next >= 0) ? prob[next] : 0;
  Serial.printf("[rain] 今日类型有雨=%d, 当前%d时, 未来降雨时段=%d~%d, 起始概率=%d%%\n",
                todayRain, curH, next, endH, g_wx.rain_start_prob);

  // 同步更新当地光照（复用独立函数）
  fetchLocalLux(c);
  return true;
}

// 无线光照接收：检查 UDP 是否有来自 C3 节点的数据包，格式 "LUX:12345"
static void checkLightUdp() {
  if (!udpReady) return;   // UDP 未就绪（WiFi 未连）时不处理，防止 lwIP 崩溃
  int len = udpRx.parsePacket();
  if (len > 0) {
    char buf[32];
    int n = udpRx.read(buf, sizeof(buf) - 1);
    if (n > 0) buf[n] = '\0';
    // 格式：LUX<序号1~4>:数值
    if (strncmp(buf, "LUX", 3) == 0 && buf[3] >= '1' && buf[3] <= '4' && buf[4] == ':') {
      int id = buf[3] - '1';   // 0~3
      float v = atof(buf + 5);
      if (v >= 0) {
        wirelessLux[id] = v;
        lightOnline[id] = true;
        lastLightMs[id] = millis();
      }
    }
  }
  // 超时判离线（逐节点检查）
  for (int i = 0; i < LIGHT_NODES; i++) {
    if (lightOnline[i] && (millis() - lastLightMs[i] > LIGHT_TIMEOUT))
      lightOnline[i] = false;
  }
}

// UART 有线发送：把四个集热器光照发给 esp32_controller
// 格式: LUX:1234;2345;3456;4567\n（离线节点发 -1）
static void sendLuxToController() {
  char line[48];
  snprintf(line, sizeof(line), "LUX:%d;%d;%d;%d\n",
           (lightOnline[0] && wirelessLux[0] >= 0) ? (int)wirelessLux[0] : -1,
           (lightOnline[1] && wirelessLux[1] >= 0) ? (int)wirelessLux[1] : -1,
           (lightOnline[2] && wirelessLux[2] >= 0) ? (int)wirelessLux[2] : -1,
           (lightOnline[3] && wirelessLux[3] >= 0) ? (int)wirelessLux[3] : -1);
  Serial1.print(line);
}

// UART 发送：天气数据给 esp32_controller（AI 推理用）
// 格式: WX:温度;湿度;天气码;降雨概率;降雨开始小时\n
// 天气码: 1晴 2多云 3阴 4雨 5雪 6雷阵雨 7雾 8未知
static int weatherCodeOf() {
  const char* w = g_wx.wea_utf8;
  if (strstr(w,"\xe6\x9b\xb4\xe9\x9b\xa8")) return 4;       // 暴雨→雨
  if (strstr(w,"\xe9\x9b\xb7")) return 6;                    // 雷阵雨
  if (strstr(w,"\xe9\x9b\xa8")) return 4;                    // 雨
  if (strstr(w,"\xe9\x9b\xaa")) return 5;                    // 雪
  if (strstr(w,"\xe9\x9c\xbe") || strstr(w,"\xe9\x9b\xbe")) return 7; // 雾/霾
  if (strstr(w,"\xe9\x98\xb4")) return 3;                    // 阴
  if (strstr(w,"\xe4\xba\x91")) return 2;                    // 多云
  if (strstr(w,"\xe6\x99\xb4")) return 1;                    // 晴
  return 8;
}
static void sendWeatherToController() {
  if (!g_wx.valid) return;
  char line[64];
  snprintf(line, sizeof(line), "WX:%d;%d;%d;%d;%d\n",
           g_wx.tem,
           g_wx.hum,
           weatherCodeOf(),
           (g_wx.rain_start_hour >= 0) ? g_wx.rain_start_prob : 0,
           (g_wx.rain_start_hour >= 0) ? g_wx.rain_start_hour : -1);
  Serial1.print(line);
  Serial.printf("[UART] wx-> %s", line);
}

// UART 接收：esp32_controller 发回的 AI 建议，格式 ADV:码\n
static void recvAdviceFromController() {
  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\n') {
      if (uartRecvBuf.startsWith("ADV:")) {
        int code = uartRecvBuf.substring(4).toInt();
        aiAdviceCode = (uint8_t)code;
        aiAdviceValid = true;
        lastAdviceMs = millis();
        Serial.printf("[UART] adv<- %d\n", code);
      } else if (uartRecvBuf.startsWith("DIS:")) {
        int d = uartRecvBuf.substring(4).toInt();
        if (d >= 1 && d <= 4) {
          g_discharging = d;
        } else {
          g_discharging = -1;   // 非干燥阶段：清除显示
        }
        g_controllerMs = millis();   // 收到控制器消息 → 视为在线
        drawLightDischarging();
        Serial.printf("[UART] dis<- %d\n", d);
      } else if (uartRecvBuf.startsWith("RMN:")) {
        g_remainSec = uartRecvBuf.substring(4).toFloat();   // 干燥剩余时间
        g_remainMs = millis();
        g_controllerMs = millis();
      } else if (uartRecvBuf.startsWith("STT:")) {
        // 元件工作状态 STT:m=60;p=0;h=1;d=45 （风机%/水泵%/加热/风门角度）
        String s = uartRecvBuf.substring(4);
        int pm = s.indexOf(";p="), ph = s.indexOf(";h="), pd = s.indexOf(";d=");
        if (pm > 0) ctrlFan    = s.substring(2, pm).toInt();
        if (ph > pm) ctrlPump   = s.substring(pm + 3, ph).toInt();
        if (pd > ph) ctrlHeater = s.substring(ph + 3, pd).toInt();
        if (pd >= 0) ctrlDamper = s.substring(pd + 3).toInt();
        g_controllerMs = millis();
        Serial.printf("[UART] stt<- fan=%d%% pump=%d%% heat=%d damp=%d\n",
                      ctrlFan, ctrlPump, ctrlHeater, ctrlDamper);
      } else if (uartRecvBuf.startsWith("STA:")) {
        ctrlStage = uartRecvBuf.substring(4).toInt();
        g_controllerMs = millis();
        Serial.printf("[UART] stage<- %d\n", ctrlStage);
      } else if (uartRecvBuf.startsWith("SYS:")) {
        ctrlMode = uartRecvBuf.substring(4).toInt();
        g_controllerMs = millis();
      } else if (uartRecvBuf.startsWith("LUX:")) {
        ctrlLux = uartRecvBuf.substring(4).toFloat();
        g_controllerMs = millis();
      } else if (uartRecvBuf.startsWith("FLT:")) {
        ctrlFault = uartRecvBuf.substring(4).toInt();
        g_controllerMs = millis();
      } else if (uartRecvBuf.startsWith("DRT:")) {
        g_drySec = uartRecvBuf.substring(4).toFloat();
        g_controllerMs = millis();
      }
      uartRecvBuf = "";
    } else if (c == '\r') {
      // 忽略回车
    } else {
      if (uartRecvBuf.length() < 31) uartRecvBuf += c;
    }
  }
}

/* =========================================================================
 *           Web 服务器（平板远程监控，独立不影响其他功能）
 * ========================================================================= */
// 网页 HTML（PROGMEM 存储不占 RAM；Web 异常不影响天气/光照/显示等核心功能）
static const char WEB_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>干燥系统远程监控</title>
<style>
body{font-family:"SimHei","Microsoft YaHei",sans-serif;background:#ffffff;color:#333;margin:0;padding:14px;font-size:22px;font-weight:bold}
h1{font-size:28px;color:#333;margin:0 0 12px;text-align:center}
.card{background:#f4f6f9;border:1px solid #e0e3e8;border-radius:8px;padding:12px;margin-bottom:10px}
.row{display:flex;justify-content:space-between;margin:3px 0}
.label{color:#333;font-weight:normal}
.value{font-weight:bold;font-size:22px}
.remain{font-size:150px;font-weight:bold;color:#0d6efd;text-align:center;line-height:1.1}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:6px}
.btn{background:#fff;color:#0d6efd;border:2px solid #0d6efd;padding:7px 0;border-radius:6px;font-size:22px;font-weight:bold}
.btn:active{background:#0d6efd;color:#fff}
.slider-cell{display:flex;flex-direction:column;gap:4px;background:#eef1f5;border-radius:6px;padding:8px}
.slider-cell span{color:#333;font-size:22px;font-weight:normal}
.slider-cell input{width:100%;height:26px}
.slider-cell b{text-align:center;font-size:22px;font-weight:bold}
.ok{color:#198754}.fault{color:#dc3545}
</style></head><body>
<h1>相变材料光热干燥远程监控</h1>
<div class="card" style="text-align:center;height:300px;display:flex;flex-direction:column;justify-content:flex-start;padding:10px 12px">
<div style="display:grid;grid-template-columns:1fr auto 1fr;align-items:center;margin:12px 0 6px">
<span style="justify-self:start;white-space:nowrap"><span class="label" style="font-size:22px;font-weight:bold">实时光照</span><span class="value" id="mlux" style="font-size:28px">--</span></span>
<div class="label" id="rtitle" style="font-size:28px;font-weight:bold">干燥剩余时间</div>
<span></span>
</div>
<div style="flex:1;display:flex;align-items:center;justify-content:center">
<div id="drysecBox" style="flex:1;text-align:center">
<div class="label" style="font-size:28px;font-weight:bold">预计干燥时长</div>
<div id="drysec" style="font-size:150px;font-weight:bold;color:#333;line-height:1.1">--</div>
</div>
<div style="flex:1">
<div class="label" id="rlab" style="font-size:28px;font-weight:bold">剩余干燥时间</div>
<div class="remain" id="remain">--</div>
</div>
</div>
</div>
<div style="display:flex;gap:10px;margin-bottom:12px">
<div class="card" style="flex:1;margin-bottom:0">
<div class="row"><span class="label">故障</span><span class="value" id="wflt">--</span></div>
<div class="row"><span class="label">控制器</span><span id="online">--</span></div>
<div class="row"><span class="label">城市</span><span class="value" id="city">--</span></div>
<div class="row"><span class="label">时间</span><span class="value" id="wtime">--</span></div>
<div class="row"><span class="label">天气</span><span class="value" id="wea">--</span></div>
<div class="row"><span class="label">干燥建议</span><span class="value" id="advice">--</span></div>
</div>
<div class="card" style="flex:1;margin-bottom:0">
<div class="row"><span class="label">温度🌡️</span><span class="value"><span id="temp">--</span><span id="tmpT" style="color:#ff9800"></span></span></div>
<div class="row"><span class="label">湿度💧</span><span class="value"><span id="hum">--</span><span id="humT" style="color:#0d6efd"></span></span></div>
<div class="row"><span class="label">最高/最低</span><span class="value" style="white-space:nowrap"><span id="wtmax" style="color:#ff9800">--</span><span style="color:#ff9800">℃</span> / <span id="wtmin" style="color:#0d6efd">--</span><span style="color:#0d6efd">℃</span></span></div>
<div class="row"><span class="label">天气变化</span><span class="value" id="wfc">--</span></div>
<div class="row"><span class="label">明天天气</span><span class="value" id="wtmr">--</span></div>
<div class="row"><span class="label">明天建议</span><span class="value" id="wtadv">--</span></div>
</div>
</div>
<div style="display:flex;gap:10px;margin-bottom:12px">
<div class="card" style="flex:1;margin-bottom:0"><div class="label" style="margin-bottom:6px;font-weight:bold">集热器光照 (lx)</div>
<div class="grid" id="lux"></div>
<div class="row"><span class="label">当前放热🔥</span><span class="value" id="dis">--</span></div>
</div>
<div class="card" style="flex:1;margin-bottom:0"><div class="label" style="margin-bottom:6px;font-weight:bold">元件工作情况</div>
<div class="row"><span class="label">风机💨</span><span class="value" id="mfan">--</span></div>
<div class="row"><span class="label">水泵💧</span><span class="value" id="mpump">--</span></div>
<div class="row"><span class="label">电加热🔥</span><span class="value" id="mheater">--</span></div>
<div class="row"><span class="label">风门⚙️</span><span class="value" id="mdamper">--</span></div>
</div>
</div>
<div class="card"><div class="label" style="margin-bottom:6px">模式切换</div>
<div class="grid" style="grid-template-columns:1fr 1fr 1fr">
<button class="btn" onclick="ctl('MODE=COLLECT')">COLLECT</button>
<button class="btn" onclick="ctl('MODE=AUTO')">AUTO</button>
<button class="btn" onclick="ctl('MODE=MANUAL')">MANUAL</button>
</div></div>
<div class="card"><div class="label" style="margin-bottom:6px">COLLECT 流程（电机/导轨需先点手动模式）</div>
<div class="grid">
<button class="btn" onclick="ctl('START')">流程启动</button>
<button class="btn" onclick="ctl('STOP')">流程停止</button>
</div>
<div style="margin-top:6px;background:#eef1f5;border:2px solid #b9c1cb;border-radius:6px;padding:8px">
<div style="text-align:center;margin-bottom:6px"><button class="btn" style="width:55%;padding:7px 0" onclick="ctl('MANUAL')">手动模式</button></div>
<div class="grid">
<button class="btn" onclick="ctl('INSERT')">导轨插入</button>
<button class="btn" onclick="ctl('LEAVE')">导轨离开</button>
<button class="btn" onclick="ctl('MOTOR=1')">电机启动</button>
<button class="btn" onclick="ctl('MOTOR=0')">电机停止</button>
</div>
</div></div>
<div class="card"><div class="label" style="margin-bottom:6px">干燥元件操控（仅 MANUAL 模式）</div>
<div class="slider-cell"><span>风机</span><input id="fanSld" type="range" min="0" max="100" step="5" value="0" oninput="this.nextElementSibling.textContent=this.value+'%'" onchange="ctl('FAN='+this.value)"><b id="fanVal">0%</b></div>
<div class="slider-cell" style="margin-top:6px"><span>水泵</span><input id="pumpSld" type="range" min="0" max="100" step="5" value="0" oninput="this.nextElementSibling.textContent=this.value+'%'" onchange="ctl('PUMP='+this.value)"><b id="pumpVal">0%</b></div>
<div class="slider-cell" style="margin-top:6px"><span>风门</span><input id="damperSld" type="range" min="0" max="90" step="1" value="0" oninput="this.nextElementSibling.textContent=this.value+'°'" onchange="ctl('DAMPER='+this.value)"><b id="damperVal">0°</b></div>
<div class="grid" style="margin-top:6px">
<button class="btn" onclick="ctl('HEATER=1')">电加热 开</button>
<button class="btn" onclick="ctl('HEATER=0')">电加热 关</button>
</div></div>
<script>
function ctl(cmd){fetch('/ctl?cmd='+cmd);}
function emojiOf(s){
if(!s)return '';
if(s.indexOf('雷')>=0)return '⛈️ ';
if(s.indexOf('雪')>=0)return '❄️ ';
if(s.indexOf('雨')>=0)return '🌧️ ';
if(s.indexOf('雾')>=0||s.indexOf('霾')>=0)return '🌫️ ';
if(s.indexOf('晴')>=0)return '☀️ ';
if(s.indexOf('云')>=0)return '⛅ ';
if(s.indexOf('阴')>=0)return '☁️ ';
return '';
}
function adviceText(c){
var m={0:'加载中',1:'光照充足正常干燥',2:'风机加大加速放热',3:'风门开启辅助散热',4:'水泵开启储热循环',5:'加热开启补热干燥',6:'排粮开启适时出料',7:'降雨预报提前收粮',8:'高湿环境抑制放热',9:'光照不足减缓干燥',10:'温度过高注意防暑',11:'系统正常保持运行'};
return m[c]||('建议'+c);
}
function render(d){
var stageText={0:'回原点',1:'就绪',2:'等待光照',3:'转盘定位',4:'气管插入',5:'干燥中',6:'导轨回位',7:'转盘换位',8:'导轨插入'};
var r=document.getElementById('remain'),rt=document.getElementById('rtitle');
var db=document.getElementById('drysecBox'),rl=document.getElementById('rlab');
if(d.remain>=0){rt.textContent='⏱️ 干燥时间';r.textContent=d.remain.toFixed(0)+' s';r.style.fontSize='150px';r.style.color='#333';db.style.display='block';rl.style.display='block';document.getElementById('drysec').textContent=(d.drysec!=null&&d.drysec>=0)?d.drysec.toFixed(0)+' s':'--';}
else if(d.stage!=null&&d.stage>=0){rt.textContent='⏳ 当前阶段';r.textContent=stageText[d.stage]||'--';r.style.fontSize='150px';r.style.color=(d.stage===1)?'#198754':'#0d6efd';db.style.display='none';rl.style.display='none';}
else{rt.textContent='⏱️ 干燥时间';r.textContent='--';r.style.fontSize='150px';r.style.color='#333';db.style.display='none';rl.style.display='none';}
var o=document.getElementById('online');
o.textContent = d.online?'✓ 在线':'✗ 离线'; o.className = d.online?'ok':'fault';
document.getElementById('city').textContent = d.city;
document.getElementById('wea').textContent = emojiOf(d.wea)+(d.wea||'')+(d.todayAdvice?('，'+d.todayAdvice):'');
document.getElementById('temp').textContent = d.temp+'°C';
document.getElementById('tmpT').textContent = d.tmpTrend===1?' ↑':(d.tmpTrend===-1?' ↓':'');
document.getElementById('hum').textContent = d.hum+'%';
document.getElementById('humT').textContent = d.humTrend===1?' ↑':(d.humTrend===-1?' ↓':'');
var av=d.advice;
document.getElementById('advice').textContent = (av!=null&&av!=='null'&&av!=='')?adviceText(av):'--';
document.getElementById('wtime').textContent = d.time||'--';
document.getElementById('wtmax').textContent = '🔺'+(d.tmax!=null?d.tmax:'--');
document.getElementById('wtmin').textContent = '🔻'+(d.tmin!=null?d.tmin:'--');
var lt=(d.luxTrend===1)?' 光照↑':((d.luxTrend===-1)?' 光照↓':'');
document.getElementById('wfc').textContent = (d.fcWea&&d.fcHour!=null&&d.fcHour>=0)?d.fcHour+'小时后转'+emojiOf(d.fcWea)+d.fcWea+lt:'--';
document.getElementById('wtmr').textContent = d.tmrWea?(emojiOf(d.tmrWea)+d.tmrWea+(d.tmrTmax!=null?' '+d.tmrTmax+'°/'+d.tmrTmin+'°':'')):'--';
document.getElementById('wtadv').textContent = d.tmrAdvice||'--';
var wf=document.getElementById('wflt');
if(d.fault!=null&&d.fault>=0){wf.textContent=d.fault?'⚠️ 故障':'✓ 正常';wf.style.color=d.fault?'#dc3545':'#198754';}
else{wf.textContent='--';wf.style.color='';}
document.getElementById('dis').textContent = d.dis>0?d.dis+' 号':'--';
document.getElementById('mlux').textContent = (d.light!=null&&d.light>=0)?'☀️ '+d.light+' lx':'--';
var l='';for(var i=0;i<4;i++)l+='<div><span style="font-weight:normal">集热器'+(i+1)+'☀️:</span> '+d.lux[i]+'</div>';
document.getElementById('lux').innerHTML=l;
document.getElementById('mfan').textContent = d.fan!=null?d.fan+'%':'--';
document.getElementById('mpump').textContent = d.pump!=null?d.pump+'%':'--';
document.getElementById('mheater').textContent = d.heater!=null?(d.heater?'开':'关'):'--';
document.getElementById('mdamper').textContent = d.damper!=null?d.damper+'°':'--';
if(d.mode!=null&&d.mode!==2){
var fs=document.getElementById('fanSld'),ps=document.getElementById('pumpSld'),ds=document.getElementById('damperSld');
fs.value=0;document.getElementById('fanVal').textContent='0%';
ps.value=0;document.getElementById('pumpVal').textContent='0%';
ds.value=0;document.getElementById('damperVal').textContent='0°';
}
}
function upd(){fetch('/data').then(r=>r.json()).then(render);}
setInterval(upd,1000);upd();
</script></body></html>
)rawhtml";

// 前向声明：categorize 定义在文件后部（adviceToday/adviceTomorrow 区域）
static WeatherCat categorize(const char* wea_utf8);

// 明日干燥建议（UTF-8 文本，复刻 adviceTomorrow 逻辑供网页显示）
static String tomorrowAdviceUtf8() {
  const WeatherData& w = g_wx;
  if (!w.valid || !w.has_tomorrow) return String("加载中");
  WeatherCat cat = categorize(w.tmr_wea_utf8);
  if (cat == CAT_RAIN || cat == CAT_SNOW || cat == CAT_STORM) {
    if (cat == CAT_STORM || cat == CAT_SNOW) return String("不宜露天干燥");
    return String("注意防雨");
  }
  if (cat == CAT_FOG) {
    if (w.win_level >= 5) return String("注意扬尘");
    return String("太阳能不足");
  }
  if (cat == CAT_OVERCAST) return String("太阳能不足");
  if (cat == CAT_CLOUDY) {
    if (w.tmr_tmax >= 35) return String("可进行晾晒");
    return String("适宜太阳能干燥");
  }
  if (cat == CAT_SUNNY) {
    if (w.tmr_tmax >= 35) return String("建议提前收粮");
    return String("适宜太阳能干燥");
  }
  return String("可进行晾晒");
}

// 今日干燥建议（UTF-8 文本，复刻 adviceToday 逻辑供网页显示）
static String todayAdviceUtf8() {
  const WeatherData& w = g_wx;
  if (!w.valid) return String("加载中");
  WeatherCat cat = categorize(w.wea_utf8);
  if (cat == CAT_RAIN || cat == CAT_SNOW || cat == CAT_STORM) {
    if (w.hum >= 85) return String("霉变风险");
    if (cat == CAT_STORM || cat == CAT_SNOW) return String("不宜露天干燥");
    return String("注意防雨");
  }
  if (cat == CAT_FOG) {
    if (w.win_level >= 5) return String("注意扬尘");
    return String("太阳能不足");
  }
  if (cat == CAT_OVERCAST) {
    if (w.hum >= 80) return String("太阳能不足");
    return String("辅助加热");
  }
  if (cat == CAT_CLOUDY) {
    if (w.tem >= 35) return String("可进行晾晒");
    if (w.hum >= 80) return String("太阳能不足");
    if (w.tem >= 20 && w.hum < 70) return String("适宜太阳能干燥");
    return String("可进行晾晒");
  }
  if (cat == CAT_SUNNY) {
    if (w.tem >= 35) return String("建议提前收粮");
    if (w.win_level >= 6) return String("注意扬尘");
    if (w.hum >= 80) return String("可进行晾晒");
    return String("适宜太阳能干燥");
  }
  return String("可进行晾晒");
}

// 构建 /data 数据（只读现有变量，不修改任何状态，异常不影响核心功能）
static String buildDataJson() {
  String j = "{";
  // 区县定位时显示"城市+区县"（如"成都郫都区"），普通城市直接显示城市名
  String cityShow;
  if (g_activeDistrict) {
    const CityEntry* cc = findCityOfDistrict(g_activeDistrict->name);
    if (cc) cityShow = String(cc->name) + String(g_activeDistrict->name);
  }
  if (cityShow.length() == 0 && g_dispCity) cityShow = g_dispCity->name;
  j += "\"city\":\"" + cityShow + "\",";
  j += "\"wea\":\"" + String(g_wx.wea_utf8) + "\",";
  j += "\"temp\":" + String(g_wx.tem) + ",";
  j += "\"hum\":" + String(g_wx.hum) + ",";
  j += "\"lux\":[";
  for (int i = 0; i < 4; i++) {
    if (i) j += ",";
    j += lightOnline[i] ? String((int)wirelessLux[i]) : String("null");
  }
  j += "],";
  j += "\"advice\":" + String(aiAdviceValid ? String(aiAdviceCode) : String("null")) + ",";
  j += "\"dis\":" + String(g_discharging > 0 ? g_discharging : -1) + ",";
  // 剩余时间：仅干燥阶段有效（RMN>=0 且 5 秒内收到）
  bool remainValid = (g_remainSec >= 0 && millis() - g_remainMs < 5000);
  j += "\"remain\":" + String(remainValid ? g_remainSec : -1.0f, 1) + ",";
  // 在线判定：5 秒内收到控制器任何消息（DIS/RMN/STT）即在线，不依赖干燥阶段
  bool controllerOnline = (millis() - g_controllerMs < 5000);
  j += "\"online\":" + String(controllerOnline ? "true" : "false");
  j += ",\"fan\":" + String(ctrlFan);
  j += ",\"pump\":" + String(ctrlPump);
  j += ",\"heater\":" + String(ctrlHeater);
  j += ",\"damper\":" + String(ctrlDamper);
  j += ",\"stage\":" + String(ctrlStage);
  j += ",\"mode\":" + String(ctrlMode);
  j += ",\"light\":" + String(ctrlLux, 0);
  // 时间：日期 + 时间 + 星期（NTP）
  String timeStr;
  {
    struct tm t;
    if (getLocalTime(&t, 0) && t.tm_year > 100) {
      const char* wd[] = {"周日","周一","周二","周三","周四","周五","周六"};
      char tb[40];
      snprintf(tb, sizeof(tb), "%04d-%02d-%02d，%02d:%02d，%s",
               t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
               t.tm_hour, t.tm_min, wd[t.tm_wday % 7]);
      timeStr = tb;
    }
  }
  j += ",\"time\":\"" + timeStr + "\"";
  j += ",\"todayAdvice\":\"" + todayAdviceUtf8() + "\"";
  j += ",\"tmax\":" + String(g_wx.tmax) + ",\"tmin\":" + String(g_wx.tmin);
  j += ",\"fcHour\":" + String(g_wx.fc_change_hour) + ",\"fcWea\":\"" + String(g_wx.fc_change_wea) + "\"";
  j += ",\"tmrWea\":\"" + String(g_wx.tmr_wea_utf8) + "\",\"tmrTmax\":" + String(g_wx.tmr_tmax) + ",\"tmrTmin\":" + String(g_wx.tmr_tmin);
  j += ",\"tmrAdvice\":\"" + tomorrowAdviceUtf8() + "\"";
  j += ",\"fault\":" + String(ctrlFault);
  j += ",\"drysec\":" + String(g_drySec, 0);
  j += ",\"tmpTrend\":" + String(tmpTrend);
  j += ",\"humTrend\":" + String(humTrend);
  // 光照趋势：下一小时 vs 当前（±20 lx 内视为平稳）
  int lt = 0;
  if (luxNextH >= 0 && localLux >= 0) {
    if (luxNextH > localLux + 20) lt = 1;
    else if (luxNextH < localLux - 20) lt = -1;
  }
  j += ",\"luxTrend\":" + String(lt);
  j += "}";
  return j;
}

// 初始化 Web 服务器（失败不影响其他功能；WiFi 未连接时跳过）
static void initWebServer() {
  if (WiFi.status() != WL_CONNECTED) return;
  webServer.on("/", HTTP_GET, []() {
    webServer.send_P(200, "text/html", WEB_HTML);
  });
  webServer.on("/data", HTTP_GET, []() {
    webServer.send(200, "application/json", buildDataJson());
  });
  webServer.on("/ctl", HTTP_GET, []() {
    String cmd = webServer.arg("cmd");
    if (cmd.length() > 0 && cmd.length() <= 31) {
      Serial1.printf("CTL:%s\n", cmd.c_str());   // 转发给控制器执行
      webServer.send(200, "text/plain", "OK");
    } else {
      webServer.send(400, "text/plain", "BAD CMD");
    }
  });
  webServer.begin();
  webReady = true;
  Serial.printf("[WEB] WiFi 网址 http://%s\n", WiFi.localIP().toString().c_str());
  Serial.printf("[WEB] 热点网址固定 http://192.168.4.1\n");
}

// 确保 UDP 已就绪：WiFi 已连接但 UDP 未初始化时补初始化
static void ensureUdpReady() {
  if (WiFi.status() == WL_CONNECTED && !udpReady) {
    udpRx.begin(UDP_PORT);
    udpReady = true;
    Serial.printf("无线光照 UDP 端口 %d 就绪\n", UDP_PORT);
  }
}

// 显示联网光照强度及变化趋势（y=100 行，1 分钟局部刷新）
// 布局：x=8 "光照" + 当前值 + 趋势(升/降/平) + "下"下一时值 + "峰"峰值值
static void drawLocalLux(uint16_t y) {
  // 擦除局部区域（x=8..232, y），避免残留
  lcd_fill(8, y, 224, 16, COLOR_BG);
  draw_string_cn(8, y, TXT_L_LUX, COLOR_DIM);            // "光照"
  if (localLux >= 0) {
    uint16_t x = 40;
    // 当前值（>=10 万显示 klx 数值）
    char tb[16];
    snprintf(tb, sizeof(tb), "%.0f", localLux);
    x = draw_string_ascii(x, y, tb, COLOR_VALUE);
    // 趋势：与下一小时比较（有数据时）
    if (luxNextH >= 0) {
      float diff = luxNextH - localLux;
      const uint8_t* tr = TXT_FLAT;
      uint16_t tc = COLOR_DIM;
      if (diff > 2000)      { tr = TXT_UP;   tc = COLOR_GOOD; }
      else if (diff < -2000){ tr = TXT_DOWN; tc = COLOR_BAD; }
      x += 2;
      x = draw_string_cn(x, y, tr, tc);                   // 升/降/平
      x += 2;
      // 下一时
      draw_string_cn(x, y, TXT_NEXT, COLOR_DIM);          // 下
      x += 16;
      char nb[16]; snprintf(nb, sizeof(nb), "%.0f", luxNextH);
      x = draw_string_ascii(x, y, nb, COLOR_DIM);
      x += 2;
    }
    // 峰值
    if (luxPeakH >= 0) {
      draw_string_cn(x, y, TXT_PEAK, COLOR_DIM);          // 峰
      x += 16;
      char pb[16]; snprintf(pb, sizeof(pb), "%.0f", luxPeakH);
      draw_string_ascii(x, y, pb, COLOR_DIM);
    }
  } else {
    draw_string_ascii(40, y, "--", COLOR_DIM);
  }
}

// 字库自检：读取汉字"北"（GBK: B1B1）16x16 点阵前 4 字节，
// 以及 ASCII '0' 的 8x16 / 16x32 点阵前 4 字节。
// 正常应是非零数据（点阵字形）；若全是 FF/00 说明对应字库区没读出来。
static void fontSelfTest() {
  uint8_t buf[64];
  const uint8_t gbk[] = {0xB1, 0xB1};
  font_get_gbk16(gbk, buf);
  Serial.print("[font] GBK16 '北': ");
  for (int i = 0; i < 4; i++) Serial.printf("%02X ", buf[i]);
  Serial.println();

  font_get_ascii8x16('0', buf);
  Serial.print("[font] ASC8  '0': ");
  for (int i = 0; i < 4; i++) Serial.printf("%02X ", buf[i]);
  Serial.println();

  font_get_ascii16x32('0', buf);
  Serial.print("[font] ASC32 '0': ");
  for (int i = 0; i < 4; i++) Serial.printf("%02X ", buf[i]);
  Serial.println();
}

/* =========================================================================
 *                9. 天气类别判定（基于 UTF-8 关键字）
 * ========================================================================= */
static WeatherCat categorize(const char* wea_utf8) {
  if (!wea_utf8 || !*wea_utf8) return CAT_UNKNOWN;
  if (strstr(wea_utf8,"\xe9\x9b\xb7")) return CAT_STORM;   // 雷
  if (strstr(wea_utf8,"\xe9\x9b\xa8")) return CAT_RAIN;    // 雨
  if (strstr(wea_utf8,"\xe9\x9b\xaa")) return CAT_SNOW;    // 雪
  if (strstr(wea_utf8,"\xe6\xb2\x99") || strstr(wea_utf8,"\xe5\xb0\x98") ||
      strstr(wea_utf8,"\xe9\x9b\xbe") || strstr(wea_utf8,"\xe9\x9c\xbe"))
    return CAT_FOG;                                                // 沙/尘/雾/霾
  if (strstr(wea_utf8,"\xe9\x98\xb3")) return CAT_OVERCAST; // 阴
  if (strstr(wea_utf8,"\xe4\xba\x91")) return CAT_CLOUDY;   // 云
  if (strstr(wea_utf8,"\xe6\x99\xb4")) return CAT_SUNNY;    // 晴
  return CAT_UNKNOWN;
}

// 天气描述 -> 中文显示（详细分级：小雨/中雨/大雨/暴雨/雷阵雨/阵雨/雨夹雪）
static const uint8_t* weatherGlyph(const char* wea_utf8) {
  if (!wea_utf8 || !*wea_utf8) return TXT_W_UNKNOWN;
  if (strstr(wea_utf8,"\xe6\x9a\xb4\xe9\x9b\xa8")) return TXT_W_STORM;             // 暴雨
  if (strstr(wea_utf8,"\xe9\x9b\xb7"))             return TXT_W_THUNDER;           // 雷阵雨
  if (strstr(wea_utf8,"\xe5\xa4\xa7\xe9\x9b\xa8")) return TXT_W_HRAIN;             // 大雨
  if (strstr(wea_utf8,"\xe4\xb8\xad\xe9\x9b\xa8")) return TXT_W_MRAIN;             // 中雨
  if (strstr(wea_utf8,"\xe5\xb0\x8f\xe9\x9b\xa8")) return TXT_W_LRAIN;             // 小雨
  if (strstr(wea_utf8,"\xe9\x98\xb5\xe9\x9b\xa8")) return TXT_W_SHOWER;            // 阵雨
  if (strstr(wea_utf8,"\xe9\x9b\xa8\xe5\xa4\xb9\xe9\x9b\xaa")) return TXT_W_SLEET; // 雨夹雪
  if (strstr(wea_utf8,"\xe9\x9b\xa8"))             return TXT_W_RAIN;              // 雨
  if (strstr(wea_utf8,"\xe9\x9b\xaa"))             return TXT_W_SNOW;              // 雪
  if (strstr(wea_utf8,"\xe6\xb2\x99"))             return TXT_W_SAND;              // 扬沙
  if (strstr(wea_utf8,"\xe5\xb0\x98"))             return TXT_W_DUST;              // 沙尘暴
  if (strstr(wea_utf8,"\xe9\x9c\xbe"))             return TXT_W_HAZE;              // 霾
  if (strstr(wea_utf8,"\xe9\x9b\xbe"))             return TXT_W_FOG;               // 雾
  if (strstr(wea_utf8,"\xe9\x98\xb4"))             return TXT_W_OVERCAST;          // 阴
  if (strstr(wea_utf8,"\xe4\xba\x91"))             return TXT_W_CLOUDY;            // 云
  if (strstr(wea_utf8,"\xe6\x99\xb4"))             return TXT_W_SUNNY;             // 晴
  return TXT_W_UNKNOWN;
}

static const uint8_t* dirGlyph(const char* win_utf8) {
  if (strstr(win_utf8,"\xe5\x8d\x97\xe4\xb8\x9c")) return TXT_DIR_SE; // 东南
  if (strstr(win_utf8,"\xe4\xb8\x9c\xe5\x8c\x97")) return TXT_DIR_NE; // 东北
  if (strstr(win_utf8,"\xe8\xa5\xbf\xe5\x8d\x97")) return TXT_DIR_SW; // 西南
  if (strstr(win_utf8,"\xe8\xa5\xbf\xe5\x8c\x97")) return TXT_DIR_NW; // 西北
  if (strstr(win_utf8,"\xe5\x8d\x97")) return TXT_DIR_S;
  if (strstr(win_utf8,"\xe5\x8c\x97")) return TXT_DIR_N;
  if (strstr(win_utf8,"\xe4\xb8\x9c")) return TXT_DIR_E;
  if (strstr(win_utf8,"\xe8\xa5\xbf")) return TXT_DIR_W;
  return TXT_DIR_E;
}

/* =========================================================================
 *                10. 干燥建议
 *    (struct Advice 定义在 weather_types.h)
 * ========================================================================= */

// 今日建议：基于当天天气
static Advice adviceToday(const WeatherData& w) {
  if (!w.valid) return { TXT_LOADING, COLOR_DIM };
  WeatherCat cat = categorize(w.wea_utf8);

  if (cat == CAT_RAIN || cat == CAT_SNOW || cat == CAT_STORM) {
    if (w.hum >= 85) return { TXT_A_MOLD, COLOR_BAD };
    if (cat == CAT_STORM || cat == CAT_SNOW) return { TXT_A_BAD, COLOR_BAD };
    return { TXT_A_RAIN, COLOR_WARN };
  }
  if (cat == CAT_FOG) {
    if (w.win_level >= 5) return { TXT_A_DUST, COLOR_WARN };
    return { TXT_A_WEAK, COLOR_WARN };
  }
  if (cat == CAT_OVERCAST) {
    if (w.hum >= 80) return { TXT_A_WEAK, COLOR_WARN };
    return { TXT_A_HEAT, COLOR_WARN };
  }
  if (cat == CAT_CLOUDY) {
    if (w.tem >= 35)  return { TXT_A_OK, COLOR_GOOD };
    if (w.hum >= 80)  return { TXT_A_WEAK, COLOR_WARN };
    if (w.tem >= 20 && w.hum < 70) return { TXT_A_GOOD, COLOR_GOOD };
    return { TXT_A_OK, COLOR_GOOD };
  }
  if (cat == CAT_SUNNY) {
    if (w.tem >= 35)        return { TXT_A_PRE, COLOR_WARN };
    if (w.win_level >= 6)   return { TXT_A_DUST, COLOR_WARN };
    if (w.hum >= 80)        return { TXT_A_OK, COLOR_GOOD };
    return { TXT_A_GOOD, COLOR_GOOD };
  }
  return { TXT_A_OK, COLOR_GOOD };
}

// 明日建议：基于明天天气（与今日逻辑相同，数据来自 tmr_*）
static Advice adviceTomorrow(const WeatherData& w) {
  if (!w.valid || !w.has_tomorrow) return { TXT_LOADING, COLOR_DIM };
  WeatherCat cat = categorize(w.tmr_wea_utf8);

  if (cat == CAT_RAIN || cat == CAT_SNOW || cat == CAT_STORM) {
    if (cat == CAT_STORM || cat == CAT_SNOW) return { TXT_A_BAD, COLOR_BAD };
    return { TXT_A_RAIN, COLOR_WARN };
  }
  if (cat == CAT_FOG) {
    if (w.win_level >= 5) return { TXT_A_DUST, COLOR_WARN };
    return { TXT_A_WEAK, COLOR_WARN };
  }
  if (cat == CAT_OVERCAST) {
    return { TXT_A_WEAK, COLOR_WARN };
  }
  if (cat == CAT_CLOUDY) {
    if (w.tmr_tmax >= 35) return { TXT_A_OK, COLOR_GOOD };
    return { TXT_A_GOOD, COLOR_GOOD };
  }
  if (cat == CAT_SUNNY) {
    if (w.tmr_tmax >= 35) return { TXT_A_PRE, COLOR_WARN };
    return { TXT_A_GOOD, COLOR_GOOD };
  }
  return { TXT_A_OK, COLOR_GOOD };
}

/* =========================================================================
 *                11. UI 绘制
 *    布局（240x320 竖屏，文字全部横排）：
 *      y=  0..26 : Header（标题居中）
 *      y= 30..54 : 定位标签 + 大号城市名(24x24) + 右侧 WiFi
 *      y= 58..90 : 天气描述(左) + 大温度(右) 16x32 数字
 *      y=100..116: 湿度 | 风向
 *      y=126..142: 最高 | 最低
 *      y=152..168: 风速 | 明天(天气)
 *      y=178..194: 时间 | 星期
 *      y=204     : 分隔线
 *      y=210..226: 标题"集热器内部光照"
 *      y=230..274: 集热器 2×2 网格（4 路光照）
 *      y=280..320: Footer 建议栏
 * ========================================================================= */
static void drawHeader() {
  lcd_fill(0, 0, SCREEN_W, HEADER_H, COLOR_HEADER_BG);
  // 标题"气象监测系统"(6字×16=96px) 单独居中
  uint16_t x0 = (SCREEN_W - 96) / 2;
  draw_string_cn(x0, 6, TXT_TITLE, COLOR_TEXT);
}

// 绘制"标签 + 值"一行
static void draw_label_val_cn(uint16_t x, uint16_t y, const uint8_t* lbl,
                              int val, const char* unit_ascii,
                              const uint8_t* unit_cn, uint16_t vcolor) {
  draw_string_cn(x, y, lbl, COLOR_DIM);
  char buf[16]; snprintf(buf, sizeof(buf), "%d", val);
  uint16_t lbl_w = 0;
  for (const uint8_t* p=lbl; *p; ) { lbl_w += (*p>=0x80)?16:8; if(*p>=0x80) p+=2; else p++; }
  draw_string_ascii(x + lbl_w + 8, y, buf, vcolor);
  uint16_t ux = x + lbl_w + 8 + strlen(buf)*8 + 4;
  if (unit_ascii) ux = draw_string_ascii(ux, y, unit_ascii, COLOR_DIM);
  if (unit_cn)    draw_string_cn(ux, y, unit_cn, COLOR_DIM);
}

// 单个集热器格子的位置（2×2 网格，整体下移 4px，从 y=230 起）
static void lightCellPos(int i, uint16_t& x, uint16_t& y) {
  x = 8 + (i % 2) * 116;
  y = 230 + (i / 2) * 22;
}

// 集热器区域：标题 + 静态标签（只在整屏重绘时画一次）
static void drawLightLabels() {
  // 区域标题"集热器内部温度"
  draw_string_cn(8, 210, TXT_HEADER_COLLECTOR, COLOR_VALUE);
  for (int i = 0; i < LIGHT_NODES; i++) {
    uint16_t x, y; lightCellPos(i, x, y);
    draw_string_cn(x + 12, y, TXT_L_NODE, COLOR_DIM);              // "集热器"(48px)
    draw_char_ascii(x + 12 + 48, y, '1' + i, COLOR_DIM, COLOR_BG); // 序号 1~4
  }
  drawLightDischarging();   // 标题后显示当前放热集热器（黄色）
}

// 标题后实时显示当前放热集热器（黄色）："X号正在放热！"
void drawLightDischarging() {
  uint16_t x = 8 + 7 * 16 + 8;                   // "集热器内部光照" 7 字后右移 8px
  lcd_fill(x, 210, 112, 16, COLOR_BG);           // 擦除旧内容（X号正在放热！约104px）
  if (g_discharging >= 1 && g_discharging <= 4) {
    draw_char_ascii(x, 210, '0' + g_discharging, COLOR_YELLOW, COLOR_BG); // 数字 X
    draw_string_cn(x + 8, 210, TXT_L_DISCH, COLOR_YELLOW);                // "号正在放热！"
  }
}

// 集热器：动态值+状态点（每秒局部刷新，只擦很小区域，避免频闪）
static void drawLightDynamic() {
  for (int i = 0; i < LIGHT_NODES; i++) {
    uint16_t x, y; lightCellPos(i, x, y);
    // 状态点（绿=在线，红=离线）
    lcd_fill(x, y + 5, 6, 6, lightOnline[i] ? COLOR_GOOD : COLOR_BAD);
    // 值区域擦除（值从 x+72 起，标签"集热器N"到 x+68）。
    // 宽度必须限制在【本格子右边界】(x+116)内，否则会抹掉邻居标签
    uint16_t vx = x + 72;
    uint16_t vw = 44;
    uint16_t cellRight = x + 116;
    if (vx + vw > cellRight) vw = cellRight - vx;
    if (vx + vw > SCREEN_W)  vw = SCREEN_W - vx;
    lcd_fill(vx, y, vw, 16, COLOR_BG);
    if (lightOnline[i] && wirelessLux[i] >= 0) {
      char tb[16];
      snprintf(tb, sizeof(tb), "%.0f", wirelessLux[i]);
      draw_string_ascii(vx, y, tb, COLOR_VALUE);
    } else {
      draw_string_ascii(vx, y, "--", COLOR_DIM);
    }
  }
}

// 完整集热器区域（整屏重绘时调用）
static void drawLightArea() {
  drawLightLabels();
  drawLightDynamic();
}

// 完整重绘
static void drawScreen() {
  // 清主区
  lcd_fill(0, HEADER_H, SCREEN_W, SCREEN_H - HEADER_H - FOOTER_H, COLOR_BG);
  drawHeader();

  if (WiFi.status() != WL_CONNECTED) {
    draw_string_cn_centered(80, TXT_CONN_FAIL, COLOR_BAD);
    draw_string_cn_centered(104, TXT_CHECK_WIFI, COLOR_WARN);
    draw_string_cn_centered(128, TXT_RETRY, COLOR_DIM);
  } else if (!g_wx.valid) {
    draw_string_cn_centered(100, TXT_FETCH_FAIL, COLOR_BAD);
    draw_string_cn_centered(124, TXT_RETRY, COLOR_DIM);
    draw_string_cn_centered(148, TXT_LOADING, COLOR_DIM);
  } else {
    // ===== 行 1: 大号城市名 + 右侧 WiFi 字样 =====
    // 自动定位时标签显示"定位"，城市名显示实际天气城市（24x24 大号）
    const uint8_t* cityLabel = g_activeCity ? TXT_L_LOCATED : TXT_L_CITY;
    const uint8_t* cityGbk   = g_dispCity   ? g_dispCity->gbk : city->gbk;
    draw_string_cn(8, 34, cityLabel, COLOR_DIM);                 // 标签 16x16
    uint16_t city_x = 44;
    // 区县定位时：先画所属城市名，再画区县名（如"唐山曹妃甸区"）
    if (g_activeDistrict) {
      const CityEntry* cc = findCityOfDistrict(g_activeDistrict->name);
      if (cc) city_x = draw_string_cn24(city_x, 30, cc->gbk, COLOR_TEXT);   // 城市名
    }
    draw_string_cn24(city_x, 30, cityGbk, COLOR_TEXT);                     // 城市/区县 24x24
    // 右侧 WiFi 字样（颜色随连接状态）
    uint16_t wifiColor = (WiFi.status() == WL_CONNECTED) ? COLOR_GOOD : COLOR_BAD;
    draw_string_ascii(SCREEN_W - 8 - 4*8, 34, "WiFi", wifiColor);

    // ===== 行 2: 天气描述 + 大温度 =====
    // 优先显示实时天气实况（Open-Meteo 当前小时），无实时数据时回退今天的整体预报
    const uint8_t* weaCN = g_wx.realtime_wea[0]
                         ? weatherGlyph(g_wx.realtime_wea)
                         : weatherGlyph(g_wx.wea_utf8);
    // 天气文字（y=60 左侧）+ 未来天气说明（逗号 + 后续天气白色）
    uint16_t wx = 8;
    wx = draw_string_cn(wx, 60, weaCN, COLOR_VALUE);
    if (g_wx.realtime_wea[0]) {
      wx = draw_string_ascii(wx + 2, 60, ",", COLOR_TEXT);            // 逗号
      wx += 2;
      if (g_wx.fc_change_hour > 0 && g_wx.fc_change_wea[0]) {
        // 未来有变化：如 "5h后转多云"
        char hb[8]; snprintf(hb, sizeof(hb), "%dh", g_wx.fc_change_hour);
        wx = draw_string_ascii(wx, 60, hb, COLOR_TEXT);               // 5h
        wx = draw_string_cn(wx, 60, TXT_AFTER, COLOR_TEXT);           // 后
        wx = draw_string_cn(wx, 60, TXT_TOW, COLOR_TEXT);             // 转
        draw_string_cn(wx, 60, weatherGlyph(g_wx.fc_change_wea), COLOR_TEXT); // 多云(白)
      } else {
        // 未来无变化：显示"今日持续晴"
        wx = draw_string_cn(wx, 60, TXT_TODAY, COLOR_TEXT);           // 今日
        wx = draw_string_cn(wx, 60, TXT_KEEP, COLOR_TEXT);            // 持续
        draw_string_cn(wx, 60, weatherGlyph(g_wx.realtime_wea), COLOR_TEXT); // 晴(白)
      }
    }
    // 大温度 16x32（右侧，上移到 y=50，与下方降雨行拉开）
    {
      char tb[8]; snprintf(tb, sizeof(tb), "%d", g_wx.tem);
        uint16_t tx = SCREEN_W - 8 - 16*strlen(tb) - 28;  // 留 "C°"
        for (uint16_t i = 0; tb[i]; i++) {
          draw_char_ascii32(tx + i*16, 50, tb[i], COLOR_VALUE, COLOR_BG);
        }
        // 单位 C°（C 用 32 字号；° 作为上标，紧贴 C 右侧、比顶部略低）
        draw_char_ascii32(tx + strlen(tb)*16 + 4, 50, 'C', COLOR_DIM, COLOR_BG);
        draw_string_cn(tx + strlen(tb)*16 + 4 + 16, 54, TXT_DEG_SYM, COLOR_DIM);
    }

    // ===== 行 2.5: 小时级降水预报（今天会不会下雨/几点到几点降雨） =====
    if (g_wx.rain_ok) {
      struct tm t;
      int curH = -1;
      if (getLocalTime(&t, 0) && t.tm_year > 100) curH = t.tm_hour;
      // 本地时间不可用：无法给出具体时刻，仅显示全天判断
      if (curH < 0) {
        if (g_wx.rain_any_today) draw_string_cn(8, 78, TXT_RAIN_TODAY, COLOR_WARN);
        else                      draw_string_cn(8, 78, TXT_RAIN_NONE, COLOR_GOOD);
      } else if (g_wx.rain_start_hour < 0) {
        // fetch 时时间不可用，仅知道全天是否有雨
        if (g_wx.rain_any_today) draw_string_cn(8, 78, TXT_RAIN_TODAY, COLOR_WARN);
        else                      draw_string_cn(8, 78, TXT_RAIN_NONE, COLOR_GOOD);
      } else if (curH >= g_wx.rain_start_hour &&
                 (g_wx.rain_end_hour < 0 || curH <= g_wx.rain_end_hour)) {
        // ===== 正在降雨（当前小时落在降雨时段内） =====
        draw_string_cn(8, 78, TXT_RAIN_NOW, COLOR_BAD);            // 正在降雨
        if (g_wx.rain_end_hour >= 0 && g_wx.rain_end_hour > curH) {
          uint16_t xr = 8 + 64;
          draw_string_cn(xr, 78, TXT_RAIN_TO, COLOR_BAD);          // 至
          char rb[8]; snprintf(rb, sizeof(rb), "%02d:00", g_wx.rain_end_hour);
          draw_string_ascii(xr + 16 + 4, 78, rb, COLOR_BAD);       // 23:00
        }
      } else if (curH < g_wx.rain_start_hour) {
        // ===== 降雨概率57% 14:00-23:00（或：降雨概率57% 14:00起雨） =====
        uint16_t xr = 8;
        draw_string_cn(xr, 78, TXT_RAIN_WORD, COLOR_WARN);         // 降雨
        xr += 32;
        draw_string_cn(xr, 78, TXT_RAIN_PROB, COLOR_WARN);         // 概率
        xr += 32;
        char pb[8]; snprintf(pb, sizeof(pb), "%d%%", g_wx.rain_start_prob);
        draw_string_ascii(xr, 78, pb, COLOR_WARN);                 // 57%
        xr += strlen(pb)*8 + 4;
        char rb[8]; snprintf(rb, sizeof(rb), "%02d:00", g_wx.rain_start_hour);
        draw_string_ascii(xr, 78, rb, COLOR_WARN);                 // 14:00
        if (g_wx.rain_end_hour > g_wx.rain_start_hour) {
          xr += strlen(rb)*8;
          draw_string_ascii(xr, 78, "-", COLOR_WARN);              // -
          xr += 8;
          char eb[8]; snprintf(eb, sizeof(eb), "%02d:00", g_wx.rain_end_hour);
          draw_string_ascii(xr, 78, eb, COLOR_WARN);               // 23:00
        } else {
          xr += strlen(rb)*8 + 4;
          draw_string_cn(xr, 78, TXT_RAIN_START, COLOR_WARN);      // 起雨
        }
      } else {
        // 当前小时已过降雨时段
        draw_string_cn(8, 78, TXT_RAIN_TODAY, COLOR_WARN);         // 今日有雨
      }
    } else {
      // 小时预报失败时降级：按当天天气类型粗略提示
      if (strstr(g_wx.wea_utf8,"\xe9\x9b\xa8") || strstr(g_wx.wea_utf8,"\xe9\x9b\xaa"))
        draw_string_cn(8, 78, TXT_RAIN_TODAY, COLOR_WARN);         // 今日有雨
      else
        draw_string_cn(8, 78, TXT_RAIN_NONE, COLOR_GOOD);          // 今日无雨
    }

    // ===== 行 3: 光照强度及变化趋势（原湿度行） =====
    drawLocalLux(100);

    // ===== 行 4: 最高 | 最低 =====
    draw_label_val_cn(8,   126, TXT_L_HIGH, g_wx.tmax, "C", TXT_DEG_SYM, COLOR_HOT);
    draw_label_val_cn(92, 126, TXT_L_LOW,  g_wx.tmin, "C", TXT_DEG_SYM, COLOR_COLD);

    // ===== 行 5: 湿度 | 明天(天气)（原风速行） =====
    draw_string_cn(8, 152, TXT_L_HUM, COLOR_DIM);
    { char tb[8]; snprintf(tb,sizeof(tb),"%d",g_wx.hum);
      draw_string_ascii(44, 152, tb, COLOR_VALUE);
      draw_string_ascii(44+strlen(tb)*8+4, 152, "%", COLOR_DIM);
    }
    draw_string_cn(92, 152, TXT_MING, COLOR_DIM);
    draw_string_ascii(92 + 16, 152, ":", COLOR_DIM);
    if (g_wx.has_tomorrow) {
      uint16_t tx = 120;
      tx = draw_string_cn(tx, 152, weatherGlyph(g_wx.tmr_wea_utf8), COLOR_TEXT);
      // 明天天气变化/持续提示：变化"晴 15h转多云"，无变化"晴,持续晴"
      tx += 2;
      tx = draw_string_ascii(tx, 152, ",", COLOR_TEXT);         // 逗号
      tx += 2;
      if (g_wx.tmr_fc_hour >= 0 && g_wx.tmr_fc_wea[0]) {
        char hb[8];
        if (g_wx.tmr_fc_hour == 0) snprintf(hb, sizeof(hb), "%d", 0);
        else snprintf(hb, sizeof(hb), "%d", g_wx.tmr_fc_hour);
        tx = draw_string_ascii(tx, 152, hb, COLOR_TEXT);        // 15
        tx = draw_string_cn(tx, 152, TXT_DIAN, COLOR_TEXT);     // 点
        tx = draw_string_cn(tx, 152, TXT_TOW, COLOR_TEXT);      // 转
        draw_string_cn(tx, 152, weatherGlyph(g_wx.tmr_fc_wea), COLOR_TEXT);  // 多云
      } else {
        tx = draw_string_cn(tx, 152, TXT_KEEP, COLOR_TEXT);     // 持续
        draw_string_cn(tx, 152, weatherGlyph(g_wx.tmr_wea_utf8), COLOR_TEXT); // 晴
      }
    } else {
      draw_string_cn(120, 152, TXT_W_UNKNOWN, COLOR_DIM);
    }

    // ===== 行 6: 时间标签 + 时间 + 日期(含年份后两位) + 星期（间距相等） =====
    struct tm t;
    if (getLocalTime(&t, 0)) {
      char tb[16]; snprintf(tb,sizeof(tb),"%02d:%02d:%02d",t.tm_hour,t.tm_min,t.tm_sec);
      draw_string_cn(8, 178, TXT_L_TIME, COLOR_DIM);    // 时间标签
      draw_string_ascii(44, 178, tb, COLOR_TEXT);       // 时间 18:05:23 (44..108)
      char db[8]; snprintf(db,sizeof(db),"%02d/%d/%d",(t.tm_year+1900)%100,t.tm_mon+1,t.tm_mday);
      draw_string_ascii(120, 178, db, COLOR_TEXT);      // 日期 26/8/13（7字符，120..176）
    } else {
      draw_string_cn(8, 178, TXT_L_TIME, COLOR_DIM);
      draw_string_ascii(44, 178, "--:--:--", COLOR_DIM);
    }
    if (g_wx.week >= 0 && g_wx.week < 7) {
      draw_string_cn(188, 178, TXT_WK[g_wx.week], COLOR_TEXT);   // 星期三 (188..236)
    } else {
      draw_string_cn(192, 178, TXT_W_UNKNOWN, COLOR_DIM);
    }

    // 分隔线（上移到 y=204）
    draw_hline(8, 204, SCREEN_W-16, COLOR_DIV);
  }

  // ===== 无线光照区域（与天气无关，始终显示） =====
  drawLightArea();

  // ===== 底部建议栏（无背景色 + 顶部分隔线） =====
  // 有 AI 建议（esp32_controller 返回）时显示 AI 建议；否则回退今/明两行
  draw_hline(8, SCREEN_H-FOOTER_H - 2, SCREEN_W-16, COLOR_DIV);
  // "今:" / "明:" 标签（x=8，建议文字紧贴其后）
  draw_string_cn(8, SCREEN_H-FOOTER_H+2, TXT_TODAY_W, COLOR_DIM);
  draw_string_ascii(8 + 16, SCREEN_H-FOOTER_H+2, ":", COLOR_DIM);
  if (aiAdviceValid) {
    AiAdvice aa = adviceFromCode(aiAdviceCode);
    draw_string_cn(8 + 24, SCREEN_H-FOOTER_H+2, aa.text, aa.color);
    // 第二行：明：+ 系统正常保持运行
    draw_string_cn(8, SCREEN_H-FOOTER_H+20, TXT_TOM_W, COLOR_DIM);
    draw_string_ascii(8 + 16, SCREEN_H-FOOTER_H+20, ":", COLOR_DIM);
    draw_string_cn(8 + 24, SCREEN_H-FOOTER_H+20, A_ADV_NORM, COLOR_DIM);
  } else {
    Advice advT = adviceToday(g_wx);
    Advice advN = adviceTomorrow(g_wx);
    // 第一行：今：建议
    draw_string_cn(8 + 24, SCREEN_H-FOOTER_H+2, advT.text, advT.color);
    // 第二行：明：建议
    draw_string_cn(8, SCREEN_H-FOOTER_H+20, TXT_TOM_W, COLOR_DIM);
    draw_string_ascii(8 + 16, SCREEN_H-FOOTER_H+20, ":", COLOR_DIM);
    draw_string_cn(8 + 24, SCREEN_H-FOOTER_H+20, advN.text, advN.color);
  }
}

/* =========================================================================
 *                12. WiFi / NTP
 * ========================================================================= */
static void connectWiFi() {
  // ---- 双模式：STA(连手机热点上网) + AP(平板直连，固定网址) ----
  if (WiFi.getMode() != WIFI_AP_STA) {
    WiFi.mode(WIFI_AP_STA);
    delay(100);
  }
  // 自带热点只启动一次，不管 STA 是否连接都保证开着
  static bool apStarted = false;
  if (!apStarted) {
    apStarted = WiFi.softAP(AP_SSID, AP_PASSWORD);
    if (apStarted) {
      Serial.printf("  自带热点已开: %s → 平板网址固定 http://192.168.4.1\n", AP_SSID);
    } else {
      Serial.println("  自带热点开启失败（不影响 WiFi 上网）");
    }
  }
  if (WiFi.status() == WL_CONNECTED) return;
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  uint8_t t = 0;
  while (WiFi.status() != WL_CONNECTED && t < 30) {
    delay(500); t++;
    if (t % 4 == 0) Serial.printf("  WiFi 连接中 %u s, status=%d\n", t / 2, WiFi.status());
  }
  if (WiFi.status() == WL_CONNECTED) {
#if FIX_IP_LAST != 0
    // ---- 网段自适应固定 IP：前 3 段跟随当前网段，仅固定主机位 ----
    IPAddress ip = WiFi.localIP();
    if (ip[3] != FIX_IP_LAST) {
      IPAddress gw     = WiFi.gatewayIP();
      IPAddress subnet = WiFi.subnetMask();
      IPAddress fixedIp(ip[0], ip[1], ip[2], FIX_IP_LAST);
      WiFi.disconnect();
      WiFi.config(fixedIp, gw, subnet, gw);   // 用网关做 DNS（跟随热点，最稳妥）
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      uint8_t t2 = 0;
      while (WiFi.status() != WL_CONNECTED && t2 < 10) { delay(500); t2++; }
      if (WiFi.status() != WL_CONNECTED) {
        // 固定 IP 重连失败（可能地址被占用），退回纯 DHCP
        Serial.println("  固定 IP 重连失败，退回 DHCP");
        WiFi.disconnect();
        WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);  // 清除静态配置，改回 DHCP
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        uint8_t t3 = 0;
        while (WiFi.status() != WL_CONNECTED && t3 < 10) { delay(500); t3++; }
      } else {
        Serial.printf("  固定 IP 生效: %s\n", WiFi.localIP().toString().c_str());
      }
    }
#endif
    Serial.printf("  WiFi OK, IP=%s\n", WiFi.localIP().toString().c_str());
    // 禁用 WiFi 省电（提高稳定性）
    WiFi.setSleep(false);
    // 注意：不再强制覆盖 DNS！
    // 部分手机热点会阻断对公共 DNS(114.114.114.114/8.8.8.8) 的查询，
    // 导致所有域名解析失败(HTTP -1)。改用热点 DHCP 下发的 DNS 最稳妥。
  } else {
    Serial.println("  WiFi 失败：请检查 SSID/密码");
  }
}

// 本地时间是否已同步（年份有效且 >2000）
static bool timeIsValid() {
  struct tm t;
  return getLocalTime(&t, 0) && t.tm_year > 100;
}

// 当前小时（0~23），时间无效返回 -1
static int currentHour() {
  struct tm t;
  if (getLocalTime(&t, 0) && t.tm_year > 100) return t.tm_hour;
  return -1;
}

static void syncTime() {
  if (WiFi.status() != WL_CONNECTED) return;
  // 用多个服务器提高成功率：国内阿里 + 腾讯 + 通用池（带 IP 备用）
  const char* servers[] = { "ntp1.aliyun.com", "ntp2.aliyun.com",
                            "cn.pool.ntp.org", "time.windows.com" };
  configTime(UTC_OFFSET_SEC, 0, servers[0], servers[1], servers[2]);
  // NTP 同步需要时间，循环等待确保时间真正就绪（否则小时降水会误判）
  struct tm t;
  for (int i = 0; i < 20; i++) {
    if (getLocalTime(&t, 500) && t.tm_year > 100) {
      Serial.printf("  NTP 时间同步成功: %04d-%02d-%02d %02d:%02d:%02d\n",
                    t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
                    t.tm_hour, t.tm_min, t.tm_sec);
      return;
    }
    delay(300);
  }
  // 失败重试：换另一组服务器再试一轮
  configTime(UTC_OFFSET_SEC, 0, servers[2], servers[3], servers[0]);
  for (int i = 0; i < 20; i++) {
    if (getLocalTime(&t, 500) && t.tm_year > 100) {
      Serial.printf("  NTP 时间同步成功(2): %04d-%02d-%02d %02d:%02d:%02d\n",
                    t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
                    t.tm_hour, t.tm_min, t.tm_sec);
      return;
    }
    delay(300);
  }
  Serial.println("  NTP 时间同步失败（降水/天气变化预测将退化为全天判断，稍后自动重试）");
}

/* =========================================================================
 *                13. setup / loop
 * ========================================================================= */
unsigned long lastWeatherMs = 0;
unsigned long lastDrawMs   = 0;

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("== 气象显示 启动 ==");

  // UART 有线链路（四个集热器光照 → esp32_controller）
  Serial1.begin(UART_LINK_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  Serial.println("UART 链路就绪 (GPIO19→TX, GPIO20←RX)");

  pinMode(PIN_BOOT, INPUT_PULLUP);

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
  lcd_init();
  Serial.println("LCD 初始化完成");
  fontSelfTest();

  // 光照数据随天气一起联网获取（见 fetchHourlyRain / fetchWeatherFor）

  // 启动画面
  lcd_fill(0, 0, SCREEN_W, SCREEN_H, COLOR_BG);
  draw_string_cn_centered(150, TXT_LOADING, COLOR_VALUE);

  Serial.printf("正在连接 WiFi: %s ...\n", WIFI_SSID);
  connectWiFi();
  if (WiFi.status() == WL_CONNECTED) {
    // 无线光照 UDP 必须在 WiFi 连接之后初始化，否则 lwIP 断言崩溃（Invalid mbox）
    udpRx.begin(UDP_PORT);
    udpReady = true;
    Serial.printf("无线光照 UDP 端口 %d 就绪\n", UDP_PORT);

    syncTime();
#if AUTO_LOCATE == 1
    Serial.println("正在定位当前城市...");
    fetchLocatedCity();
#endif
    Serial.print("获取天气: ");
    Serial.println(fetchWeatherRetry() ? "成功" : "失败");

    // Web 服务器（平板远程监控；失败不影响其他功能）
    initWebServer();
  }
  drawScreen();
  Serial.println("启动流程结束");
  lastWeatherMs = millis();
}

void loop() {
  unsigned long now = millis();

  // 无线光照接收（每秒都会处理，超时自动判离线）
  checkLightUdp();

  // UART 有线发送：每 2 秒把四个集热器光照发给 esp32_controller
  if (now - lastUartSendMs >= UART_SEND_INTERVAL_MS) {
    lastUartSendMs = now;
    sendLuxToController();
  }

  // UART 接收：esp32_controller 发回的 AI 干燥建议
  recvAdviceFromController();

  // Web 服务器：处理一个待处理请求（独立，不阻塞主循环，异常不影响其他功能）
  if (webReady) webServer.handleClient();

  // BOOT 按键：手动刷新（含重新定位）
  static bool lastBoot = HIGH;
  bool curBoot = digitalRead(PIN_BOOT);
  if (lastBoot == HIGH && curBoot == LOW) {
    Serial.println("BOOT 键按下：手动刷新");
    if (WiFi.status() != WL_CONNECTED) connectWiFi();
    if (WiFi.status() == WL_CONNECTED) {
      ensureUdpReady();
      syncTime();
#if AUTO_LOCATE == 1
      fetchLocatedCity();
#endif
      fetchWeatherRetry();
    }
    drawScreen();
    lastWeatherMs = now;
    delay(250);  // 消抖
  }
  lastBoot = curBoot;

  // 定期获取（含重新定位）
  if (now - lastWeatherMs >= WEATHER_INTERVAL_MS) {
    Serial.println("定时刷新天气...");
    if (WiFi.status() != WL_CONNECTED) connectWiFi();
    if (WiFi.status() == WL_CONNECTED) {
      ensureUdpReady();
      syncTime();
#if AUTO_LOCATE == 1
      fetchLocatedCity();
#endif
      fetchWeatherRetry();
    }
    lastWeatherMs = now;
    drawScreen();
  }

  // 如果启动时没拿到数据，每 30 秒自动补一次
  if (!g_wx.valid && now - lastWeatherMs >= 30000) {
    lastWeatherMs = now;
    if (WiFi.status() != WL_CONNECTED) connectWiFi();
    if (WiFi.status() == WL_CONNECTED) {
      ensureUdpReady();
#if AUTO_LOCATE == 1
      fetchLocatedCity();
#endif
      fetchWeatherRetry(3);
      drawScreen();
    }
  }

  // NTP 时间无效时，每 30 秒自动重试同步（成功后重算降水/天气变化）
  static unsigned long lastNtpRetryMs = 0;
  if (!timeIsValid() && WiFi.status() == WL_CONNECTED &&
      now - lastNtpRetryMs >= 30000) {
    lastNtpRetryMs = now;
    Serial.println("NTP 时间仍无效，自动重试同步...");
    syncTime();
    if (timeIsValid() && g_wx.valid) {
      // 时间刚同步成功：立即重算小时级数据
      const CityEntry* lc = g_dispCity ? g_dispCity : city;
      fetchHourlyRain(lc);
      drawScreen();
    }
  }

  // 当地光照：每 1 分钟联网刷新数据 + 重绘；失败时 15 秒快速重试
  if (now - lastLocalLuxMs >= LUX_REFRESH_MS) {
    const CityEntry* lc = g_dispCity ? g_dispCity : city;
    if (WiFi.status() == WL_CONNECTED) {
      if (fetchLocalLux(lc)) lastLocalLuxMs = now;            // 成功：进入 1 分钟周期
      else lastLocalLuxMs = now + LUX_REFRESH_MS - 45000;     // 失败：15 秒后再试
    } else {
      lastLocalLuxMs = now;
    }
    drawLocalLux(100);
  }

  // 每秒仅刷新"时间"和"光照"区域（避免整屏闪烁）
  if (now - lastDrawMs >= TIME_REFRESH_MS) {
    lastDrawMs = now;
    struct tm t;
    if (getLocalTime(&t, 0)) {
      char tb[16]; snprintf(tb,sizeof(tb),"%02d:%02d:%02d",t.tm_hour,t.tm_min,t.tm_sec);
      // 时间固定 8 字符，直接覆盖绘制（字符会画满背景色），无需先整块擦除
      draw_string_ascii(44, 178, tb, COLOR_TEXT);
    }
    // 光照值/状态变化时才局部重绘（逐节点比较，只擦数值小区域）
    static float  lastLux[LIGHT_NODES] = {-9999,-9999,-9999,-9999};
    static bool   lastOn[LIGHT_NODES]  = {false,false,false,false};
    static bool   lightInited = false;
    bool needDraw = !lightInited;
    for (int i = 0; i < LIGHT_NODES; i++) {
      float cur = (lightOnline[i] && wirelessLux[i] >= 0) ? wirelessLux[i] : -1.0;
      if (cur != lastLux[i] || lightOnline[i] != lastOn[i]) needDraw = true;
      lastLux[i] = cur; lastOn[i] = lightOnline[i];
    }
    if (needDraw) {
      drawLightDynamic();
      lightInited = true;
    }
  }

  delay(20);
}
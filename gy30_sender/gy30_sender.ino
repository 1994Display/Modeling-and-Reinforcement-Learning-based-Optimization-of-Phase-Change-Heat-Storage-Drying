/**
 * ============================================================================
 *  ESP32-C3 无线光照传感器节点（发送端）
 *  硬件：ESP32-C3-SuperMini + GY-30 (BH1750) 光照强度传感器
 *  功能：读取光照强度(lux)，通过 WiFi UDP 广播发送，供 ESP32-S3 天气屏接收显示
 *
 *  接线（GY-30 → ESP32-C3-SuperMini）：
 *    VCC → 3.3V
 *    GND → GND
 *    SDA → GPIO 8
 *    SCL → GPIO 9
 *    （若传感器模块带 ADDR 脚悬空即可，地址 0x23）
 *
 *  烧录：Arduino IDE + "ESP32C3 Dev Module"（零第三方库依赖）
 * ============================================================================
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>

/* ============================= 配置区 ============================= */
// WiFi（与 ESP32-S3 天气屏连同一个热点/路由器）
const char* WIFI_SSID     = "cdwwsbz";
const char* WIFI_PASSWORD = "blwrd233";

// ===== 节点序号（1~4，每个 C3 各烧一个不同的序号） =====
// 用于区分多路光照：数据包格式 LUX<序号>:数值，如 "LUX1:1364"
// 四个 C3 分别改成 1、2、3、4 再烧录
#define NODE_ID  1

// WiFi 发射功率（dBm）。若贴热点很近导致认证超时(原因码2)，
// 可降低功率避免干扰热点接收。可选项：
//   WIFI_POWER_19_5dBm / WIFI_POWER_13dBm / WIFI_POWER_11dBm
//   WIFI_POWER_8_5dBm / WIFI_POWER_7dBm / WIFI_POWER_5dBm
#define WIFI_TX_POWER   WIFI_POWER_8_5dBm   // 调小功率，提高贴脸场景连接稳定性

// I2C 引脚（ESP32-C3-SuperMini）
#define I2C_SDA      8
#define I2C_SCL      9
#define BH1750_ADDR  0x23   // GY-30 默认地址

// UDP 广播端口（与接收端一致）
#define UDP_PORT     8266

// 发送间隔（毫秒）
#define SEND_INTERVAL_MS  1000
/* ================================================================== */

WiFiUDP udp;
bool    bhReady = false;
uint32_t bhFailCount = 0;   // 连续读取失败次数
uint32_t bhOkCount   = 0;   // 成功读取次数
int     g_wifiFailReason = -1;   // 最近一次 WiFi 连接失败的原因码

// ---- BH1750 驱动（原生 Wire，无需库） ----
bool bh1750Init() {
  Wire.beginTransmission(BH1750_ADDR);
  Wire.write(0x01);                       // Power ON
  if (Wire.endTransmission() != 0) return false;
  delay(10);
  Wire.beginTransmission(BH1750_ADDR);
  Wire.write(0x10);                       // 连续高分辨率模式 (1lux, 120ms)
  if (Wire.endTransmission() != 0) return false;
  delay(180);
  return true;
}

float bh1750Read() {
  if (!bhReady) return -1.0;
  Wire.requestFrom(BH1750_ADDR, 2);
  if (Wire.available() < 2) return -1.0;
  uint16_t raw = (Wire.read() << 8) | Wire.read();
  return raw / 1.2;                       // 分辨率 1 lux/bit，÷1.2 为实际值
}

// WiFi 事件回调：捕获连接断开时的原因码
void onWifiDisconnected(WiFiEvent_t event, WiFiEventInfo_t info) {
  g_wifiFailReason = info.wifi_sta_disconnected.reason;
  Serial.printf("[WiFi] 连接断开, 原因码=%d\n", g_wifiFailReason);
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("======================================");
  Serial.printf("== GY-30 无线光照传感器节点 %d ==\n", NODE_ID);
  Serial.println("======================================");

  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.printf("I2C 引脚: SDA=GPIO%d, SCL=GPIO%d, 地址=0x%02X\n", I2C_SDA, I2C_SCL, BH1750_ADDR);
  bhReady = bh1750Init();
  if (bhReady) {
    Serial.println("[初始化] BH1750 检测成功 ✅");
  } else {
    Serial.println("[初始化] BH1750 检测失败 ❌");
    Serial.println("[初始化] 排查提示：");
    Serial.println("  1. 确认 VCC→3.3V, GND→GND");
    Serial.println("  2. 确认 SDA→GPIO8, SCL→GPIO9");
    Serial.println("  3. 传感器供电时模块上应有一个小灯亮");
    Serial.println("  4. 试试把 SDA/SCL 对调（部分模块丝印相反）");
  }

  // ===== 射频扫描测试（在连接之前扫描，排除连接过程的干扰）=====
  Serial.println("[WiFi] 射频扫描测试（连接前）...");
  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_TX_POWER);        // 应用调小的发射功率
  Serial.printf("[WiFi] 发射功率已设为 %d dBm\n", WiFi.getTxPower());
  int nScan = WiFi.scanNetworks();       // 阻塞扫描
  if (nScan > 0) {
    Serial.printf("[WiFi] 扫描到 %d 个网络:\n", nScan);
    for (int i = 0; i < nScan && i < 15; i++) {
      Serial.printf("  [%d] %-28s RSSI=%d dBm\n", i,
                    WiFi.SSID(i).c_str(), WiFi.RSSI(i));
    }
    bool found = false;
    for (int i = 0; i < nScan; i++)
      if (WiFi.SSID(i) == WIFI_SSID) { found = true; break; }
    Serial.printf("[WiFi] 目标热点 %s : %s\n", WIFI_SSID, found ? "已找到!" : "未找到!");
  } else {
    Serial.printf("[WiFi] 扫描到 %d 个网络（一个都扫不到 = 射频/天线硬件问题）\n", nScan);
  }
  WiFi.scanDelete();

  // 连接 WiFi（加长等待 + 状态诊断）
  WiFi.onEvent(onWifiDisconnected, WiFiEvent_t(ARDUINO_EVENT_WIFI_STA_DISCONNECTED));
  WiFi.setSleep(false);   // 禁用省电，提高连接稳定性
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("正在连接 WiFi: %s ...\n", WIFI_SSID);
  uint8_t t = 0;
  g_wifiFailReason = -1;
  while (WiFi.status() != WL_CONNECTED && t < 60) {   // 最长 30 秒
    delay(500); t++;
    if (t % 4 == 0) Serial.printf("  WiFi 连接中 %u s, status=%d\n", t / 2, WiFi.status());
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("WiFi OK, IP=%s (信号强度 %d dBm)\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());
  } else {
    Serial.printf("WiFi 失败 (status=%d), 断开原因码=%d\n", WiFi.status(), g_wifiFailReason);
    switch (g_wifiFailReason) {
      case 202: Serial.println("  -> 认证/握手超时：多半是密码错误"); break;
      case 15:  Serial.println("  -> 4次握手超时：密码错误或热点认证方式不兼容"); break;
      case 200: Serial.println("  -> 认证失败：密码错误"); break;
      case 17:  Serial.println("  -> AP 拒绝：热点连接数已满或设备被限制"); break;
      case 21:  Serial.println("  -> AP 满员：热点连接数已满"); break;
      default:  Serial.println("  -> 其它原因，见原因码表"); break;
    }
  }

  udp.begin(UDP_PORT);
  Serial.printf("UDP 端口 %d 就绪，开始广播光照数据\n", UDP_PORT);
}

void loop() {
  static unsigned long lastSend = 0;
  static unsigned long lastFailLog = 0;   // 失败日志节流
  static unsigned long lastWifiTry = 0;   // 断线自动重连节流
  if (millis() - lastSend < SEND_INTERVAL_MS) { delay(10); return; }
  lastSend = millis();

  // ---- WiFi 自动重连（每 10 秒用 reconnect，避免与驱动冲突） ----
  if (WiFi.status() != WL_CONNECTED) {
    if (millis() - lastWifiTry > 10000) {
      lastWifiTry = millis();
      Serial.println("[WiFi] 断线，尝试重连...");
      WiFi.reconnect();   // 温和重连，不强制断开
    }
    // 光照照常采集（只是不发送）
  }

  // ---- 采集光照 ----
  float lux = bh1750Read();
  if (lux < 0) {
    // 采集失败：连续失败每 5 秒打一次日志，避免刷屏
    bhFailCount++;
    if (millis() - lastFailLog > 5000) {
      lastFailLog = millis();
      if (!bhReady)
        Serial.println("[光照] 采集失败：BH1750 未初始化，请检查 VCC/GND/SDA/SCL 接线");
      else
        Serial.printf("[光照] 采集失败（I2C 无响应，连续 %u 次失败）⚠️\n", bhFailCount);
    }
    return;
  }
  bhFailCount = 0;
  bhOkCount++;
  Serial.printf("[节点%d] 采集成功 #%u: %.0f lux\n", NODE_ID, bhOkCount, lux);

  // ---- UDP 广播（带节点序号：LUX<序号>:数值） ----
  if (WiFi.status() == WL_CONNECTED) {
    char buf[32];
    snprintf(buf, sizeof(buf), "LUX%d:%.0f", NODE_ID, lux);
    IPAddress bc = WiFi.localIP();
    bc[3] = 255;
    udp.beginPacket(bc, UDP_PORT);
    udp.write((uint8_t*)buf, strlen(buf));
    udp.endPacket();
    Serial.printf("[发送] -> %s:%d (%s)\n", bc.toString().c_str(), UDP_PORT, buf);
  } else {
    Serial.println("[发送] 跳过：WiFi 未连接");
  }
}

/*
  DHT11 温湿度检测 + SSD1315 OLED 显示
  硬件：ESP32-C3-SuperPRO Mini
  传感器：DHT11
  显示屏：0.96寸 OLED（中景园 SSD1315，I2C，4针）
  接线说明：
    DHT11 VCC  -> ESP32-C3 3.3V/5V
    DHT11 GND  -> ESP32-C3 GND
    DHT11 DATA -> ESP32-C3 GPIO3
    OLED  VCC  -> ESP32-C3 3.3V/5V
    OLED  GND  -> ESP32-C3 GND
    OLED  SCL  -> ESP32-C3 GPIO9 (SCL)
    OLED  SDA  -> ESP32-C3 GPIO8 (SDA)
*/

#include <Wire.h>
#include "DHT.h"
#include "oledfont.h"

// ---------- 类型定义 ----------
typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int  u32;

// ---------- DHT11 ----------
#define DHTPIN  3
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// ---------- OLED I2C ----------
#define OLED_ADDR 0x3C
#define SDA_PIN   8
#define SCL_PIN   9

// ---------- OLED 命令/数据 ----------
#define OLED_CMD  0
#define OLED_DATA 1

// ==================== OLED 底层函数 ====================

void OLED_WR_Byte(u8 dat, u8 mode) {
  Wire.beginTransmission(OLED_ADDR);
  if (mode) Wire.write(0x40);
  else      Wire.write(0x00);
  Wire.write(dat);
  Wire.endTransmission();
}

void OLED_Set_Pos(u8 x, u8 y) {
  OLED_WR_Byte(0xb0 + y, OLED_CMD);
  OLED_WR_Byte(((x & 0xf0) >> 4) | 0x10, OLED_CMD);
  OLED_WR_Byte((x & 0x0f), OLED_CMD);
}

void OLED_Display_On(void) {
  OLED_WR_Byte(0x8D, OLED_CMD);  // SET DCDC
  OLED_WR_Byte(0x14, OLED_CMD);  // DCDC ON
  OLED_WR_Byte(0xAF, OLED_CMD);  // DISPLAY ON
}

void OLED_Display_Off(void) {
  OLED_WR_Byte(0x8D, OLED_CMD);
  OLED_WR_Byte(0x10, OLED_CMD);
  OLED_WR_Byte(0xAE, OLED_CMD);
}

void OLED_Clear(void) {
  u8 i, n;
  for (i = 0; i < 8; i++) {
    OLED_WR_Byte(0xb0 + i, OLED_CMD);
    OLED_WR_Byte(0x00, OLED_CMD);
    OLED_WR_Byte(0x10, OLED_CMD);
    for (n = 0; n < 128; n++) OLED_WR_Byte(0, OLED_DATA);
  }
}

void OLED_ColorTurn(u8 i) {
  if (!i) OLED_WR_Byte(0xA6, OLED_CMD);
  else    OLED_WR_Byte(0xA7, OLED_CMD);
}

void OLED_DisplayTurn(u8 i) {
  if (i == 0) {
    OLED_WR_Byte(0xC8, OLED_CMD);
    OLED_WR_Byte(0xA1, OLED_CMD);
  } else {
    OLED_WR_Byte(0xC0, OLED_CMD);
    OLED_WR_Byte(0xA0, OLED_CMD);
  }
}

void OLED_Init(void) {
  // ESP32: SDA, SCL 在 Wire.begin 中设定
  Wire.begin(SDA_PIN, SCL_PIN);

  OLED_WR_Byte(0xAE, OLED_CMD); // display off
  OLED_WR_Byte(0x00, OLED_CMD); // set low column
  OLED_WR_Byte(0x10, OLED_CMD); // set high column
  OLED_WR_Byte(0x40, OLED_CMD); // set start line
  OLED_WR_Byte(0x81, OLED_CMD); // contrast
  OLED_WR_Byte(0xCF, OLED_CMD);
  OLED_WR_Byte(0xA1, OLED_CMD); // segment remap
  OLED_WR_Byte(0xC8, OLED_CMD); // COM scan direction
  OLED_WR_Byte(0xA6, OLED_CMD); // normal display
  OLED_WR_Byte(0xA8, OLED_CMD); // multiplex
  OLED_WR_Byte(0x3F, OLED_CMD); // 1/64 duty
  OLED_WR_Byte(0xD3, OLED_CMD); // display offset
  OLED_WR_Byte(0x00, OLED_CMD);
  OLED_WR_Byte(0xD5, OLED_CMD); // clock divide
  OLED_WR_Byte(0x80, OLED_CMD);
  OLED_WR_Byte(0xD9, OLED_CMD); // pre-charge
  OLED_WR_Byte(0xF1, OLED_CMD);
  OLED_WR_Byte(0xDA, OLED_CMD); // COM pins
  OLED_WR_Byte(0x12, OLED_CMD);
  OLED_WR_Byte(0xDB, OLED_CMD); // VCOMH
  OLED_WR_Byte(0x40, OLED_CMD);
  OLED_WR_Byte(0x20, OLED_CMD); // addressing mode
  OLED_WR_Byte(0x02, OLED_CMD);
  OLED_WR_Byte(0x8D, OLED_CMD); // charge pump
  OLED_WR_Byte(0x14, OLED_CMD);
  OLED_WR_Byte(0xA4, OLED_CMD); // entire display off
  OLED_WR_Byte(0xA6, OLED_CMD); // normal display
  OLED_Clear();
  OLED_WR_Byte(0xAF, OLED_CMD); // display on
}

// ==================== OLED 显示字符/数字 ====================

u32 oled_pow(u8 m, u8 n) {
  u32 result = 1;
  while (n--) result *= m;
  return result;
}

// 显示一个 ASCII 字符，sizey: 8 或 16
void OLED_ShowChar(u8 x, u8 y, const u8 chr, u8 sizey) {
  u8 c = 0, sizex = sizey / 2, temp;
  u16 i = 0, size1;
  if (sizey == 8) size1 = 6;
  else size1 = (sizey / 8 + ((sizey % 8) ? 1 : 0)) * (sizey / 2);
  c = chr - ' ';
  OLED_Set_Pos(x, y);
  for (i = 0; i < size1; i++) {
    if (i % sizex == 0 && sizey != 8) OLED_Set_Pos(x, y++);
    if (sizey == 8) {
      temp = pgm_read_byte(&asc2_0806[c][i]);
      OLED_WR_Byte(temp, OLED_DATA);
    } else if (sizey == 16) {
      temp = pgm_read_byte(&asc2_1608[c][i]);
      OLED_WR_Byte(temp, OLED_DATA);
    } else return;
  }
}

// 显示字符串
void OLED_ShowString(u8 x, u8 y, const char *chr, u8 sizey) {
  u8 j = 0;
  while (chr[j] != '\0') {
    OLED_ShowChar(x, y, chr[j++], sizey);
    if (sizey == 8) x += 6;
    else x += sizey / 2;
  }
}

// 显示数字
void OLED_ShowNum(u8 x, u8 y, u32 num, u8 len, u8 sizey) {
  u8 t, temp, m = 0;
  u8 enshow = 0;
  if (sizey == 8) m = 2;
  for (t = 0; t < len; t++) {
    temp = (num / oled_pow(10, len - t - 1)) % 10;
    if (enshow == 0 && t < (len - 1)) {
      if (temp == 0) {
        OLED_ShowChar(x + (sizey / 2 + m) * t, y, ' ', sizey);
        continue;
      } else enshow = 1;
    }
    OLED_ShowChar(x + (sizey / 2 + m) * t, y, temp + '0', sizey);
  }
}

// 显示 ℃（° 用自绘圆点阵，C 用标准字库字符，避免方向/颠倒问题）
void OLED_ShowDegreeC(u8 x, u8 y) {
  static const u8 deg[] PROGMEM = {
    0x00, 0x1C, 0x22, 0x22, 0x22, 0x1C, 0x00, 0x00
  };
  for (u8 i = 0; i < 8; i++) {
    if (i % 8 == 0) OLED_Set_Pos(x, y);
    OLED_WR_Byte(pgm_read_byte(&deg[i]), OLED_DATA);
  }
  OLED_ShowChar(x + 8, y, 'C', 16);
}

// ==================== I2C 扫描器 ====================

void I2C_Scan() {
  Serial.println("I2C 扫描中...");
  byte error, address;
  int devices = 0;
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("发现 I2C 设备，地址: 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      devices++;
    }
    else if (error == 4) {
      Serial.print("未知错误，地址: 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (devices == 0)
    Serial.println("未找到任何 I2C 设备！请检查接线");
  else
    Serial.println("扫描完成");
}

// ==================== 主程序 ====================

void setup() {
  Serial.begin(115200);
  delay(1000);  // 等待串口稳定

  // --- I2C 扫描（诊断用）---
  Wire.begin(SDA_PIN, SCL_PIN);
  I2C_Scan();

  // 如果 OLED 地址是 0x3D，请把上面 #define OLED_ADDR 改为 0x3D

  // 初始化 OLED
  OLED_Init();
  OLED_ColorTurn(0);    // 0:正常 1:反色
  OLED_DisplayTurn(0);  // 0:正常 1:翻转180
  OLED_Clear();

  // 启动画面
  OLED_ShowString(32, 1, "DHT11", 16);
  OLED_ShowString(10, 4, "Temp & Hum", 16);
  delay(1500);
  OLED_Clear();

  // 初始化 DHT11
  dht.begin();
  Serial.println("DHT11 温湿度检测启动");
}

void loop() {
  delay(2000);

  float h = dht.readHumidity() - 6.0;      // 湿度在传感器基础上减 6
  float t = dht.readTemperature() + 6.0;   // 温度在传感器基础上加 6

  if (isnan(h) || isnan(t)) {
    Serial.println("错误：无法从 DHT11 读取数据！");
    OLED_Clear();
    OLED_ShowString(25, 3, "ERROR!", 16);
    return;
  }

  // --- 串口 ---
  Serial.print("温度: ");
  Serial.print(t, 1);
  Serial.print(" C  湿度: ");
  Serial.print(h, 1);
  Serial.println(" %");

  // --- OLED 显示 ---
  OLED_Clear();

  // 温度：大号居中显示 (8x16 字体)
  OLED_ShowString(10, 0, "Temp & Hum", 8);
  // 温度数值
  int t_int = (int)t;
  int t_dec = (int)(t * 10) % 10;
  // 整数部分
  OLED_ShowNum(20, 2, t_int, 2, 16);
  OLED_ShowString(44, 2, ".", 16);
  OLED_ShowNum(52, 2, t_dec, 1, 16);
  // ℃
  OLED_ShowDegreeC(68, 2);

  // 湿度（同样大字体）
  int h_int = (int)h;
  int h_dec = (int)(h * 10) % 10;
  OLED_ShowNum(20, 5, h_int, 2, 16);
  OLED_ShowString(44, 5, ".", 16);
  OLED_ShowNum(52, 5, h_dec, 1, 16);
  // %
  OLED_ShowString(68, 5, "%", 16);
}

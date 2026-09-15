# -*- coding: utf-8 -*-
"""生成 硬件元件与接口总览图 PNG（只含 元件名称/型号 + 接口，不含功能与数量）"""
from PIL import Image, ImageDraw, ImageFont
import os

W, H = 2200, 3200
IMG = Image.new("RGB", (W, H), "white")
D = ImageDraw.Draw(IMG)

# ---- 字体 ----
FONT_PATH = None
for f in ["C:/Windows/Fonts/msyh.ttc", "C:/Windows/Fonts/simhei.ttf",
          "C:/Windows/Fonts/simsun.ttc"]:
    if os.path.exists(f):
        FONT_PATH = f
        break

def font(sz):
    if FONT_PATH:
        try:
            return ImageFont.truetype(FONT_PATH, sz)
        except Exception:
            pass
    return ImageFont.load_default()

F_TITLE = font(56)
F_SUB   = font(30)
F_ZONE  = font(34)
F_HEAD  = font(30)
F_ROW   = font(27)
F_NOTE  = font(25)

# ---- 颜色 ----
C_TXT   = (30, 30, 30)
C_LINE  = (175, 185, 195)
C_ALT   = (240, 244, 249)

# 四块板主题色（深/浅）
ZONES = [
    # (标题, 深色, 浅色)
    ("一、主控制器板  ESP32-S3-N16R8  （esp32_controller.ino）", (15, 76, 129), (226, 237, 248)),
    ("二、天气屏板  ESP32-S3-N16R8  （weather_display.ino）",   (0, 112, 96),  (226, 244, 240)),
    ("三、无线光照节点  ESP32-C3-SuperMini  （gy30_sender.ino）",(176, 96, 0),  (252, 239, 222)),
    ("四、温湿度显示节点  ESP32-C3-SuperPRO Mini  （dht11_sensor.ino）", (91, 66, 150), (239, 234, 250)),
]

x0, x1 = 20, W - 20
colw = [760, 1400]          # 元件名称(型号) / 接口
PADX = 16
LINE_H = 34
MIN_RH = 56
y = 0

def wrap_text(txt, maxw, f):
    if not txt:
        return [""]
    res, cur = [], ""
    for ch in txt:
        t = cur + ch
        if D.textlength(t, font=f) > maxw and cur:
            res.append(cur)
            cur = ch
        else:
            cur = t
    if cur:
        res.append(cur)
    return res

def draw_row(y, cells, fill=None, zc=None):
    lines = [wrap_text(txt, colw[i] - PADX * 2, F_ROW) for i, txt in enumerate(cells)]
    n = max(len(x) for x in lines)
    h = max(MIN_RH, n * LINE_H + 16)
    if fill:
        D.rectangle([x0, y, x1, y + h], fill=fill)
    cx = x0
    for i, txt in enumerate(cells):
        for j, ln in enumerate(lines[i]):
            D.text((cx + PADX, y + 8 + j * LINE_H), ln, font=F_ROW, fill=C_TXT)
        cx += colw[i]
    D.line([x0, y, x1, y], fill=C_LINE)
    for i in range(1):
        xx = x0 + sum(colw[:i + 1])
        D.line([xx, y, xx, y + h], fill=C_LINE)
    return y + h

def header(y, title, deep):
    D.rectangle([x0, y, x1, y + 50], fill=deep)
    D.text((x0 + PADX, y + 9), title, font=F_HEAD, fill=(255, 255, 255))
    return y + 50

def zone(y, title, deep):
    D.rectangle([x0, y, x1, y + 56], fill=deep)
    D.text((x0 + PADX, y + 10), title, font=F_ZONE, fill=(255, 255, 255))
    return y + 56

def group(y, title, light):
    """小分组标题（元件类别）"""
    D.rectangle([x0, y, x1, y + 36], fill=light)
    D.text((x0 + PADX, y + 5), title, font=F_NOTE, fill=(60, 60, 60))
    return y + 36

# ---- 标题 ----
D.text((x0, 24), "干燥系统硬件元件与接口总览", font=F_TITLE, fill=(10, 60, 120))
D.text((x0, 104), "说明：仅列出各板卡的元件名称/型号与接口（功能、数量从略）", font=F_SUB, fill=(110, 110, 110))
y = 158

# ================= 元件数据（名称(型号) | 接口） =================
ctrl = [
    ("【传感元件】", [
        ("DHT11 温湿度传感器", "DATA→GPIO4；VCC 3.3V；GND 共地"),
        ("DS18B20 水温传感器（防水探头）", "DQ→GPIO18（4.7kΩ 上拉）；VCC 3.3V"),
        ("BH1750 光照传感器", "SDA→GPIO40；SCL→GPIO41；ADDR→GND；VCC 3.3V"),
        ("碰撞开关模块（YL-99）", "OUT→GPIO2；VCC 3.3V（无碰撞HIGH/有碰撞LOW）"),
        ("NPN 接近开关（常开）", "信号线→GPIO42；棕线→12V；蓝线→GND（共地）"),
        ("导轨碰撞开关1（出风口原点）", "OUT→GPIO38（NC，触发LOW）"),
        ("导轨碰撞开关2（出风口终点）", "OUT→GPIO37（NC，触发LOW）"),
        ("导轨碰撞开关3（进风口终点）", "OUT→GPIO47（NC，触发LOW）"),
        ("导轨碰撞开关4（进风口原点）", "OUT→GPIO48（NC，触发LOW）"),
    ]),
    ("【执行机构】", [
        ("主风机（维可思 MOSFET 驱动模块）", "SIG→GPIO8；VIN+→12V；VIN-→风机+；风机-→GND"),
        ("同步风机（CS25N06 MOSFET 驱动模块）", "SIG→GPIO21（与风机1同步）；VIN→12V"),
        ("水泵", "PWM→GPIO6；VCC 5V；GND"),
        ("加热片（MOS 开关模块，高电平导通）", "IO→GPIO9；DC+/DC-→电源；OUT+/OUT-→加热片"),
        ("风门舵机（MG90S）", "PCA9685 通道 CH4；V+ 外接 5~6V"),
        ("排粮舵机（MG90S）", "PCA9685 通道 CH5；V+ 外接 5~6V"),
        ("进风口导轨舵机（连续旋转，ESP32Servo 直驱）", "信号→GPIO7；VCC 5V；GND"),
        ("出风口导轨舵机（连续旋转，ESP32Servo 直驱）", "信号→GPIO10；VCC 5V；GND"),
        ("转盘直流减速电机", "接 IBT-2 驱动板 M+/M- 端子"),
    ]),
    ("【驱动板卡】", [
        ("PCA9685 舵机驱动板（16 通道 12bit）", "SDA→GPIO40；SCL→GPIO41；VCC 5V；GND"),
        ("IBT-2 电机驱动板（BTS7960 双半桥）", "RPWM→GPIO3；LPWM→GPIO1；R_EN/L_EN→5V；B+/B-→12V；GND 共地"),
    ]),
    ("【显示 / 通信 / 电源】", [
        ("ILI9341 触摸液晶屏（240×320）", "CS→GPIO5；DC→GPIO16；RST→GPIO17；MOSI→GPIO11；SCLK→GPIO12；MISO→GPIO13；背光"),
        ("UART 通信口（与天气屏交叉）", "TX→GPIO19（接天气屏RX）；RX→GPIO20（接天气屏TX）"),
        ("WiFi 无线（接收光照 UDP）", "端口 8266（收 4 路光照节点广播）"),
        ("系统供电", "12V / 5V / 3.3V，全部 GND 共地"),
    ]),
]

disp = [
    ("ILI9341 TFT 液晶模块（240×320，2.8″）", "MOSI→GPIO11；MISO→GPIO13；SCK→GPIO12；CS→GPIO10；DC→GPIO9；RST→GPIO14；BLK→GPIO21"),
    ("GT24L24A2W16 汉字字库芯片（GBK 点阵）", "SPI 数据/时钟与液晶复用；CS 与液晶复用 GPIO10（高=字库/低=液晶）"),
    ("BOOT 按键（手动刷新/重新定位）", "GPIO0（INPUT_PULLUP）"),
    ("UART 通信口（与主控制器交叉）", "TX→GPIO19（接主控RX）；RX→GPIO20（接主控TX）；GND 共地"),
    ("WiFi 无线（STA 联网 + AP 备用热点）", "STA：连路由，平板访问 http://10.255.19.200；AP：热点 GymDisplay，http://192.168.4.1"),
    ("供电", "5V / 3.3V，与主控制器共地"),
]

light = [
    ("ESP32-C3-SuperMini 主控板", "板载 WiFi 天线；USB-C 供电"),
    ("GY-30 光照传感器模块（BH1750，地址 0x23）", "SDA→GPIO8；SCL→GPIO9；VCC 3.3V；ADDR 悬空"),
    ("UDP 广播（发送给天气屏）", "WiFi UDP 端口 8266；格式 LUX<序号>:数值"),
]

dht = [
    ("ESP32-C3-SuperPRO Mini 主控板", "板载 WiFi；USB 供电"),
    ("DHT11 温湿度传感器", "DATA→GPIO3；VCC 3.3V/5V"),
    ("OLED 显示屏（中景园 SSD1315，0.96″，I2C）", "SDA→GPIO8；SCL→GPIO9；地址 0x3C"),
]

# ================= 绘制主控制器 =================
deep, light_c = ZONES[0][1], ZONES[0][2]
y = zone(y, ZONES[0][0], deep)
for gtitle, rows in ctrl:
    y = group(y, gtitle, light_c)
    y = header(y, "元件名称（型号） | 接口", deep)
    for i, r in enumerate(rows):
        y = draw_row(y, list(r), C_ALT if i % 2 else None)
    y += 12

# ================= 天气屏 =================
deep, light_c = ZONES[1][1], ZONES[1][2]
y = zone(y, ZONES[1][0], deep)
y = header(y, "元件名称（型号） | 接口", deep)
for i, r in enumerate(disp):
    y = draw_row(y, list(r), C_ALT if i % 2 else None)

# ================= 光照节点 =================
y += 12
deep, light_c = ZONES[2][1], ZONES[2][2]
y = zone(y, ZONES[2][0], deep)
y = header(y, "元件名称（型号） | 接口", deep)
for i, r in enumerate(light):
    y = draw_row(y, list(r), C_ALT if i % 2 else None)

# ================= 温湿度节点 =================
y += 12
deep, light_c = ZONES[3][1], ZONES[3][2]
y = zone(y, ZONES[3][0], deep)
y = header(y, "元件名称（型号） | 接口", deep)
for i, r in enumerate(dht):
    y = draw_row(y, list(r), C_ALT if i % 2 else None)

y += 20
D.text((x0, y), "注：同一节点板卡固件相同，仅节点序号不同，图中均只列一份元件。", font=F_NOTE, fill=(120, 120, 120))

IMG = IMG.crop((0, 0, W, min(y + 60, H)))
IMG.save("硬件元件与接口总览.png", "PNG")
print("saved: 硬件元件与接口总览.png")

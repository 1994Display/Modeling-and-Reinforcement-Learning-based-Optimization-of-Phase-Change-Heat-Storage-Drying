# -*- coding: utf-8 -*-
"""生成干燥系统全部元件接线表 PNG（自动换行版）"""
from PIL import Image, ImageDraw, ImageFont
import os

W, H = 2400, 2600
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

F_TITLE = font(58)
F_ZONE  = font(38)
F_HEAD  = font(33)
F_ROW   = font(29)
F_NOTE  = font(27)

# ---- 颜色 ----
C_HEAD  = (13, 110, 253)
C_HEADT = (255, 255, 255)
C_ZONE  = (15, 76, 129)
C_ZONET = (255, 255, 255)
C_LINE  = (180, 190, 200)
C_ALT   = (238, 243, 249)
C_TXT   = (30, 30, 30)
C_GRAY  = (110, 110, 110)

x0, x1 = 20, W - 20
colw = [500, 950, 950]   # 元件 / 接线、引脚 / 说明
PADX = 16
LINE_H = 36
MIN_RH = 58

def wrap_text(txt, maxw):
    """按像素宽度把文字拆成多行"""
    if not txt:
        return [""]
    res, cur = [], ""
    for ch in txt:
        t = cur + ch
        if D.textlength(t, font=F_ROW) > maxw and cur:
            res.append(cur)
            cur = ch
        else:
            cur = t
    if cur:
        res.append(cur)
    return res

def draw_row(y, cells, fill=None):
    """画一行（自动换行 + 自适应行高），返回行底 y"""
    lines = [wrap_text(txt, colw[i] - PADX * 2) for i, txt in enumerate(cells)]
    n = max(len(x) for x in lines)
    h = max(MIN_RH, n * LINE_H + 18)
    if fill:
        D.rectangle([x0, y, x1, y + h], fill=fill)
    cx = x0
    for i, txt in enumerate(cells):
        for j, ln in enumerate(lines[i]):
            D.text((cx + PADX, y + 10 + j * LINE_H), ln, font=F_ROW, fill=C_TXT)
        cx += colw[i]
    D.line([x0, y, x1, y], fill=C_LINE)
    for i in range(2):
        xx = x0 + sum(colw[:i + 1])
        D.line([xx, y, xx, y + h], fill=C_LINE)
    return y + h

def header(y, title):
    D.rectangle([x0, y, x1, y + 54], fill=C_HEAD)
    D.text((x0 + 16, y + 10), title, font=F_HEAD, fill=C_HEADT)
    for i in range(2):
        xx = x0 + sum(colw[:i + 1])
        D.line([xx, y, xx, y + 54], fill=C_HEADT)
    return y + 54

def zone(y, title):
    D.rectangle([x0, y, x1, y + 58], fill=C_ZONE)
    D.text((x0 + 16, y + 10), title, font=F_ZONE, fill=C_ZONET)
    return y + 58

# ---- 标题 ----
D.text((W // 2 - 280, 24), "干燥系统全部元件接线表", font=F_TITLE, fill=(10, 60, 120))
D.text((W // 2 - 260, 96), "ESP32-S3 控制器 + 天气屏   ·   2026-08-25", font=F_NOTE, fill=C_GRAY)

y = 150

# ================= 控制器 =================
y = zone(y, "一、控制器 ESP32-S3-N16R8（esp32_controller）")
y = header(y, "元件 | 接线 / 引脚 | 说明")

ctrl_rows = [
    ("DHT11 温湿度", "DATA → GPIO4；VCC → 3.3V；GND", "环境温湿度"),
    ("DS18B20 水温", "DQ → GPIO18；VCC → 3.3V；GND（建议 4.7k 上拉）", "干燥仓温度"),
    ("风机1", "PWM → GPIO8（维可思 MOSFET 驱动）", "主风机"),
    ("风机2", "PWM → GPIO21（CS25N06 MOSFET，与风机1 同步）", "同步风机"),
    ("水泵", "PWM → GPIO6", "储热循环泵"),
    ("加热片", "IO → GPIO9（高电平导通）", "电加热"),
    ("风门舵机", "PCA9685 通道 CH4", "风门 0°~90°"),
    ("排粮舵机", "PCA9685 通道 CH5", "排粮放料"),
    ("进风口导轨舵机", "信号 → GPIO7；VCC → 5V；GND", "进风口导轨"),
    ("出风口导轨舵机", "信号 → GPIO10；VCC → 5V；GND", "出风口导轨"),
    ("PCA9685 舵机板", "SDA → GPIO40；SCL → GPIO41；VCC → 5V；GND", "舵机驱动（I2C）"),
    ("BH1750 光照", "SDA → GPIO40；SCL → GPIO41；ADDR → GND；VCC → 3.3V", "环境光照（与 PCA9685 共用 I2C）"),
    ("碰撞开关", "OUT → GPIO2（无碰撞 HIGH / 有碰撞 LOW）", "位置检测"),
    ("NPN 接近开关", "白(信号) → GPIO42；棕 → 12V；蓝 → GND（共地）", "转盘电机定位"),
    ("直流电机（转盘，IBT-2）",
     "RPWM→GPIO3；LPWM→GPIO1；R_EN/L_EN→5V；VCC→5V；GND→ESP32 GND（共地）；"
     "B+→12V；B-→12V GND；M+/M-→电机两根线",
     "转盘电机（BTS7960 双PWM 驱动）"),
    ("导轨开关1 出风口原点", "OUT → GPIO38（NC，触发 LOW）", "出风原点"),
    ("导轨开关2 出风口终点", "OUT → GPIO37（NC，触发 LOW）", "出风终点"),
    ("导轨开关3 进风口终点", "OUT → GPIO47（NC，触发 LOW）", "进风终点"),
    ("导轨开关4 进风口原点", "OUT → GPIO48（NC，触发 LOW）", "进风原点"),
    ("LCD ILI9341 触摸屏", "CS=5, DC=16, RST=17, MOSI=11, SCLK=12, MISO=13, 背光", "控制屏（SPI）"),
    ("UART TX", "GPIO19 → 天气屏 GPIO20", "与天气屏通信"),
    ("UART RX", "GPIO20 ← 天气屏 GPIO19", ""),
    ("无线光照节点", "UDP 端口 8266（WiFi）", "4 个 C3 节点实时光照"),
    ("电源", "12V / 5V / 3.3V，所有 GND 共地", "系统供电"),
]
for i, row in enumerate(ctrl_rows):
    fill = C_ALT if i % 2 else None
    y = draw_row(y, row, fill)

# ================= 天气屏 =================
y += 18
y = zone(y, "二、天气屏 ESP32-S3-N16R8（weather_display）")
y = header(y, "元件 | 接线 / 引脚 | 说明")

disp_rows = [
    ("LCD ILI9341 + 字库", "MOSI=11, MISO=13, SCK=12, CS=10, DC=9, RST=14, BLK=21", "天气显示（SPI）"),
    ("BOOT 按键", "GPIO0（按下刷新）", "手动刷新/重新定位"),
    ("UART TX", "GPIO19 → 控制器 GPIO20", "与控制器通信"),
    ("UART RX", "GPIO20 ← 控制器 GPIO19", ""),
    ("WiFi 无线", "连接路由/热点；平板访问 http://10.255.19.200", "天气/光照/平板页面"),
    ("热点备用", "自带热点 GymDisplay，平板用 http://192.168.4.1", "路由器不可用时"),
    ("电源", "5V / 3.3V，与控制器共地", "供电"),
]
for i, row in enumerate(disp_rows):
    fill = C_ALT if i % 2 else None
    y = draw_row(y, row, fill)

D.line([x0, y, x1, y], fill=C_LINE)
y += 12
notes = [
    "注：1) 接近开关已由 GPIO39 改接 GPIO42（GPIO39 带 SUB-SPI 复用，不可靠）。",
    "    2) 电机定位依赖 NPN 接近开关信号（无金属=HIGH，检测到金属=LOW）。",
    "    3) 无线光照节点为独立 C3 板，通过 UDP 上报。",
    "    4) 两块板 UART 交叉：控制器 TX(19)→天气屏 RX(20)；天气屏 TX(19)→控制器 RX(20)。",
    "    5) IBT-2 直流电机：R_EN/L_EN 必须接 5V 高电平，GND 必须与 ESP32 共地。",
]
for n in notes:
    D.text((x0 + 8, y + 4), n, font=F_NOTE, fill=C_GRAY)
    y += 40

out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "接线表.png")
IMG = IMG.crop((0, 0, W, y + 10))
IMG.save(out, "PNG")
print("saved:", out)

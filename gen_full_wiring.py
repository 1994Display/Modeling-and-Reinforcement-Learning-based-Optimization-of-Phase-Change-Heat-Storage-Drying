# -*- coding: utf-8 -*-
"""
生成《粮食干燥系统全元器件接线图》PNG
覆盖：主控制器 ESP32-S3 + 天气屏 ESP32-S3 + 光照节点 ESP32-C3(×4) + 温湿度节点 ESP32-C3
数据来源：四个 .ino 固件的引脚定义与头部接线注释（v34.x，BTS7960/IBT-2 版）
"""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch
import matplotlib.patches as mpatches

plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

fig, ax = plt.subplots(figsize=(26, 19))
ax.set_xlim(0, 26)
ax.set_ylim(0, 19)
ax.axis('off')
ax.set_facecolor('#f7f8fa')
fig.patch.set_facecolor('#f7f8fa')

# ── 颜色 ──
C_MAIN   = '#34495e'   # 主控
C_SENS   = '#27ae60'   # 传感器
C_ACT    = '#2980b9'   # 执行器
C_I2C    = '#e67e22'   # I2C 器件
C_SPI    = '#8e44ad'   # SPI 器件
C_UART   = '#16a085'   # UART
C_POWER  = '#c0392b'   # 电源
C_NODE   = '#6c3483'   # 从板
C_WIRE_IN  = '#27ae60'
C_WIRE_OUT = '#2980b9'
C_WIRE_I2C = '#e67e22'
C_WIRE_SPI = '#8e44ad'
C_WIRE_UART= '#16a085'
C_WIRE_PWR = '#c0392b'
C_WIRE_UDP = '#95a5a6'
C_TXT    = '#2c3e50'
C_SUB    = '#7f8c8d'

def box(x, y, w, h, color, title, sub=None, tc='white', fs=8.5, subfs=6.3):
    b = FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.12",
                       facecolor=color, edgecolor=color, linewidth=1.4)
    ax.add_patch(b)
    if sub:
        ax.text(x + w/2, y + h*0.64, title, ha='center', va='center',
                color=tc, fontsize=fs, fontweight='bold')
        ax.text(x + w/2, y + h*0.26, sub, ha='center', va='center',
                color=tc, fontsize=subfs)
    else:
        ax.text(x + w/2, y + h/2, title, ha='center', va='center',
                color=tc, fontsize=fs, fontweight='bold')

def wire(x1, y1, x2, y2, color, lw=1.4, ls='-', label=None, fs=6):
    ax.plot([x1, x2], [y1, y2], color=color, linewidth=lw, linestyle=ls, zorder=0)
    if label:
        ax.text((x1+x2)/2, (y1+y2)/2 + 0.05, label, ha='center', va='bottom',
                fontsize=fs, color=color, style='italic',
                bbox=dict(boxstyle='round,pad=0.08', facecolor='white',
                          edgecolor=color, linewidth=0.4, alpha=0.9))

# ============================================================
# 标题
# ============================================================
ax.text(13, 18.55, '粮食干燥系统 —— 全元器件接线图', ha='center', va='center',
        fontsize=20, fontweight='bold', color='#1a252f')
ax.text(13, 18.0, '四块板卡 · ESP32-S3 主控制器 + 天气屏 · ESP32-C3 光照节点(×4) + 温湿度节点', ha='center',
        fontsize=10, color=C_SUB)

# ============================================================
# 主控制器 ESP32-S3（中央大框）
# ============================================================
box(9.0, 10.0, 8.0, 4.0, C_MAIN, '主控制器  ESP32-S3-N16R8', 'esp32_controller.ino', fs=11, subfs=7)
ax.text(9.25, 13.45, '传感输入:', fontsize=6.8, color='#f1c40f', fontweight='bold')
ax.text(9.25, 13.05, '  GPIO4 DHT11   GPIO18 DS18B20\n'
                     '  GPIO40/41 I2C(BH1750+PCA9685)', fontsize=6.4, color='white')
ax.text(9.25, 12.15, '执行输出:', fontsize=6.8, color='#f1c40f', fontweight='bold')
ax.text(9.25, 11.6, '  GPIO8 风机1  GPIO21 风机2  GPIO6 水泵\n'
                    '  GPIO9 加热片  GPIO3/1 IBT-2电机\n'
                    '  GPIO7 进导轨  GPIO10 出导轨', fontsize=6.4, color='white')
ax.text(13.9, 12.15, '开关/通信:', fontsize=6.8, color='#f1c40f', fontweight='bold')
ax.text(13.9, 11.6, '  GPIO2 碰撞  GPIO42 接近\n'
                    '  GPIO38/37/47/48 导轨限位\n'
                    '  SPI(11/12/13/5/16/17) ILI9341\n'
                    '  UART TX19/RX20', fontsize=6.4, color='white')

# ============================================================
# 顶部：ILI9341 触摸屏 + PCA9685 舵机板
# ============================================================
box(9.0, 14.6, 3.9, 1.0, C_SPI, 'ILI9341 触摸屏', 'SPI: CS5 DC16 RST17\nMOSI11 SCLK12 MISO13', fs=7.5, subfs=5.6)
box(13.1, 14.6, 3.9, 1.0, C_I2C, 'PCA9685 舵机板', 'I2C 0x40  SDA40 SCL41\nCH4风门 CH5排粮', fs=7.5, subfs=5.6)
wire(10.95, 14.6, 10.95, 14.0, C_WIRE_SPI, lw=1.5, label='SPI')
wire(15.05, 14.6, 15.05, 14.0, C_WIRE_I2C, lw=1.5, label='I2C')

# ============================================================
# 左列：传感器（6个）
# ============================================================
sens = [
    ('DHT11 温湿度传感器', 'DATA→GPIO4 · VCC 3.3V', C_SENS, 13.30),
    ('DS18B20 水温传感器', 'DQ→GPIO18(4.7kΩ上拉)', C_SENS, 12.45),
    ('BH1750 光照传感器', 'SDA→GPIO40 SCL→GPIO41', C_I2C, 11.60),
    ('碰撞开关 (YL-99)', 'OUT→GPIO2', C_SENS, 10.75),
    ('NPN 接近开关', '信号→GPIO42 · 棕12V 蓝GND', C_SENS, 9.90),
    ('导轨碰撞开关 ×4', 'GPIO38/37/47/48', C_SENS, 9.05),
]
for name, sub, clr, yy in sens:
    box(0.6, yy, 7.6, 0.78, clr, name, sub, fs=7.6, subfs=5.8)

# 传感器连线
wire(8.2, 13.69, 9.0, 13.69, C_WIRE_IN, label='GPIO4')
wire(8.2, 12.84, 9.0, 12.84, C_WIRE_IN, label='GPIO18')
wire(8.2, 11.99, 9.0, 11.99, C_WIRE_I2C, label='I2C')
wire(8.2, 11.14, 9.0, 11.14, C_WIRE_IN, label='GPIO2')
wire(8.2, 10.29, 9.0, 10.29, C_WIRE_IN, label='GPIO42')
wire(8.2, 9.44, 9.0, 9.44, C_WIRE_IN, label='SW1-4')

# ============================================================
# 右列：执行器（9个）
# ============================================================
acts = [
    ('风机1 (维可思 MOS)', 'SIG→GPIO8 · 12V', C_ACT, 13.35),
    ('风机2 (CS25N06 MOS)', 'SIG→GPIO21 · 与风机1同步', C_ACT, 12.55),
    ('水泵', 'SIG→GPIO6 · 5V', C_ACT, 11.75),
    ('加热片 (MOS)', 'IO→GPIO9 · HIGH=加热', C_ACT, 10.95),
    ('转盘直流电机 (IBT-2)', 'RPWM→GPIO3 LPWM→GPIO1\nR_EN/L_EN→5V B+→12V', C_ACT, 10.15),
    ('进气管舵机 (连续旋转)', 'SIG→GPIO7 · 5V', C_ACT, 9.35),
    ('出气管舵机 (连续旋转)', 'SIG→GPIO10 · 5V', C_ACT, 8.55),
    ('风门舵机 MG90S', 'PCA9685 CH4 · 0~90°', C_I2C, 7.75),
    ('排粮舵机 MG90S', 'PCA9685 CH5 · 0~90°', C_I2C, 6.95),
]
for name, sub, clr, yy in acts:
    box(17.6, yy, 7.6, 0.78, clr, name, sub, fs=7.2, subfs=5.6)

wire(17.6, 13.74, 17.0, 13.74, C_WIRE_OUT, label='GPIO8')
wire(17.6, 12.94, 17.0, 12.94, C_WIRE_OUT, label='GPIO21')
wire(17.6, 12.14, 17.0, 12.14, C_WIRE_OUT, label='GPIO6')
wire(17.6, 11.34, 17.0, 11.34, C_WIRE_OUT, label='GPIO9')
wire(17.6, 10.54, 17.0, 10.54, C_WIRE_OUT, label='GPIO3/1')
wire(17.6, 9.74, 17.0, 9.74, C_WIRE_OUT, label='GPIO7')
wire(17.6, 8.94, 17.0, 8.94, C_WIRE_OUT, label='GPIO10')
wire(17.6, 8.14, 17.0, 8.14, C_WIRE_I2C, label='CH4')
wire(17.6, 7.34, 17.0, 7.34, C_WIRE_I2C, label='CH5')

# ============================================================
# 电源（主控区左下/右下）
# ============================================================
box(0.6, 6.6, 3.6, 1.0, C_POWER, '12V 电源', '风机 / IBT-2 电机', fs=7.5, subfs=5.6)
box(4.4, 6.6, 3.6, 1.0, C_POWER, '5V 电源', '水泵 / 舵机 / PCA9685', fs=7.5, subfs=5.6)
box(0.6, 5.4, 3.6, 1.0, C_POWER, '3.3V', 'DHT11/DS18B20/BH1750', fs=7.5, subfs=5.6)
box(4.4, 5.4, 3.6, 1.0, C_POWER, 'GND', '全系统共地', fs=7.5, subfs=5.6)

# ============================================================
# 下半区：三块从板
# ============================================================
# 天气屏（中央下）
box(9.0, 3.6, 8.0, 3.0, C_NODE, '天气屏  ESP32-S3-N16R8', 'weather_display.ino', fs=9, subfs=6.5)
ax.text(9.25, 6.05, 'ILI9341+字库: MOSI11 MISO13 SCK12\n'
                    '  CS10 DC9 RST14 BLK21 (CS复用字库)\n'
                    'BOOT按键 GPIO0', fontsize=6.2, color='white')
ax.text(9.25, 4.75, 'UART: TX19→主控RX  RX20←主控TX\n'
                    'WiFi: 收4路光照UDP8266 + 拉天气', fontsize=6.2, color='white')

# 光照节点 ×4（左下）
box(0.6, 6.0, 7.6, 0.9, C_NODE, '光照节点 ESP32-C3 ×4', 'gy30_sender.ino · NODE_ID=1~4', fs=8, subfs=5.8)
box(0.6, 4.6, 7.6, 1.1, C_I2C, 'GY-30 (BH1750) 光照模块', 'SDA→GPIO8 · SCL→GPIO9\nVCC 3.3V · 地址 0x23', fs=7.2, subfs=5.6)

# 温湿度节点（右下）
box(17.6, 6.0, 7.6, 0.9, C_NODE, '温湿度节点 ESP32-C3', 'dht11_sensor.ino', fs=8, subfs=5.8)
box(17.6, 4.6, 7.6, 1.1, C_SENS, 'DHT11 + OLED(SSD1315)', 'DHT11 DATA→GPIO3\nOLED SDA→GPIO8 SCL→GPIO9 (0x3C)', fs=7.2, subfs=5.6)

# 无线通信：光照节点 → 天气屏 (UDP)
wire(8.2, 5.15, 9.0, 5.15, C_WIRE_UDP, lw=1.6, ls='--', label='UDP 8266')
ax.text(8.6, 5.45, 'LUX1~4:值\n(无线广播)', ha='center', fontsize=5.8, color=C_WIRE_UDP)

# UART：天气屏 ↔ 主控（双向）
wire(12.5, 6.6, 12.5, 10.0, C_WIRE_UART, lw=1.6, label='UART TX19/RX20')
wire(14.5, 10.0, 14.5, 6.6, C_WIRE_UART, lw=1.6)

# 温湿度节点独立显示（无连线，标注说明）
ax.text(21.4, 3.35, '（温湿度节点独立本地显示，不联网）', ha='center',
        fontsize=6, color=C_SUB, style='italic')

# ============================================================
# 图例
# ============================================================
legend_items = [
    ('主控制器 / 从板', C_MAIN),
    ('传感器(输入)', C_SENS),
    ('执行器(输出)', C_ACT),
    ('I2C 器件(0x40/0x23)', C_I2C),
    ('SPI 器件', C_SPI),
    ('电源', C_POWER),
    ('UART 有线', C_WIRE_UART),
    ('UDP 无线(8266)', C_WIRE_UDP),
]
patches = [mpatches.Patch(color=c, label=n) for n, c in legend_items]
leg = ax.legend(handles=patches, loc='upper right', fontsize=7,
                framealpha=0.9, edgecolor='#bdc3c7', bbox_to_anchor=(0.99, 0.995))
leg.set_zorder(20)

# 底部注脚
ax.text(13, 0.35, '注：1) NPN接近开关由GPIO39改接GPIO42(GPIO39带SUB-SPI复用不可靠)；2) 两块S3板UART交叉：主控TX(19)→天气屏RX(20)、天气屏TX(19)→主控RX(20)；\n'
                 '    3) IBT-2为BTS7960双半桥驱动板(R_EN/L_EN须接5V高电平，GND须与ESP32共地)；4) 导轨舵机为连续旋转(500us=CCW/1500us=停/2500us=CW)，风门/排粮为MG90S标准舵机。',
        ha='center', va='center', fontsize=6.4, color=C_SUB, linespacing=1.7)

plt.tight_layout(pad=1.0)
out = r'c:/Users/Acer/Desktop/gym/全系统接线图.png'
plt.savefig(out, dpi=180, bbox_inches='tight', facecolor='#f7f8fa', edgecolor='none')
print('已保存:', out)
plt.close()

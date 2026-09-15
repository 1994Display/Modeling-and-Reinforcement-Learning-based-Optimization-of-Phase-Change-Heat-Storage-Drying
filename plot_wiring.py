import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch
import numpy as np

plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

fig, ax = plt.subplots(1, 1, figsize=(18, 12))
ax.set_xlim(0, 18)
ax.set_ylim(0, 12)
ax.axis('off')
ax.set_facecolor('#f5f5f5')
fig.patch.set_facecolor('#f5f5f5')

# ── Colors ──
COL_HEADER_BG   = '#2c3e50'
COL_HEADER_TEXT = 'white'
COL_ROW_ODD     = '#ecf0f1'
COL_ROW_EVEN    = '#ffffff'
COL_ESP_BG      = '#34495e'
COL_ESP_TEXT    = 'white'
COL_DHT_BG      = '#27ae60'
COL_DS18B20_BG  = '#1abc9c'
COL_FAN_BG      = '#2980b9'
COL_PUMP_BG     = '#8e44ad'
COL_DAMPER_BG   = '#e67e22'
COL_DISCH_BG    = '#e74c3c'
COL_HEATER_BG   = '#f39c12'
COL_RAIL_BG     = '#16a085'
COL_DCMOTOR_BG  = '#6c3483'
COL_LIGHT_BG    = '#f4d03f'
COL_COLLISION_BG= '#5d6d7e'
COL_LCD_BG      = '#16a085'
COL_POWER_BG    = '#c0392b'
COL_L298N_BG    = '#7d3c98'
COL_PCA_BG      = '#2e86c1'
COL_BORDER      = '#bdc3c7'
COL_TEXT        = '#2c3e50'
COL_SUBTEXT     = '#7f8c8d'
COL_WIRE        = '#555555'

def draw_box(x, y, w, h, color, text, text_color='white', fontsize=9, bold=True):
    box = FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.15",
                         facecolor=color, edgecolor=color, linewidth=1.5)
    ax.add_patch(box)
    ax.text(x + w/2, y + h/2, text, ha='center', va='center',
            color=text_color, fontsize=fontsize, fontweight='bold' if bold else 'normal')

def draw_wire(x1, y1, x2, y2, color=COL_WIRE, lw=1.5, ls='-', label=''):
    ax.plot([x1, x2], [y1, y2], color=color, linewidth=lw, linestyle=ls, zorder=1)
    if label:
        mid_x, mid_y = (x1+x2)/2, (y1+y2)/2
        ax.text(mid_x, mid_y + 0.08, label, ha='center', va='bottom',
                fontsize=7, color=COL_WIRE, style='italic')

# ============================================================
# TITLE
# ============================================================
ax.text(9, 11.55, 'ESP32-S3  粮食干燥系统接线图（v34.6 全元件）', ha='center', va='center',
        fontsize=18, fontweight='bold', color='#2c3e50')

# ============================================================
# ESP32-S3 主板（中央）
# ============================================================
draw_box(6.2, 8.6, 5.6, 1.5, COL_ESP_BG, 'ESP32-S3 主板', COL_ESP_TEXT, 14)

# GPIO labels inside ESP box
gpio_data = [
    (6.6, 9.6, 'GPIO4', 'DHT11', COL_DHT_BG),
    (7.5, 9.6, 'GPIO8', '风机PWM', COL_FAN_BG),
    (8.4, 9.6, 'GPIO6', '水泵PWM', COL_PUMP_BG),
    (9.3, 9.6, 'GPIO9', '加热片', COL_HEATER_BG),
    (10.2, 9.6, 'GPIO2', '碰撞', COL_COLLISION_BG),
    (11.4, 9.6, 'GPIO18', 'DS18B20', COL_DS18B20_BG),
]
for gx, gy, name, role, clr in gpio_data:
    ax.text(gx, gy, f'{name}\n({role})', ha='center', va='center',
            fontsize=6, color=clr, fontweight='bold',
            bbox=dict(boxstyle='round,pad=0.2', facecolor='#1a252f', edgecolor=clr, linewidth=1))

ax.text(6.6, 8.8, 'GPIO7\n进导轨', ha='center', va='center',
        fontsize=6, color=COL_RAIL_BG, fontweight='bold',
        bbox=dict(boxstyle='round,pad=0.2', facecolor='#1a252f', edgecolor=COL_RAIL_BG, linewidth=1))
ax.text(7.6, 8.8, 'GPIO10\n出导轨', ha='center', va='center',
        fontsize=6, color=COL_RAIL_BG, fontweight='bold',
        bbox=dict(boxstyle='round,pad=0.2', facecolor='#1a252f', edgecolor=COL_RAIL_BG, linewidth=1))
ax.text(8.6, 8.8, 'GPIO3/1\nL298N', ha='center', va='center',
        fontsize=6, color=COL_L298N_BG, fontweight='bold',
        bbox=dict(boxstyle='round,pad=0.2', facecolor='#1a252f', edgecolor=COL_L298N_BG, linewidth=1))
ax.text(9.6, 8.8, 'I2C40/41\nBH1750+PCA', ha='center', va='center',
        fontsize=6, color=COL_LIGHT_BG, fontweight='bold',
        bbox=dict(boxstyle='round,pad=0.2', facecolor='#1a252f', edgecolor=COL_LIGHT_BG, linewidth=1))
ax.text(10.7, 8.8, 'SPI\nILI9341', ha='center', va='center',
        fontsize=6, color=COL_LCD_BG, fontweight='bold',
        bbox=dict(boxstyle='round,pad=0.2', facecolor='#1a252f', edgecolor=COL_LCD_BG, linewidth=1))

# 导轨碰撞开关 GPIO38/37/47/48
ax.text(11.5, 8.8, 'SW1-4\nGPIO38/37\n47/48', ha='center', va='center',
        fontsize=5.5, color=COL_COLLISION_BG, fontweight='bold',
        bbox=dict(boxstyle='round,pad=0.2', facecolor='#1a252f', edgecolor=COL_COLLISION_BG, linewidth=1))

# ============================================================
# 元器件模块（环绕 ESP32 排列）
# ============================================================
# 左侧列
draw_box(0.8, 9.0, 2.8, 1.0, COL_DHT_BG, 'DHT11 温湿度\n(干燥仓内)', 'white', 8)
draw_box(0.8, 7.5, 2.8, 1.0, COL_LIGHT_BG, 'BH1750 光照\n(I2C)', 'black', 8)
draw_box(0.8, 6.0, 2.8, 1.0, COL_DS18B20_BG, 'DS18B20 水温\n(GPIO18)', 'white', 8)

# 顶部
draw_box(3.8, 9.0, 2.6, 1.0, COL_FAN_BG, '风机 12V\n(MOSFET)', 'white', 8)
draw_box(6.6, 9.0, 2.6, 1.0, COL_PUMP_BG, '水泵 5V\n(GPIO6)', 'white', 8)
draw_box(9.4, 9.0, 2.6, 1.0, COL_HEATER_BG, '加热片\n(MOS GPIO9)', 'black', 8)

# 右侧
draw_box(12.4, 9.0, 2.6, 1.0, COL_RAIL_BG, '进气管舵机\n(GPIO7 连续旋转)', 'white', 8)
draw_box(15.2, 9.0, 2.6, 1.0, COL_RAIL_BG, '出气管舵机\n(GPIO10 连续旋转)', 'white', 8)
draw_box(12.4, 7.5, 2.6, 1.0, COL_DAMPER_BG, '风门舵机 MG90S\n(PCA9685 CH4)', 'white', 8)
draw_box(15.2, 7.5, 2.6, 1.0, COL_DISCH_BG, '排粮舵机 MG90S\n(PCA9685 CH5)', 'white', 8)

# 底部
draw_box(12.4, 5.8, 2.6, 1.0, COL_L298N_BG, 'L298N 驱动\n(直流减速电机)', 'white', 8)
draw_box(12.4, 4.3, 2.6, 1.0, COL_DCMOTOR_BG, '直流减速电机\n(L298N 控制)', 'white', 8)
draw_box(8.0, 5.8, 3.0, 1.0, COL_COLLISION_BG, '碰撞开关\n(GPIO2 电机)', 'white', 8)
draw_box(8.0, 4.3, 3.0, 1.0, COL_COLLISION_BG, '导轨碰撞开关 ×4\n(GPIO38/37/47/48)', 'white', 8)

draw_box(4.2, 5.8, 2.6, 1.0, COL_LCD_BG, 'ILI9341 触摸屏\n(SPI)', 'white', 8)
draw_box(4.2, 4.3, 2.6, 1.0, COL_POWER_BG, '12V 电源\n(风机+电机)', 'white', 8)
draw_box(1.2, 4.3, 2.2, 1.0, '#f39c12', '5V 供电\n(舵机+水泵)', 'white', 8)

# PCA9685 舵机驱动板
draw_box(13.9, 6.6, 1.6, 0.8, COL_PCA_BG, 'PCA9685\n16路舵机板', 'white', 7)

# ============================================================
# 接线（连线）
# ============================================================
# DHT11 → ESP32 (GPIO4)
draw_wire(3.6, 9.5, 6.6, 9.8, '#27ae60', 1.5, '-', 'DATA->GPIO4')
# BH1750 → ESP32 I2C
draw_wire(3.6, 8.0, 9.6, 8.8, '#f4d03f', 1.5, '-', 'I2C->GPIO40/41')
# DS18B20 → ESP32 GPIO18
draw_wire(3.6, 6.5, 11.4, 9.6, '#1abc9c', 1.5, '-', 'DATA->GPIO18')

# 风机 → GPIO8
draw_wire(5.1, 9.5, 7.5, 9.8, '#2980b9', 1.5, '-', 'SIG->GPIO8')
# 水泵 → GPIO6
draw_wire(7.9, 9.5, 8.4, 9.8, '#8e44ad', 1.5, '-', 'SIG->GPIO6')
# 加热片 → GPIO9
draw_wire(10.7, 9.5, 9.3, 9.8, '#f39c12', 1.5, '-', 'SIG->GPIO9')

# 进气管舵机 → GPIO7
draw_wire(13.7, 9.5, 6.6, 8.8, '#16a085', 1.5, '-', 'SIG->GPIO7')
# 出气管舵机 → GPIO10
draw_wire(16.5, 9.5, 7.6, 8.8, '#16a085', 1.5, '-', 'SIG->GPIO10')

# 风门舵机 → PCA9685 CH4
draw_wire(13.7, 8.0, 13.9, 7.0, '#e67e22', 1.2, '-', 'CH4')
# 排粮舵机 → PCA9685 CH5
draw_wire(16.5, 8.0, 15.3, 7.0, '#e74c3c', 1.2, '-', 'CH5')
# PCA9685 → ESP32 I2C
draw_wire(14.7, 6.6, 9.6, 8.8, '#2e86c1', 1.2, '-', 'I2C')

# L298N → ESP32 (GPIO3/1)
draw_wire(13.7, 6.3, 8.6, 8.8, '#7d3c98', 1.5, '-', 'IN1->GPIO3\nIN2->GPIO1')
# 直流电机 → L298N
draw_wire(13.7, 5.3, 13.7, 5.8, '#6c3483', 1.5, '-', 'OUT')

# 碰撞开关(电机) → GPIO2
draw_wire(9.5, 6.3, 9.3, 9.6, '#5d6d7e', 1.5, '-', 'GPIO2')
# 导轨碰撞开关 → ESP32
draw_wire(9.5, 4.8, 11.5, 8.8, '#5d6d7e', 1.5, '-', 'SW1-4')

# ILI9341 → ESP32 SPI
draw_wire(5.5, 6.3, 10.7, 8.8, '#16a085', 1.5, '-', 'SPI')

# 12V → 风机/电机
draw_wire(5.5, 4.8, 5.1, 9.0, '#c0392b', 1.2, '--', '12V')
draw_wire(6.2, 4.8, 13.7, 5.8, '#c0392b', 1.2, '--', '12V')
# 5V → 舵机/水泵
draw_wire(2.3, 4.8, 13.7, 7.5, '#f39c12', 1.2, '--', '5V')
draw_wire(2.3, 5.0, 7.9, 9.0, '#f39c12', 1.2, '--', '5V')

# 共地示意
ax.annotate('', xy=(2.2, 4.2), xytext=(6.4, 8.5),
            arrowprops=dict(arrowstyle='->', color='#95a5a6', lw=1.2, linestyle='dashed'))
ax.text(4.2, 6.0, '所有 GND 共地', fontsize=7, color='#95a5a6', style='italic',
        bbox=dict(boxstyle='round,pad=0.1', facecolor='white', edgecolor='#95a5a6', linewidth=0.5))

# ============================================================
# 图例 Legend
# ============================================================
legend_items = [
    ('DHT11 温湿度（干燥仓内）', COL_DHT_BG),
    ('BH1750 光照传感器', COL_LIGHT_BG),
    ('DS18B20 水温传感器', COL_DS18B20_BG),
    ('风机 (MOSFET 驱动)', COL_FAN_BG),
    ('水泵', COL_PUMP_BG),
    ('加热片 (MOS 驱动)', COL_HEATER_BG),
    ('进/出气管舵机（连续旋转）', COL_RAIL_BG),
    ('风门舵机 (MG90S)', COL_DAMPER_BG),
    ('排粮舵机 (MG90S)', COL_DISCH_BG),
    ('L298N + 直流减速电机', COL_L298N_BG),
    ('碰撞开关', COL_COLLISION_BG),
    ('PCA9685 舵机板', COL_PCA_BG),
    ('ILI9341 触摸屏', COL_LCD_BG),
    ('ESP32-S3 主板', COL_ESP_BG),
    ('12V / 5V 电源', COL_POWER_BG),
]
legend_patches = [mpatches.Patch(color=color, label=name) for name, color in legend_items]
legend = ax.legend(handles=legend_patches, loc='upper right', fontsize=6.5,
                   framealpha=0.9, edgecolor=COL_BORDER,
                   bbox_to_anchor=(1.02, 1.02), ncol=2)
legend.set_zorder(10)

# ============================================================
# 底部接线总表（纯文字表格）
# ============================================================
table_data = [
    ['元器件', '引脚', '连接目标', '信号/电压', '说明'],
    ['DHT11', 'VCC/DATA/GND', 'ESP32 3.3V / GPIO4 / GND', '3.3V 单总线', '干燥仓内温湿度'],
    ['BH1750', 'SDA/SCL', 'ESP32 GPIO40/41 (I2C)', 'I2C', '光照强度'],
    ['DS18B20', 'DATA', 'ESP32 GPIO18', '单总线', '水箱水温'],
    ['风机', 'SIG', 'ESP32 GPIO8 (PWM)', 'PWM 12V', '维可思MOSFET调速'],
    ['水泵', 'SIG', 'ESP32 GPIO6 (PWM)', 'PWM 5V', '水泵调速'],
    ['加热片', 'SIG', 'ESP32 GPIO9', '数字 HIGH/LOW', 'MOS驱动 启停'],
    ['进气管舵机', 'SIG', 'ESP32 GPIO7', 'PWM 连续旋转', '导轨插入/离开'],
    ['出气管舵机', 'SIG', 'ESP32 GPIO10', 'PWM 连续旋转', '导轨插入/离开'],
    ['风门舵机', 'SIG', 'PCA9685 CH4', 'PWM 50Hz', '0~90° 分热'],
    ['排粮舵机', 'SIG', 'PCA9685 CH5', 'PWM 50Hz', '0~90° 排粮'],
    ['PCA9685', 'SDA/SCL', 'ESP32 GPIO40/41 (I2C)', 'I2C', '舵机驱动板'],
    ['直流电机', 'IN1/IN2', 'ESP32 GPIO3/GPIO1', 'L298N 驱动', '集热器移动'],
    ['碰撞开关(电机)', 'OUT', 'ESP32 GPIO2', 'HIGH/LOW', '无碰撞=H,碰撞=L'],
    ['导轨碰撞开关×4', 'OUT', 'ESP32 GPIO38/37/47/48', 'LOW=触发', '导轨限位'],
    ['ILI9341', 'SPI', 'ESP32 SPI', 'SPI', '触摸屏'],
    ['12V 电源', 'VIN', '风机 + L298N', '12V', '独立供电'],
    ['5V 电源', 'VIN', '舵机 + 水泵', '5V', '独立供电'],
]

table_ax = fig.add_axes([0.03, 0.01, 0.94, 0.24])
table_ax.axis('off')
table = table_ax.table(
    cellText=table_data,
    cellLoc='center',
    loc='center',
    colWidths=[0.18, 0.18, 0.24, 0.16, 0.24],
)

table.auto_set_font_size(False)
table.set_fontsize(6.5)
table.scale(1.0, 1.3)

for i in range(len(table_data)):
    for j in range(5):
        cell = table[(i, j)]
        cell.set_linewidth(0.5)
        cell.set_edgecolor(COL_BORDER)
        if i == 0:
            cell.set_facecolor(COL_HEADER_BG)
            cell.set_text_props(color='white', fontweight='bold', fontsize=7)
        elif i % 2 == 1:
            cell.set_facecolor(COL_ROW_ODD)
            cell.set_text_props(color=COL_TEXT, fontsize=6.5)
        else:
            cell.set_facecolor(COL_ROW_EVEN)
            cell.set_text_props(color=COL_TEXT, fontsize=6.5)

plt.tight_layout(pad=3.0)
plt.savefig(r'c:\Users\HTY\Desktop\gym\wiring_diagram.png', dpi=200, bbox_inches='tight',
            facecolor='#f5f5f5', edgecolor='none')
print("接线图已保存: wiring_diagram.png")
plt.close()

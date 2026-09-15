"""
ESP32-S3 粮食干燥系统 —— 引脚接线总表生成器
输出: wiring_table.png （纯表格，无接线图）
用法: python wiring_table.py
"""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
import matplotlib.font_manager as fm

# 设置中文字体
plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

# ============================================================
# 接线总表数据（按 ESP32 引脚分类）
# ============================================================
header = ["引脚", "类型", "连接元件", "信号/电压", "说明"]

rows = [
    # ===== 数字/模拟 输入输出 =====
    ["GPIO4",   "数字输入", "DHT11 DATA",        "单总线 3.3V",  "干燥仓温湿度数据"],
    ["GPIO18",  "数字输入", "DS18B20 DATA",       "单总线 3.3V",  "水箱水温数据（需4.7kΩ上拉）"],
    ["GPIO2",   "数字输入", "碰撞开关 OUT",       "HIGH=无碰撞\nLOW=有碰撞", "直流电机位置检测"],
    ["GPIO38",  "数字输入", "导轨开关1 (RAIL_SW1)","LOW=触发",    "出风口原点"],
    ["GPIO37",  "数字输入", "导轨开关2 (RAIL_SW2)","LOW=触发",    "出风口终点"],
    ["GPIO47",  "数字输入", "导轨开关3 (RAIL_SW3)","LOW=触发",    "进风口终点"],
    ["GPIO48",  "数字输入", "导轨开关4 (RAIL_SW4)","LOW=触发",    "进风口原点"],

    # ===== PWM 输出 =====
    ["GPIO8",   "PWM 输出", "风机 MOS SIG",       "PWM 20kHz\n12V 0-100%", "风机转速"],
    ["GPIO6",   "PWM 输出", "水泵 SIG",           "PWM 20kHz\n5V 0-100%",  "水泵功率"],

    # ===== 数字输出 =====
    ["GPIO9",   "数字输出", "加热片 MOS",          "HIGH=加热\nLOW=关闭",   "加热片启停"],
    ["GPIO3",   "数字输出", "L298N IN1",           "HIGH/LOW",               "直流电机反转控制"],
    ["GPIO1",   "数字输出", "L298N IN2",           "HIGH/LOW",               "直流电机控制"],

    # ===== 导轨舵机（ESP32Servo 直驱）=====
    ["GPIO7",   "PWM 输出", "进气管舵机 SIG",      "50Hz 500-2500us",        "进风口导轨（连续旋转）"],
    ["GPIO10",  "PWM 输出", "出气管舵机 SIG",      "50Hz 500-2500us",        "出风口导轨（连续旋转）"],

    # ===== I2C =====
    ["GPIO40",  "I2C SDA", "BH1750 SDA\nPCA9685 SDA", "I2C 3.3V",  "光照传感器 + 舵机驱动板"],
    ["GPIO41",  "I2C SCL", "BH1750 SCL\nPCA9685 SCL", "I2C 3.3V",  "光照传感器 + 舵机驱动板"],

    # ===== SPI（TFT_eSPI 配置）=====
    ["SPI",     "SPI",     "ILI9341 触摸屏",      "SPI 3.3V",   "显示屏（按 TFT_eSPI 库配置）"],

    # ===== 电源 =====
    ["3.3V",    "电源",    "ESP32 供电",          "3.3V",       "DHT11/BH1750/DS18B20/舵机信号"],
    ["5V",      "电源",    "水泵/舵机",           "5V",         "水泵、MG90S舵机、L298N逻辑"],
    ["12V",     "电源",    "风机/L298N",          "12V",        "风机MOSFET输入、L298N电机电源"],
    ["GND",     "电源",    "所有元件",            "GND",        "全系统共地"],
]

# ============================================================
# PCA9685 通道表（舵机）
# ============================================================
pca_header = ["PCA9685 通道", "连接元件", "信号", "说明"]
pca_rows = [
    ["CH4", "风门舵机 MG90S", "PWM 50Hz 0-180°", "风门角度：0°=全干燥, 45°=半储热, 90°=全储热"],
    ["CH5", "排粮舵机 MG90S", "PWM 50Hz 0-180°", "排粮：90°=排粮, 0°=关闭"],
]

# ============================================================
# 绘制
# ============================================================
fig = plt.figure(figsize=(14, 16))
fig.patch.set_facecolor('white')

# 标题
fig.text(0.5, 0.97, "ESP32-S3 粮食干燥系统 — 引脚接线总表", ha='center',
         fontsize=18, fontweight='bold', color='#2c3e50')
fig.text(0.5, 0.945, "基于 esp32_controller.ino 引脚定义（v34.6）", ha='center',
         fontsize=11, color='#7f8c8d')

# 主接线表
ax = fig.add_axes([0.05, 0.30, 0.90, 0.62])
ax.axis('off')
table = ax.table(cellText=rows, colLabels=header, cellLoc='center', loc='center',
                 colWidths=[0.09, 0.11, 0.20, 0.20, 0.25])

table.auto_set_font_size(False)
table.set_fontsize(9)
table.scale(1.0, 1.5)

# 表头样式
for j in range(5):
    cell = table[(0, j)]
    cell.set_facecolor('#2c3e50')
    cell.set_text_props(color='white', fontweight='bold', fontsize=10)

# 行样式（按功能分组着色）
colors = {
    "传感器": '#eafaf1',
    "电机/泵": '#eaf2f8',
    "加热/机械": '#fef9e7',
    "I2C/SPI": '#f4ecf7',
    "电源": '#fdedec',
}
row_colors = [
    "#eafaf1", "#eafaf1", "#eafaf1", "#eafaf1", "#eafaf1", "#eafaf1", "#eafaf1",  # 传感器/开关
    "#eaf2f8", "#eaf2f8",                                                          # 电机/泵
    "#fef9e7", "#fef9e7", "#fef9e7",                                                # 加热/机械
    "#fef9e7", "#fef9e7",                                                          # 导轨舵机
    "#f4ecf7", "#f4ecf7",                                                          # I2C
    "#f4ecf7",                                                                     # SPI
    "#fdedec", "#fdedec", "#fdedec", "#fdedec",                                    # 电源
]
for i, c in enumerate(row_colors):
    for j in range(5):
        table[(i+1, j)].set_facecolor(c)
        table[(i+1, j)].set_edgecolor('#bdc3c7')
        table[(i+1, j)].set_linewidth(0.5)

# PCA9685 通道表
ax2 = fig.add_axes([0.05, 0.12, 0.90, 0.15])
ax2.axis('off')
pca_table = ax2.table(cellText=pca_rows, colLabels=pca_header, cellLoc='center', loc='center',
                      colWidths=[0.15, 0.25, 0.25, 0.35])
pca_table.auto_set_font_size(False)
pca_table.set_fontsize(9)
pca_table.scale(1.0, 1.6)
for j in range(4):
    cell = pca_table[(0, j)]
    cell.set_facecolor('#2e86c1')
    cell.set_text_props(color='white', fontweight='bold', fontsize=10)
for i in range(1, 3):
    for j in range(4):
        pca_table[(i, j)].set_facecolor('#eaf2f8')
        pca_table[(i, j)].set_edgecolor('#bdc3c7')

# 注脚
fig.text(0.5, 0.05, "注：导轨舵机为连续旋转舵机（500us=CCW全速, 1500us=停止, 2500us=CW全速）\n"
         "     风门/排粮舵机为 MG90S 标准舵机，由 PCA9685 驱动（I2C 地址 0x40）\n"
         "     BH1750 光照传感器 I2C 地址 0x23（备用 0x5C）",
         ha='center', fontsize=9, color='#7f8c8d', linespacing=1.8)

plt.savefig(r'c:\Users\HTY\Desktop\gym\wiring_table.png', dpi=200, bbox_inches='tight',
            facecolor='white', edgecolor='none')
print("接线总表已保存: wiring_table.png")
plt.close()

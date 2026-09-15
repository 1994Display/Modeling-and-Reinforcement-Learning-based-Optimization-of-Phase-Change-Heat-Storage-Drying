# -*- coding: utf-8 -*-
"""生成 COLLECT 模式自动流程循环闭环流程图"""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Polygon, FancyArrowPatch

plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'Arial Unicode MS']
plt.rcParams['axes.unicode_minus'] = False

fig, ax = plt.subplots(figsize=(13, 15))
ax.set_xlim(0, 13)
ax.set_ylim(0, 15)
ax.axis('off')

BOX_W = 4.2
BOX_H = 0.95
CX = 6.5  # 主链中心 x

def box(x, y, text, fc='#E8F0FE', ec='#2F5BAA'):
    ax.add_patch(Rectangle((x - BOX_W/2, y - BOX_H/2), BOX_W, BOX_H,
                           facecolor=fc, edgecolor=ec, linewidth=1.6, zorder=2))
    ax.text(x, y, text, ha='center', va='center', fontsize=10.5, zorder=3)

def diamond(x, y, text, w=3.0, h=1.5):
    ax.add_patch(Polygon([(x, y+h/2), (x+w/2, y), (x, y-h/2), (x-w/2, y)],
                         closed=True, facecolor='#FFF3CD', edgecolor='#B8860B',
                         linewidth=1.6, zorder=2))
    ax.text(x, y, text, ha='center', va='center', fontsize=9.5, zorder=3)

def arrow(x0, y0, x1, y1, color='#333333', style='-|>', lw=1.6):
    ax.add_patch(FancyArrowPatch((x0, y0), (x1, y1), arrowstyle=style,
                                 mutation_scale=14, color=color, linewidth=lw, zorder=1))

# ===== 主流程链 =====
box(CX, 14.0, '上电启动')
box(CX, 12.8, '回原点  CS_HOMING\n(进风→SW4 出风→SW1)')
box(CX, 11.5, '就绪  CS_READY\n(等待 START / 手动操控)')
box(CX, 10.2, '固定等待 10s  CS_WAIT_LIGHT')
box(CX, 8.9, '转盘启动定位  CS_MOTOR_GO\n(碰撞开关 松开→再压)')
box(CX, 7.6, '插入集热器  CS_PIPES_INSERT\n(进风→SW3 出风→SW2 · 记录参考光照)')
box(CX, 6.3, '自动干燥  CS_AUTO_DRY\n(3吸1放 · 光照驱动周期 20~60s)')

# 主链箭头
arrow(CX, 14.0-BOX_H/2, CX, 12.8+BOX_H/2)
arrow(CX, 12.8-BOX_H/2, CX, 11.5+BOX_H/2)
arrow(CX, 11.5-BOX_H/2, CX, 10.2+BOX_H/2)
arrow(CX, 10.2-BOX_H/2, CX, 8.9+BOX_H/2)
arrow(CX, 8.9-BOX_H/2, CX, 7.6+BOX_H/2)
arrow(CX, 7.6-BOX_H/2, CX, 6.3+BOX_H/2)

# 判断菱形
diamond(CX, 4.9, '第 3 次\n干燥?')

# 从自动干燥到判断
arrow(CX, 6.3-BOX_H/2, CX, 4.9+0.75)

# 否 → 直接到导轨回原点（左分支）
ax.text(CX-2.2, 5.0, '否', fontsize=11, ha='center', color='#C0392B', fontweight='bold')
arrow(CX-1.5, 4.9, CX-2.2, 4.9, color='#C0392B')
arrow(CX-2.2, 4.9, CX-2.2, 3.6, color='#C0392B')
arrow(CX-2.2, 3.6, CX-BOX_W/2, 3.6, color='#C0392B')

# 是 → 排粮 → 回导轨回原点（右分支）
ax.text(CX+2.2, 5.0, '是', fontsize=11, ha='center', color='#2E7D32', fontweight='bold')
arrow(CX+1.5, 4.9, CX+2.2, 4.9, color='#2E7D32')
box(CX+3.6, 4.9, '排粮 3s\n(舵机90°→复位0°)', fc='#E8F5E9', ec='#2E7D32')
arrow(CX+2.2, 4.9, CX+3.6-BOX_W/2, 4.9, color='#2E7D32')
arrow(CX+3.6, 4.9-BOX_H/2, CX+3.6, 3.6, color='#2E7D32')
arrow(CX+3.6, 3.6, CX+BOX_W/2, 3.6, color='#2E7D32')

# 导轨回原点
box(CX, 3.6, '导轨回原点  CS_DRY_HOMING\n(进风→SW4 出风→SW1)')
box(CX, 2.3, '转盘换位  CS_DRY_MOTOR\n(电机转 · 碰撞定位)')
box(CX, 1.0, '重新插入  CS_DRY_INSERT\n(进风→SW3 出风→SW2)')

arrow(CX, 3.6-BOX_H/2, CX, 2.3+BOX_H/2)
arrow(CX, 2.3-BOX_H/2, CX, 1.0+BOX_H/2)

# 循环回环：重新插入 → 回到自动干燥（左侧）
arrow(CX-BOX_W/2, 1.0, 1.0, 1.0)
arrow(1.0, 1.0, 1.0, 6.3)
arrow(1.0, 6.3, CX-BOX_W/2, 6.3)
ax.text(0.65, 3.6, '重新计时\n回到干燥', fontsize=10, ha='center', color='#1A5276',
        rotation=90, fontweight='bold')

plt.tight_layout()
plt.savefig('collect_flowchart.png', dpi=130, bbox_inches='tight',
            facecolor='white')
print('OK: collect_flowchart.png saved')

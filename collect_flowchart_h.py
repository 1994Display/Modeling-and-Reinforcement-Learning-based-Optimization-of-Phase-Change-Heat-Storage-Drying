# -*- coding: utf-8 -*-
"""生成 立体循环集热自动闭环流程图（横向 · 紧凑 · 大字体）"""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, FancyArrowPatch, FancyBboxPatch, Rectangle

plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'Arial Unicode MS']
plt.rcParams['axes.unicode_minus'] = False

fig, ax = plt.subplots(figsize=(23, 11.5))
ax.set_xlim(0, 23)
ax.set_ylim(0, 11.5)
ax.axis('off')

BOX_W = 3.35
BOX_H = 1.7

def box(x, y, title, sub, fc='#EAF2FF', ec='#3B6FD4'):
    ax.add_patch(FancyBboxPatch((x - BOX_W/2, y - BOX_H/2), BOX_W, BOX_H,
                                boxstyle='round,pad=0.02,rounding_size=0.14',
                                facecolor=fc, edgecolor=ec, linewidth=2.2, zorder=2))
    if sub:
        ax.text(x, y + 0.42, title, ha='center', va='center', fontsize=21,
                fontweight='bold', color='#16305E', zorder=3)
        ax.text(x, y - 0.42, sub, ha='center', va='center', fontsize=14.5,
                color='#22334A', zorder=3, linespacing=1.4)
    else:
        ax.text(x, y, title, ha='center', va='center', fontsize=23,
                fontweight='bold', color='#16305E', zorder=3)

def diamond(x, y, text, w=3.0, h=2.2):
    ax.add_patch(Polygon([(x, y+h/2), (x+w/2, y), (x, y-h/2), (x-w/2, y)],
                         closed=True, facecolor='#FFF7DC', edgecolor='#D99A06',
                         linewidth=2.2, zorder=2))
    ax.text(x, y, text, ha='center', va='center', fontsize=18,
            fontweight='bold', color='#7A5500', zorder=3, linespacing=1.3)

def arrow(x0, y0, x1, y1, color='#4A4A4A', lw=2.2):
    ax.add_patch(FancyArrowPatch((x0, y0), (x1, y1), arrowstyle='-|>',
                                 mutation_scale=20, color=color, linewidth=lw,
                                 zorder=1, shrinkA=0, shrinkB=0))

# 标题
ax.text(11.5, 10.9, '立体循环集热自动闭环流程图',
        ha='center', va='center', fontsize=26, fontweight='bold', color='#12264C')

# ===== 主链（y=8.6）=====
box(1.8, 8.6, '上电启动', '')
box(5.0, 8.6, '回原点', '进→SW4\n出→SW1')
box(8.2, 8.6, '就绪', '等待\nSTART')
box(11.4, 8.6, '固定等待', '10 秒')
box(14.6, 8.6, '转盘定位', '碰撞\n松开→再压')
box(17.8, 8.6, '插入集热器', '记录\n参考光照')
box(21.0, 8.6, '自动干燥', '3吸1放\n20~60秒')

for x0, x1 in [(1.8, 5.0), (5.0, 8.2), (8.2, 11.4), (11.4, 14.6),
               (14.6, 17.8), (17.8, 21.0)]:
    arrow(x0 + BOX_W/2, 8.6, x1 - BOX_W/2, 8.6)

# 判断菱形
diamond(21.0, 5.7, '第 3 次\n干燥?')
arrow(21.0, 8.6 - BOX_H/2, 21.0, 5.7 + 1.1)

# ===== 循环链（y=2.4）=====
box(21.0, 2.4, '排粮 3 秒', '舵机\n90°→0°', fc='#E8F7EC', ec='#2E9E4F')
box(17.6, 2.4, '导轨回原点', '进→SW4\n出→SW1')
box(14.2, 2.4, '转盘换位', '电机转\n碰撞定位')
box(10.8, 2.4, '重新插入', '进→SW3\n出→SW2')

# 是分支
ax.text(22.2, 5.7, '是', fontsize=22, color='#1F7A33', fontweight='bold',
        ha='left', va='center')
arrow(21.0 + 1.5, 5.7, 22.3, 5.7, color='#2E9E4F')
arrow(22.3, 5.7, 22.3, 4.4, color='#2E9E4F')
arrow(22.3, 4.4, 21.0, 4.4, color='#2E9E4F')
arrow(21.0, 4.4, 21.0, 2.4 + BOX_H/2, color='#2E9E4F')

# 否分支
ax.text(19.3, 5.2, '否', fontsize=22, color='#C0392B', fontweight='bold',
        ha='center', va='center')
arrow(21.0 - 1.5, 5.7, 17.6 + BOX_W/2, 2.4 + BOX_H/2, color='#C0392B')

# 排粮 → 导轨回原点
arrow(21.0 - BOX_W/2, 2.4, 17.6 + BOX_W/2, 2.4, color='#2E9E4F')

# 导轨回原点 → 转盘换位 → 重新插入
arrow(17.6 - BOX_W/2, 2.4, 14.2 + BOX_W/2, 2.4)
arrow(14.2 - BOX_W/2, 2.4, 10.8 + BOX_W/2, 2.4)

# 回环：重新插入 → 自动干燥
arrow(10.8, 2.4 - BOX_H/2, 10.8, 0.9)
arrow(10.8, 0.9, 21.0, 0.9)
arrow(21.0, 0.9, 21.0, 8.6)
arrow(21.0, 8.6, 21.0 + BOX_W/2, 8.6)
ax.text(15.9, 1.25, '换位完成 → 重新计时 → 回到自动干燥', fontsize=16,
        ha='center', color='#1A5276', fontweight='bold')

# 图例（右上角空白区）
legend_y = 10.5
ax.add_patch(Rectangle((16.5, legend_y-0.3), 0.6, 0.6, facecolor='#EAF2FF',
                       edgecolor='#3B6FD4', linewidth=2.0))
ax.text(17.35, legend_y, '主流程', fontsize=14, va='center', color='#333333')
ax.add_patch(Polygon([(19.5, legend_y-0.3), (20.1, legend_y), (19.5, legend_y+0.3),
                      (18.9, legend_y)], closed=True, facecolor='#FFF7DC',
                edgecolor='#D99A06', linewidth=2.0))
ax.text(20.35, legend_y, '判断', fontsize=14, va='center', color='#333333')
ax.add_patch(Rectangle((22.0, legend_y-0.3), 0.6, 0.6, facecolor='#E8F7EC',
                       edgecolor='#2E9E4F', linewidth=2.0))
ax.text(22.85, legend_y, '排粮', fontsize=14, va='center', color='#333333')

plt.tight_layout()
plt.savefig('collect_flowchart_h.png', dpi=140, bbox_inches='tight',
            facecolor='white')
print('OK: collect_flowchart_h.png saved')

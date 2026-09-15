# -*- coding: utf-8 -*-
# 生成常见城市的 (UTF-8, GBK) 双编码表，供 Arduino 草图使用
import sys

cities = [
  "北京","上海","广州","深圳","杭州","南京","武汉","成都","重庆","西安",
  "天津","苏州","长沙","青岛","厦门","福州","济南","合肥","郑州","昆明",
  "哈尔滨","长春","沈阳","大连","南宁","海口","三亚","兰州","贵阳","太原",
  "石家庄","南昌","南宁","银川","西宁","呼和浩特","乌鲁木齐","拉萨","温州","佛山",
  "东莞","珠海","无锡","宁波","唐山","保定","邯郸","秦皇岛","潍坊","烟台"
]

print("// 城市表：UTF-8（发 API）+ GBK（显示）")
print("// 用法：选一行取消注释（或者改 CITY_INDEX）")
for c in cities:
    try:
        u = c.encode("utf-8")
        g = c.encode("gbk")
        u_hex = ", ".join("0x{:02X}".format(b) for b in u)
        g_hex = ", ".join("0x{:02X}".format(b) for b in g)
        print(f'  // {c}: UTF-8{{{u_hex}}}, GBK{{{g_hex}}}, lenU={len(u)}, lenG={len(g)}')
    except Exception as e:
        print(f'  // ERR {c}: {e}', file=sys.stderr)
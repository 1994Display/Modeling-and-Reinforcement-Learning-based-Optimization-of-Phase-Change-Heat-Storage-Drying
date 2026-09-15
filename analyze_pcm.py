import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# 读取数据
df = pd.read_excel(r'C:\Users\HTY\Desktop\数据.xlsx', sheet_name='PCM', header=None)
data = df.iloc[2:].copy()
data.columns = ['time_s', 'T_pcm', 'col2', 'col3', 'T_in1', 'T_in2']
data = data[['time_s', 'T_pcm', 'T_in1', 'T_in2']].astype(float)
data['hours'] = data['time_s'] / 3600

V_dot_air = 0.015  # m3/s (假设风机风量 ~50 m3/h)
rho_air = 1.2
cp_air = 1005    # J/kgK
T_amb = 20.0

# 已知
T_solidus, T_liquidus = 37.0, 44.0
I_solar = 800.0
eta = 0.12

# 峰值点
idx_peak = data['T_pcm'].idxmax()
t_peak = data['time_s'].iloc[idx_peak]
T_max = data['T_pcm'].iloc[idx_peak]

# ===== 按实际物理过程分段 =====
# 关键发现: 充电期极短, 说明太阳能输入远大于模型假设
# 用文献值: 石蜡 cp_sol≈1.8, cp_liq≈2.1 kJ/kgK, L≈180 kJ/kg

cp_sol = 1.8   # kJ/kgK 
cp_liq = 2.1   # kJ/kgK
L_latent = 180 # kJ/kg (石蜡典型值)

# 放电阶段更干净 (只有风机提取, 无太阳能), 更适合推参数
# 放电总时长 5.41h → end
mask_dis = data['time_s'] >= t_peak
dis = data[mask_dis].copy()

# 放电阶段能量平衡推 m
# 液态冷却: Tmax → 44°C
mask_dl = (dis['T_pcm'] > T_liquidus)
dl = dis[mask_dl]
if len(dl) > 5:
    dT_dl = dl['T_pcm'].iloc[0] - dl['T_pcm'].iloc[-1]
    dt_dl = (dl['hours'].iloc[-1] - dl['hours'].iloc[0]) * 3600
    
    # 凝固相变段
    mask_dlat = (dis['T_pcm'] >= T_solidus) & (dis['T_pcm'] <= T_liquidus)
    dlat = dis[mask_dlat]
    dt_dlat = (dlat['hours'].iloc[-1] - dlat['hours'].iloc[0]) * 3600
    
    # 风机提取的热量 Q = m_dot * cp * (T_out - T_in) * dt
    # 用进口温度变化: T_in1≈常温, T_in2≈PCM换热后出风
    # 放电时 T_in2 从 ~82°C 骤降到 ~40°C (说明风机开始抽PCM热量)
    
    # 近似: 风机工作时, 出风温度 ≈ (T_in1 + T_pcm)/2
    # Q_extract = V_dot × ρ × cp × (T_out_avg - T_in1) × dt
    
    # 更直接的: 液态冷却速率 = P_extract / (m* cp_liq)
    # 考虑空气也带走热量, 但此时T_in2 > T_pcm (太阳能余热+储热都在放)
    
    # 用固态冷却段推 m (最干净: 无相变, 无太阳能)
    mask_ds = (dis['T_pcm'] < T_solidus) & (dis['T_pcm'] > T_amb + 2)
    ds = dis[mask_ds]
    
    if len(ds) > 20:
        # 取 37°C→30°C 这段 (较快冷却, 避免逼近环境时失真)
        mask_fit = (ds['T_pcm'] >= 22) & (ds['T_pcm'] <= 36)
        fit = ds[mask_fit]
        
        # 指数衰减: T(t) = T_amb + (T0-T_amb)*exp(-t/tau)
        # tau = m*cp / (UA) = m*cp / (Vdot*rho*cp_air*NTU)
        
        # 简单线性拟合
        T_vals = fit['T_pcm'].values
        t_vals = fit['time_s'].values
        t_rel = t_vals - t_vals[0]
        
        # ln((T - T_amb)/(T0 - T_amb)) = -t/tau
        y = np.log((T_vals - T_amb) / (T_vals[0] - T_amb))
        coeff = np.polyfit(t_rel, y, 1)
        tau = -1.0 / coeff[0]
        
        print(f"=== 固态冷却段指数拟合 ===")
        print(f"  时间常数 tau = {tau:.0f} s = {tau/60:.1f} min")
        print(f"  R^2 = {np.corrcoef(t_rel, y)[0,1]**2:.4f}")
        
        # 若 UA = Vdot*rho*cp_air (简化, 假设 NTU→大)
        # 则 m = tau * UA / cp_sol
        UA_min = V_dot_air * rho_air * cp_air  # W/K (最小UA, 理想换热)
        
        # 实际换热需要 NTU = exp(UA_actual/UA_min)
        # 从数据 T_in2 和 T_pcm 的温差可以估算 NTU
        
        # 固态冷却段入口温度T_in1 ≈ 20°C, T_in2 ≈ 25-30°C
        # ΔT ≈ T_pcm - T_in1, 用 lumped capacitance: m*cp*dT/dt = -UA*(T-T_in)
        # 从斜率: dT/dt = -UA*(T-T_in)/(m*cp)
        # 在 T=30°C 点: T-T_in ≈ 10°C, dT/dt ≈ -0.5°C/h = -0.000139°C/s
        # UA/(m*cp) = -dT/dt / (T-T_in) = 0.000139/10 = 1.39e-5
        # m = UA/(1.39e-5*cp)
        
        # 直接用tau: m*cp_sol/UA = tau
        # UA_min = 0.015*1.2*1005 = 18.09 W/K
        # m_min = tau * UA_min / (cp_sol*1000) 
        m_est_min = tau * UA_min / (cp_sol * 1000)
        print(f"  UA_min = {UA_min:.2f} W/K")
        print(f"  m_est_min = {m_est_min:.2f} kg (假设 NTU→∞)")
        print(f"  若 NTU=1 (ε=0.63): m ≈ {m_est_min*1.0:.2f} kg")
        print(f"  若 NTU=0.5 (ε=0.39): m ≈ {m_est_min*0.5:.2f} kg")
        
        # 反过来: 若 m=5kg, 推 UA
        UA_for_m5 = 5 * cp_sol * 1000 / tau
        NTU = UA_for_m5 / UA_min
        print(f"\n  若 m = 5kg: UA = {UA_for_m5:.2f} W/K, NTU = {NTU:.3f}")
        print(f"  NTU={NTU:.3f} -> effectiveness = {1-np.exp(-NTU):.3f}")

# ===== 用凝固相变段反推 m =====
if len(dlat) > 5:
    dt_dlat_hr = dt_dlat / 3600
    # 凝固段: Q_latent = m * L = P_extract * dt
    # P_extract 从前后固态冷却段推导
    
    # 取凝固段前后的空气温差估算提取功率
    # 凝固段 T_in1 ≈ 20-22°C, T_in2 ≈ 30-40°C
    T_in1_dlat = dlat['T_in1'].mean()
    T_in2_dlat = dlat['T_in2'].mean()
    
    # 空气吸收的热量: P_air = Vdot*rho*cp*(T_out - T_in)
    P_extract_est = V_dot_air * rho_air * cp_air * (T_in2_dlat - T_in1_dlat)
    
    # m = P_extract * dt / L (忽略少量显热 37→44)
    m_from_latent = P_extract_est * dt_dlat / (L_latent * 1000)
    
    print(f"\n=== 凝固段反推 PCM 质量 ===")
    print(f"  凝固段时长: {dt_dlat_hr:.3f} h = {dt_dlat:.0f} s")
    print(f"  进口温度1 (均值): {T_in1_dlat:.1f} °C")
    print(f"  进口温度2 (均值): {T_in2_dlat:.1f} °C")
    print(f"  空气提取功率: {P_extract_est:.1f} W")
    print(f"  推算 m = {m_from_latent:.2f} kg")
    print(f"  若 m = {m_from_latent:.2f} kg, 验证液态冷却段...")
    
    # 验证: 液态冷却段 ΔT=60°C 需要 Q=m*cp_liquid*60 kJ
    Q_liq = m_from_latent * cp_liq * dT_dl * 1000
    t_liq_check = Q_liq / P_extract_est
    print(f"  液态冷却需散热: {Q_liq/1000:.1f} kJ → 预计 {(t_liq_check/3600)*60:.0f} 分钟")
    print(f"  实际液态冷却: {(dt_dl/60):.0f} 分钟")

# ===== 汇总 =====
m_use = m_from_latent if m_from_latent > 0.1 else 5.0
# 用 m_use 重算太阳能功率
# 充电固态段: Q = m*cp_sol*(37-18.6), t≈1.62h
# 取固态段数据
mask_s_chg = (data['T_pcm'] < T_solidus) & (data['time_s'] < t_peak)
s_chg = data[mask_s_chg]
if len(s_chg) > 5:
    dt_s_chg = (s_chg['hours'].iloc[-1] - s_chg['hours'].iloc[0]) * 3600
    Q_s_chg = m_use * cp_sol * (T_solidus - s_chg['T_pcm'].iloc[0]) * 1000
    P_solar_from_solid = Q_s_chg / dt_s_chg
    
    # 液态过热段
    mask_l_chg = (data['T_pcm'] > T_liquidus) & (data['time_s'] <= t_peak)
    l_chg = data[mask_l_chg]
    dt_l_chg = (l_chg['hours'].iloc[-1] - l_chg['hours'].iloc[0]) * 3600
    Q_l_chg = m_use * cp_liq * (T_max - T_liquidus) * 1000
    P_solar_from_liquid = Q_l_chg / dt_l_chg
    
    print(f"\n=== 太阳能功率推算 (m={m_use:.2f}kg) ===")
    print(f"  固态段推算 P_solar = {P_solar_from_solid:.1f} W")
    print(f"  液态段推算 P_solar = {P_solar_from_liquid:.1f} W")
    
    # 太阳能集热面积
    A_from_solid = P_solar_from_solid / (I_solar * eta)
    A_from_liquid = P_solar_from_liquid / (I_solar * eta)
    P_avg = (P_solar_from_solid + P_solar_from_liquid) / 2
    A_avg = P_avg / (I_solar * eta)
    
    print(f"\n  对应集热面积:")
    print(f"    固态段: A = {A_from_solid:.3f} m^2 = {A_from_solid*10000:.0f} cm^2")
    print(f"    液态段: A = {A_from_liquid:.3f} m^2 = {A_from_liquid*10000:.0f} cm^2")
    print(f"    平均: A = {A_avg:.3f} m^2 = {A_avg*10000:.0f} cm^2")

# ===== 最终推荐参数 =====
print(f"\n{'='*60}")
print("=== ANSYS Fluent 物性参数设置建议 ===")
print(f"{'='*60}")
print(f"""
[PCM 材料属性] (Material Properties)
  密度 (Density):          780-820 kg/m^3 (石蜡)
  比热-固态 (Cp_solid):    {cp_sol*1000:.0f} J/kg·K
  比热-液态 (Cp_liquid):   {cp_liq*1000:.0f} J/kg·K  
  导热系数 (k):            0.5-2.0 W/m·K (膨胀石墨增强)
  固相线 (Solidus):        {T_solidus:.0f} °C = {T_solidus+273.15:.0f} K
  液相线 (Liquidus):       {T_liquidus:.0f} °C = {T_liquidus+273.15:.0f} K
  潜热 (Latent Heat):      {L_latent*1000:.0f} J/kg
  
  注: Solidification & Melting 模型, 选 Mushy zone parameter = 1e5 (默认)

[边界条件] (Boundary Conditions)
  风道入口: Velocity Inlet / Mass Flow Inlet
    入口温度: 从数据 T_in1(t) 输入 (19.8 - 44.5°C)
    速度: 根据风机流量设定
  
  太阳能侧 (若耦合): 
    热流密度: 800 × 0.12 = 96 W/m^2 × 集热面积 {A_avg:.3f} m^2

[验证目标] (Validation Targets)
  1. PCM 出口风温曲线  vs 实测 T_in2(t) 放电段
  2. PCM 温降曲线      vs 实测 T_pcm(t) 放电段
  3. 相变平台持续时间  - 实测约 {dt_dlat/3600:.1f}h
  4. PCM 最高温度      - 实测 {T_max:.1f}°C (验证过度充电保护)
""")

# 打印给 Python 模型 的参数修正
print(f"{'='*60}")
print("=== my_test.py 模型参数修正建议 ===")
print(f"{'='*60}")
print(f"""
当前 my_test.py 中参数:
  self.m_pcm = 5.0 kg       →  建议改为 {m_use:.2f} kg
  self.P_solar = 70.0 W     →  建议改为 {P_avg:.1f} W
  self.T_solidus = 37°C     →  保持
  self.T_liquidus = 44°C    →  保持
  c_p 隐含值需要更新        →  cp_sol={cp_sol*1000:.0f}, cp_liq={cp_liq*1000:.0f} J/kgK
  潜热 (在 physical_model)  →  确认是否使用 {L_latent} kJ/kg
  
  另外: 太阳能功率可能随时间变化 (太阳升起时递增)
        可加入时变函数: P_solar(t) = f(时间), 而非定值
""")

# ===== 详细图表 =====
fig, axes = plt.subplots(2, 2, figsize=(16, 10))
ts = data['hours'].values
tp = data['T_pcm'].values

# 子图1: 完整周期 + 相变标注
ax = axes[0, 0]
ax.plot(ts, tp, 'r-', lw=0.8, label='T_PCM')
ax.plot(ts, data['T_in1'], 'b-', lw=0.5, alpha=0.5, label='T_in1')
ax.plot(ts, data['T_in2'], 'orange', lw=0.5, alpha=0.5, label='T_in2')
ax.axhline(T_solidus, color='green', ls='--', alpha=0.5)
ax.axhline(T_liquidus, color='green', ls='--', alpha=0.5)
ax.axvline(t_peak/3600, color='gray', ls=':', alpha=0.5)
ax.fill_between([T_solidus, T_liquidus], 0, 110, color='green', alpha=0.08, transform=ax.get_xaxis_transform())
ax.annotate(f'Peak: {T_max:.1f}°C\n@ {t_peak/3600:.2f}h', xy=(t_peak/3600, T_max),
           xytext=(t_peak/3600+1, T_max+5), fontsize=8,
           arrowprops=dict(arrowstyle='->', color='gray'))
ax.annotate(f'Phase Change\n37-44°C', xy=(2.5, 40), fontsize=8, color='green')
ax.set_xlabel('Time (hours)')
ax.set_ylabel('Temperature (°C)')
ax.set_title('Full 24h Cycle: PCM + Inlet Temperatures')
ax.legend(fontsize=7)
ax.grid(True, alpha=0.3)
ax.set_xlim(0, 25)

# 子图2: 放电段放大
ax2 = axes[0, 1]
t0_dis = t_peak / 3600
t_end_dis = data['hours'].iloc[-1]
ax2.plot(ts, tp, 'r-', lw=0.8)
ax2.plot(ts, data['T_in1'], 'b-', lw=0.5, alpha=0.5)
ax2.plot(ts, data['T_in2'], 'orange', lw=0.5, alpha=0.5)
ax2.axhline(T_solidus, color='green', ls='--', alpha=0.5)
ax2.axhline(T_liquidus, color='green', ls='--', alpha=0.5)
ax2.axvline(t0_dis, color='gray', ls=':', alpha=0.5)
# 标注放电各阶段
if len(dl) > 5:
    ax2.axvspan(dl['hours'].iloc[0], dl['hours'].iloc[-1], alpha=0.1, color='red')
    ax2.text(dl['hours'].mean()-0.8, 80, 'Liquid\nCooling', fontsize=7, color='red')
if len(dlat) > 5:
    ax2.axvspan(dlat['hours'].iloc[0], dlat['hours'].iloc[-1], alpha=0.15, color='green')
    ax2.text(dlat['hours'].mean()-1.0, 40, f'Solidification\n{dt_dlat/3600:.1f}h', fontsize=7, color='green')
if len(ds) > 5:
    ax2.axvspan(ds['hours'].iloc[0], ds['hours'].iloc[-1], alpha=0.08, color='blue')
ax2.set_xlabel('Time (hours)')
ax2.set_ylabel('Temperature (°C)')
ax2.set_title('Discharge Phase Detail (Fan ON)')
ax2.grid(True, alpha=0.3)
ax2.set_xlim(t0_dis - 0.5, t0_dis + 10)
ax2.set_ylim(15, 110)

# 子图3: 温度变化率
ax3 = axes[1, 0]
dt_s = 10
dT = np.diff(tp)
dTdt = dT / (dt_s / 3600)
tm = ts[:-1] + dt_s/7200
ax3.plot(tm, dTdt, lw=0.5, color='steelblue')
ax3.axhline(0, color='black', ls='-', alpha=0.3)
ax3.axvline(t_peak/3600, color='gray', ls=':', alpha=0.5)
ax3.fill_between(tm, 0, dTdt, where=(dTdt>=0), color='red', alpha=0.2)
ax3.fill_between(tm, dTdt, 0, where=(dTdt<0), color='blue', alpha=0.2)
ax3.set_xlabel('Time (hours)')
ax3.set_ylabel('dT/dt (°C/h)')
ax3.set_title('Temperature Change Rate')
ax3.grid(True, alpha=0.3)
ax3.set_xlim(0, 25)
ax3.set_ylim(-50, 80)

# 子图4: T_pcm vs T_in2 散点 (换热温差)
ax4 = axes[1, 1]
# 充电阶段
mchg = data['time_s'] <= t_peak
ax4.scatter(data.loc[mchg, 'T_in2'] - data.loc[mchg, 'T_pcm'], 
           data.loc[mchg, 'T_pcm'], s=1, alpha=0.3, c='red', label='Charging')
# 放电阶段
mdis = data['time_s'] > t_peak
ax4.scatter(data.loc[mdis, 'T_in2'] - data.loc[mdis, 'T_pcm'],
           data.loc[mdis, 'T_pcm'], s=1, alpha=0.3, c='blue', label='Discharging')
ax4.axvline(0, color='gray', ls=':', alpha=0.5)
ax4.axhline(T_solidus, color='green', ls='--', alpha=0.4)
ax4.axhline(T_liquidus, color='green', ls='--', alpha=0.4)
ax4.set_xlabel('T_in2 - T_pcm (°C)')
ax4.set_ylabel('T_pcm (°C)')
ax4.set_title('Heat Transfer Driving Force')
ax4.legend(fontsize=7)
ax4.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig(r'c:\Users\HTY\Desktop\gym\pcm_analysis.png', dpi=150)
print("\n详细图表已保存: pcm_analysis.png")

"""
位置环积分饱和对照仿真（含速度环 PI + 机械模型）

链路：位置环(4k) -> 速度给定 -> 速度环PI(4k) -> iq限幅 -> 转矩 -> 机械 -> 位置

模型 A：修复前 —— 位置环固定限幅 30，MCL 原生 integral += err 只有硬钳位，无抗饱和
模型 B：修复后 —— 接近限速 sqrt(2*a*|err|) + clamping 抗饱和 + 死区泄放
"""
import math

Ts = 0.00025                     # 控制周期 250us (4kHz)

# ---- 位置环 ----
POS_KP, POS_KI = 20.0, 0.025
POS_IMAX       = 300.0
POS_OUTMAX     = 30.0
DECEL          = 150.0
DEADBAND       = 0.01
LEAK           = 2.0

# ---- 速度环（MCL: integral += ki*err）----
SPD_KP, SPD_KI = 0.02, 0.00008
SPD_IMAX       = 3.0

# ---- 机械 / 电气 ----
IQ_MAX  = 3.0        # A
KT      = 0.03       # N.m/A
J       = 2.0e-5     # kg.m^2
B_VIS   = 2.0e-5     # N.m.s/rad
T_FRIC  = 0.006      # N.m 库仑摩擦
W_STIC  = 0.05       # rad/s 静摩擦死区速度

THETA_T = 1.5708     # 目标 90 deg


def sim(mode, t_end=2.0):
    th = w = 0.0
    i_pos = i_spd = 0.0
    prev_i = 0.0
    log = []
    n = int(t_end / Ts)
    for k in range(n):
        t = k * Ts
        err = THETA_T - th

        # ===== 位置环 =====
        if mode == 'B':
            aerr = abs(err) - DEADBAND
            vlim = 0.0 if aerr <= 0 else min(POS_OUTMAX, math.sqrt(2.0 * DECEL * aerr))
        else:
            vlim = POS_OUTMAX

        prev_i = i_pos
        i_pos += err                                  # MCL 原生：integral += err（无 dt、无 ki）
        i_pos = max(-POS_IMAX, min(POS_IMAX, i_pos))  # 只有硬钳位

        w_ref = POS_KP * err + POS_KI * i_pos
        w_ref = max(-vlim, min(vlim, w_ref))

        if mode == 'B':                               # 抗饱和 / 泄放
            if vlim <= 0.0:
                if i_pos > LEAK:    i_pos -= LEAK
                elif i_pos < -LEAK: i_pos += LEAK
                else:               i_pos = 0.0
            elif abs(w_ref) >= vlim - 1e-3 and err * w_ref > 0.0:
                i_pos = prev_i

        # ===== 速度环 =====
        ew = w_ref - w
        i_spd += SPD_KI * ew
        i_spd = max(-SPD_IMAX, min(SPD_IMAX, i_spd))
        iq = SPD_KP * ew + i_spd
        iq = max(-IQ_MAX, min(IQ_MAX, iq))

        # ===== 机械 =====
        T = KT * iq - B_VIS * w
        if abs(w) > W_STIC:
            T -= T_FRIC * (1.0 if w > 0 else -1.0)
        else:
            if abs(T) < T_FRIC:
                T = 0.0
                w = 0.0
            else:
                T -= T_FRIC * (1.0 if T > 0 else -1.0)
        w += (T / J) * Ts
        th += w * Ts

        if k % 2 == 0:
            log.append((round(t * 1000, 2), th, w, i_pos, w_ref, iq))
        if mode == 'B' and abs(err) < DEADBAND and abs(w) < 0.02 and k > 400:
            break
    return log


def report(log, name):
    th = [r[1] for r in log]; w = [r[2] for r in log]
    ig = [r[3] for r in log]; wr = [r[4] for r in log]

    segs, sign, start = [], 0, 0
    for i, wi in enumerate(w):
        s = 1 if wi > 0.15 else (-1 if wi < -0.15 else 0)
        if s and sign and s != sign:
            segs.append((sign, round(log[start][0], 0), round(log[i][0], 0),
                         round(math.degrees(th[start]), 1), round(math.degrees(th[i]), 1)))
            start = i
        if s: sign = s
    if sign:
        segs.append((sign, round(log[start][0], 0), round(log[-1][0], 0),
                     round(math.degrees(th[start]), 1), round(math.degrees(th[-1]), 1)))

    print(f'=== {name} ===')
    print(f'  运动分段 (方向, 起ms, 止ms, 起deg, 止deg):')
    for s in segs:
        d = '正转' if s[0] > 0 else '反转'
        print(f'     {d}  {s[1]:>5} -> {s[2]:>5} ms   {s[3]:>7.1f} -> {s[4]:>7.1f} deg')
    print(f'  过冲          : {math.degrees(max(th) - THETA_T):+.2f} deg')
    print(f'  integral 峰值 : {max(ig):.1f}  终值 {ig[-1]:.2f}')
    print(f'  最大反向速度  : {min(w):.3f} rad/s')
    print(f'  结束位置      : {math.degrees(th[-1]):.2f} deg  (目标 90.00)\n')
    return log


a = report(sim('A'), 'A 无抗饱和（修复前）')
b = report(sim('B'), 'B 接近限速+抗饱和（修复后）')

# A 的退饱和时间
ig = [r[3] for r in a]; t = [r[0] for r in a]; tha = [r[1] for r in a]
pk = ig.index(max(ig))
z = next((i for i in range(pk, len(ig)) if ig[i] <= 0), None)
if z:
    print(f'A: integral 峰值 {max(ig):.0f} @ {t[pk]:.0f} ms，'
          f'退到 0 用了 {t[z]-t[pk]:.0f} ms，'
          f'这期间又多走 {math.degrees(tha[z]-THETA_T):+.1f} deg')

# ===== 关键：误差翻负 -> 输出翻负 的滞后 =====
print('\n--- A 的控制作用滞后 ---')
e_neg = next((i for i in range(len(a)) if THETA_T - a[i][1] < 0), None)
o_neg = next((i for i in range(len(a)) if a[i][4] < 0), None)
print(f'  误差 err 翻负 : t={t[e_neg]:.0f} ms   位置={math.degrees(tha[e_neg]):.2f} deg')
print(f'  输出 w_ref 翻负: t={t[o_neg]:.0f} ms   位置={math.degrees(tha[o_neg]):.2f} deg')
print(f'  滞后时间      : {t[o_neg]-t[e_neg]:.0f} ms')
print(f'  滞后期间多冲  : {math.degrees(tha[o_neg]-tha[e_neg]):.2f} deg')
print(f'  此刻 integral : {ig[o_neg]:.1f}  (积分项 {0.025*ig[o_neg]:+.2f} rad/s, '
      f'比例项 {20*(THETA_T-tha[o_neg]):+.2f} rad/s)')

# 理论：要压过饱和积分，误差必须多大
print(f'\n  理论: integral=300 时积分项 = 0.025*300 = {0.025*300:.2f} rad/s')
print(f'        要让 kp*err 压过它，err 必须 < {-0.025*300/20:.4f} rad '
      f'= {math.degrees(-0.025*300/20):.2f} deg')
print(f'        也就是说：已经越过目标后，还得再冲过 {abs(math.degrees(-0.025*300/20)):.1f} deg，')
print(f'        控制器才"想起来"该往回转。')

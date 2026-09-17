"""
位置环 S 曲线规划闭环仿真

对比三种方案在同一套电机模型上的表现：
  A) 改进前：固定目标 + 接近限速 sqrt(2*a*|err|)
  B) S 曲线，但**不加**速度前馈（位置环 P 去追移动的目标点）
  C) S 曲线 + 速度前馈（本工程采用的方案）

模型参数与实机一致：
  J=6.2e-6 kg·m^2, Kt=0.054 N·m/A, 库仑摩擦 0.004 N·m
  速度环 4kHz  kp=0.02  ki=0.00008 (integral += ki*err), iq 限幅 ±3A
  位置环 4kHz  kp=20    ki=0.025   (integral += err),  积分限幅 ±300
  速度反馈过 IIR(100Hz) 等效时延约 5ms

运行：python tools/sim_scurve.py
"""
import math
from collections import deque

from test_scurve import Scurve  # 复用规划器（与 C 版同一套公式）

DT = 0.00025          # 4kHz
KT = 0.054            # N·m/A
J = 6.2e-6            # kg·m^2
TC = 0.004            # 库仑摩擦 N·m
B = 2.0e-6            # 粘性系数
I_MAX = 3.0           # Q 轴电流限幅 A

SP_KP, SP_KI = 0.02, 0.00008
PO_KP, PO_KI = 20.0, 0.025
PO_INT_MAX = 300.0
OUT_MAX = 30.0        # 位置环输出（速度给定）上限
DECEL_MAX = 150.0
DEADBAND = 0.01
LEAK = 2.0
TRACK_VLIM = 8.0

DELAY_N = 20          # 5ms @4kHz


class Motor:
    def __init__(self):
        self.w = 0.0
        self.th = 0.0
        self.iq = 0.0
        self.sp_int = 0.0
        self.fb_buf = deque([0.0] * (DELAY_N + 1))

    def step(self, ref_speed):
        # ---- 速度环 PI（4kHz）----
        fb = self.fb_buf[0]
        err = ref_speed - fb
        self.sp_int += SP_KI * err
        if self.sp_int > I_MAX:
            self.sp_int = I_MAX
        if self.sp_int < -I_MAX:
            self.sp_int = -I_MAX
        iq = SP_KP * err + self.sp_int
        if iq > I_MAX:
            iq = I_MAX
        if iq < -I_MAX:
            iq = -I_MAX
        self.iq = iq

        # ---- 机械 ----
        tq = KT * iq
        load = B * self.w
        if abs(self.w) > 1e-3:
            load += TC * (1.0 if self.w > 0 else -1.0)
        else:
            # 静摩擦区：转矩不足以克服摩擦则保持静止
            if abs(tq) < TC:
                tq_eff = 0.0
                self.w = 0.0
            else:
                tq_eff = tq - TC * (1.0 if tq > 0 else -1.0)
            load = 0.0
            tq = tq_eff
        dw = (tq - load) / J * DT
        self.w += dw
        self.th += self.w * DT

        # ---- 反馈延迟 ----
        self.fb_buf.append(self.w)
        self.fb_buf.popleft()


def run(mode, delta=12.5 * math.pi, vmax=25.0, amax=150.0, jmax=3000.0, t_end=3.0):
    m = Motor()
    sc = Scurve()
    if mode != "A":
        sc.start(0.0, delta, vmax, amax, jmax)

    pos_int = 0.0
    n = int(t_end / DT)
    log = []
    ff_on = (mode == "C")

    for k in range(n):
        # ---- 规划器推进 ----
        if mode == "A":
            ref_pos = delta
            ff = 0.0
            running = False
        else:
            if sc.state == 1:
                sc.step()
            ref_pos = sc.th
            ff = sc.w if ff_on else 0.0
            running = (sc.state == 1)

        err = ref_pos - m.th

        # ---- 位置环限幅 ----
        if running:
            budget = max(0.0, OUT_MAX - abs(ff))
            vlim = min(budget, TRACK_VLIM)
        else:
            aerr = abs(err) - DEADBAND
            vlim = 0.0 if aerr <= 0 else min(math.sqrt(2 * DECEL_MAX * aerr), OUT_MAX)

        # ---- 位置环 PI（MCL 语义：integral += err）----
        int_prv = pos_int
        pos_int += err
        pos_int = max(-PO_INT_MAX, min(PO_INT_MAX, pos_int))
        out = PO_KP * err + PO_KI * pos_int

        if out > vlim:
            out = vlim
        elif out < -vlim:
            out = -vlim

        # 抗饱和
        if vlim > 0 and abs(out) >= vlim - 1e-3 and err * out > 0:
            pos_int = int_prv
        # 死区泄放
        if vlim <= 0.0:
            if pos_int > LEAK:
                pos_int -= LEAK
            elif pos_int < -LEAK:
                pos_int += LEAK
            else:
                pos_int = 0.0

        # ---- 前馈注入 ----
        ref_speed = out + ff
        ref_speed = max(-OUT_MAX, min(OUT_MAX, ref_speed))

        m.step(ref_speed)
        log.append((k * DT, m.th, ref_pos, m.w, ref_speed, err))

    return log


def report(name, log, delta, show_follow=True):
    th = [r[1] for r in log]
    ref = [r[2] for r in log]
    w = [r[3] for r in log]
    err_follow = [ref[i] - th[i] for i in range(len(log))]
    # 只在运动过程中统计跟随误差（速度超过 0.5rad/s 时）
    moving = [abs(err_follow[i]) for i in range(len(log)) if abs(w[i]) > 0.5]
    max_follow = max(moving) if moving else 0.0

    final = th[-1]
    resid = delta - final
    overshoot = max(th) - delta
    max_w = max(abs(x) for x in w)

    # 到位时间：首次进入 ±0.02rad 且之后不再超出
    t_arrive = None
    for i in range(len(th) - 1, -1, -1):
        if abs(delta - th[i]) > 0.02:
            if i + 1 < len(th):
                t_arrive = (i + 1) * DT
            break
    t_arrive = 0.0 if t_arrive is None else t_arrive

    # 末端抖动：最后 0.5s 内位置峰峰值
    tail = th[int(len(th) * 5 / 6):]
    ripple = max(tail) - min(tail)

    follow_s = (f"{max_follow:.4f} rad ({math.degrees(max_follow):.2f}°)"
                if show_follow else "不适用（目标固定）")
    print(f"[{name}]")
    print(f"  运动段最大跟随误差 : {follow_s}")
    print(f"  最大速度           : {max_w:.2f} rad/s")
    print(f"  到位时间(±0.02rad) : {t_arrive:.3f} s")
    print(f"  终点位置           : {final:.5f} rad   残差 {resid:+.5f} rad ({math.degrees(resid):+.3f}°)")
    print(f"  过冲               : {overshoot:.5f} rad ({math.degrees(overshoot):+.3f}°)")
    print(f"  末段 0.5s 抖动峰峰 : {ripple:.5f} rad ({math.degrees(ripple):.3f}°)")
    print()
    return max_follow, resid, overshoot, ripple


def dump_speed(name, log, step_ms=50):
    """每 step_ms 采样一次实际速度，供画图/比对"""
    stride = int(step_ms * 1e-3 / DT)
    pts = [(log[i][0], log[i][3]) for i in range(0, len(log), stride)]
    print(f"{name} 速度采样 (t_ms, w_rad/s):")
    print("  " + " ".join(f"{t*1000:.0f}:{w:.2f}" for t, w in pts))
    print()


if __name__ == "__main__":
    D = 12.5 * math.pi
    print(f"位移 {D:.3f} rad (6.25 圈)，仿真 3s\n")
    la = run("A", D)
    lb = run("B", D)
    lc = run("C", D)
    report("A 改进前：固定目标 + 接近限速", la, D, show_follow=False)
    report("B S曲线，无速度前馈", lb, D)
    report("C S曲线 + 速度前馈（本方案）", lc, D)
    dump_speed("A", la)
    dump_speed("C", lc)

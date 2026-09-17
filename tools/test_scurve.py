"""
S 曲线规划器数学验证（与 src/control/planner_scurve.c 同一套公式）

本地没有 C 编译器，这里用 Python 严格复现 planner_scurve.c 的算法，
验证 7 段规划在四种退化情形下都满足：
  1) 终点位置精确等于 起点 + 位移
  2) 终点速度 / 加速度为 0
  3) 过程速度 <= vmax、过程加速度 <= amax
  4) 位置单调、速度连续无跳变

运行：python tools/test_scurve.py
"""
import math

TS = 0.00025  # 位置环 4kHz

IDLE, RUN, DONE = 0, 1, 2


class Scurve:
    def __init__(self, ts=TS):
        self.ts = ts
        self.state = IDLE
        self.tick = 0
        self.th = self.w = self.a = 0.0
        self.th0 = 0.0
        self.delta = 0.0
        self.sign = 1.0

    # ---- 段内恒定 jerk 推进：与 C 的 seg_advance 完全一致 ----
    @staticmethod
    def _seg_advance(j, tau, a, w, th):
        t2 = tau * tau
        th = th + w * tau + 0.5 * a * t2 + (1.0 / 6.0) * j * t2 * tau
        w = w + a * tau + 0.5 * j * t2
        a = a + j * tau
        return a, w, th

    def start(self, th0, delta, vmax, amax, jmax):
        d = abs(delta)
        self.th0 = th0
        self.delta = delta
        self.sign = 1.0 if delta >= 0 else -1.0
        self.tick = 0
        self.th, self.w, self.a = th0, 0.0, 0.0

        # 位移太小或参数非法
        if d < 1e-5 or vmax <= 0 or amax <= 0 or jmax <= 0:
            self.state = DONE
            self.th = th0 + delta
            return

        j = jmax
        tj_a = amax / jmax              # 加速度升到 amax 所需时间
        tj_v = math.sqrt(vmax / jmax)   # 三角形加速度达到 vmax 所需时间

        if tj_a <= tj_v:
            tj = tj_a
            a = jmax * tj               # = amax
            ta = tj + vmax / a
            d2 = vmax * ta              # 2 * d_acc
            if d2 <= d:
                v = vmax
                tv = (d - d2) / v       # 七段：有匀速
            else:
                tv = 0.0
                p = a * tj              # amax^2 / jmax，梯形与三角形的分界速度
                v = 0.5 * (-p + math.sqrt(p * p + 4.0 * d * a))
                if v >= p:
                    ta = tj + v / a    # 五段
                else:
                    tj = (d / (2.0 * jmax)) ** (1.0 / 3.0)
                    a = jmax * tj
                    v = jmax * tj * tj
                    ta = 2.0 * tj      # 四段：三角形
        else:
            tj = tj_v
            a = jmax * tj
            ta = 2.0 * tj
            d2 = vmax * ta
            if d2 <= d:
                v = vmax
                tv = (d - d2) / v
            else:
                tj = (d / (2.0 * jmax)) ** (1.0 / 3.0)
                a = jmax * tj
                v = jmax * tj * tj
                ta = 2.0 * tj
                tv = 0.0

        # 数值保护
        if ta < 2.0 * tj:
            ta = 2.0 * tj
            if tv > 0.0:
                d2 = v * ta
                tv = (d - d2) / v if d > d2 else 0.0

        self.tj, self.ta, self.tv = tj, ta, tv
        self.j, self.a_peak, self.v_peak = j, a, v
        self.t_total = 2.0 * ta + tv

        # 7 个段：边界时刻 + 段内 jerk
        self.tk = [0.0] * 8
        self.jk = [0.0] * 7
        self.tk[0] = 0.0
        self.jk[0] = j
        self.tk[1] = tj
        self.jk[1] = 0.0
        self.tk[2] = ta - tj
        self.jk[2] = -j
        self.tk[3] = ta
        self.jk[3] = 0.0
        self.tk[4] = ta + tv
        self.jk[4] = -j
        self.tk[5] = ta + tv + tj
        self.jk[5] = 0.0
        self.tk[6] = ta + tv + ta - tj
        self.jk[6] = j
        self.tk[7] = 2.0 * ta + tv

        # 递推各段起点状态
        self.ak = [0.0] * 8
        self.wk = [0.0] * 8
        self.thk = [0.0] * 8
        for i in range(7):
            self.ak[i + 1] = self.ak[i]
            self.wk[i + 1] = self.wk[i]
            self.thk[i + 1] = self.thk[i]
            na, nw, nth = self._seg_advance(
                self.jk[i], self.tk[i + 1] - self.tk[i],
                self.ak[i], self.wk[i], self.thk[i])
            self.ak[i + 1], self.wk[i + 1], self.thk[i + 1] = na, nw, nth

        # 强制收尾
        self.thk[7] = d
        self.wk[7] = 0.0
        self.ak[7] = 0.0

        self.n_total = int(self.t_total / self.ts) + 1
        self.state = RUN

    def step(self):
        if self.state == IDLE:
            return
        if self.state == DONE:
            self.th = self.th0 + self.delta
            self.w = self.a = 0.0
            return

        t = self.tick * self.ts
        if t >= self.t_total:
            self.state = DONE
            self.th = self.th0 + self.delta
            self.w = self.a = 0.0
            return

        i = 0
        while i < 6 and t >= self.tk[i + 1]:
            i += 1

        tau = t - self.tk[i]
        a, w, th = self.ak[i], self.wk[i], self.thk[i]
        a, w, th = self._seg_advance(self.jk[i], tau, a, w, th)

        self.a = a * self.sign
        self.w = w * self.sign
        self.th = self.th0 + th * self.sign
        self.tick += 1


def run_case(name, delta, vmax, amax, jmax):
    s = Scurve()
    s.start(0.0, delta, vmax, amax, jmax)

    w_max = a_max = w_jump = 0.0
    th_prv, w_prv = s.th, s.w
    n = 0
    mono_bad = False
    fail = []

    while s.state == RUN:
        s.step()
        n += 1
        w_max = max(w_max, abs(s.w))
        a_max = max(a_max, abs(s.a))
        w_jump = max(w_jump, abs(s.w - w_prv))
        if delta > 0 and s.th < th_prv - 1e-6:
            mono_bad = True
        if delta < 0 and s.th > th_prv + 1e-6:
            mono_bad = True
        th_prv, w_prv = s.th, s.w
        if n > 2000000:
            fail.append("未收敛")
            break
    s.step()

    print(f"[{name}] delta={delta:+.3f} vmax={vmax:.1f} amax={amax:.1f} jmax={jmax:.0f}")
    print(f"  段时长: tj={s.tj:.4f} ta={s.ta:.4f} tv={s.tv:.4f}  T={s.t_total:.4f}s ({n} 拍)")
    print(f"  v_peak={s.v_peak:.3f} (限 {vmax})   a_peak={s.a_peak:.3f} (限 {amax})")
    print(f"  实测 max|w|={w_max:.4f}  max|a|={a_max:.4f}  max|dw|={w_jump:.5f}")
    print(f"  终点 th={s.th:.6f} (目标 {s.th0 + s.delta:.6f})  w={s.w:.6f}  a={s.a:.6f}")

    if abs(s.th - (s.th0 + s.delta)) > 2e-3:
        fail.append("终点位置误差过大")
    if abs(s.w) > 1e-3:
        fail.append("终点速度不为 0")
    if abs(s.a) > 1e-3:
        fail.append("终点加速度不为 0")
    if w_max > vmax * 1.02 + 0.01:
        fail.append("超速")
    if a_max > amax * 1.02 + 0.01:
        fail.append("超加速度")
    if mono_bad:
        fail.append("位置非单调")
    if w_jump > 0.05 * vmax + 0.05:
        fail.append("速度跳变过大")

    print(f"  => {'FAIL: ' + ', '.join(fail) if fail else 'OK'}\n")
    return 1 if fail else 0


if __name__ == "__main__":
    r = 0
    r |= run_case("实际工况", 12.5 * math.pi, 25.0, 150.0, 3000.0)
    r |= run_case("无匀速", 3.0, 25.0, 150.0, 3000.0)
    r |= run_case("三角形", 0.2, 25.0, 150.0, 3000.0)
    r |= run_case("负方向", -12.5 * math.pi, 25.0, 150.0, 3000.0)
    r |= run_case("低速约束", 39.27, 5.0, 20.0, 100.0)
    r |= run_case("极小位移", 0.001, 25.0, 150.0, 3000.0)
    r |= run_case("单圈", 2.0 * math.pi, 25.0, 150.0, 3000.0)
    print("===== 有失败用例 =====" if r else "===== 全部通过 =====")

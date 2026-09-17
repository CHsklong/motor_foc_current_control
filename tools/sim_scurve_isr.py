# -*- coding: utf-8 -*-
"""
严格复刻 ctrl_foc.c 的 ISR 时序 + MCL 语义，对比 POS_SCURVE_ENABLE=0/1。

与 sim_scurve.py 的区别（那一份是 4kHz 单速率近似）：
  * 20kHz ISR，pre() 每拍跑，位置环/速度环每 5 拍跑一次（div 计数）
  * 抗饱和判定用的是【PID 自身输出】（加前馈之前），不是 exec_ref.speed
  * S 曲线模式下 vlim = clamp(OUTPUT_MAX-|ff|, POS_TRACK_VLIM_MIN, OUTPUT_MAX)
"""
import math
from test_scurve import Scurve

PWM_FREQ = 20000.0
TS_ISR = 1.0 / PWM_FREQ
POS_TS = 250e-6
POS_DIV = 5

KP_POS, KI_POS = 20.0, 0.025
INT_MAX = 300.0
OUT_MAX = 30.0
TRACK_VLIM = 8.0
FF_GAIN = 1.0
DECEL_MAX = 150.0
DEADBAND = 0.01
LEAK = 2.0

KP_SPD, KI_SPD = 0.02, 0.00008
SPD_INT_MAX = 3.0
SPD_OUT_MAX = 3.0

KT = 0.054
J = 6.2e-6
TC = 0.004          # 库仑摩擦 N*m
DLY_N = 100         # 5ms @20kHz

V_MAX, A_MAX, J_MAX = 25.0, 150.0, 3000.0
DELTA = 12.5 * math.pi


def lim(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


TRACK_VLIM_MIN = 4.0
ABORT_ERR = 3.0


def run(enable, t_end=3.0, log_every=0.1, tc=TC, fix=True):
    sc = Scurve(POS_TS)
    if enable:
        sc.start(0.0, DELTA, V_MAX, A_MAX, J_MAX)

    pos_int = 0.0
    pos_int_prv = 0.0
    omega_ff = 0.0
    vlim = 0.0
    pos_ref = DELTA
    pos_err = 0.0

    w = 0.0
    th = 0.0
    sp_int = 0.0
    last_ref = 0.0
    fb = [0.0] * (DLY_N + 1)
    fbi = 0

    cnt = 0
    abort_cnt = [0]
    n = int(t_end / TS_ISR)
    out_log = []
    for k in range(n):
        fire = False
        cnt += 1
        if cnt >= POS_DIV:
            cnt = 0
            fire = True

        # ---------------- position_loop_pre() ----------------
        if fire:
            if enable:
                sc.step()
                omega_ff = sc.w
            else:
                omega_ff = 0.0
            if enable and sc.state != 0:          # != SCURVE_IDLE
                pos_ref = sc.th

            if enable and sc.state == 1 and fix:          # 失速保护
                e0 = pos_ref - th
                if abs(e0) > ABORT_ERR:
                    sc.state = 2                          # DONE
                    sc.th = sc.th0 + sc.delta
                    sc.w = 0.0
                    omega_ff = 0.0
                    pos_ref = sc.th
                    abort_cnt[0] += 1

            pos_err = pos_ref - th
            pos_int_prv = pos_int

            if enable and sc.state == 1:          # SCURVE_RUN
                budget = OUT_MAX - abs(omega_ff)
                if fix:
                    budget = min(max(budget, TRACK_VLIM_MIN), OUT_MAX)
                else:
                    budget = 0.0 if budget < 0 else budget
                    budget = budget if budget < TRACK_VLIM else TRACK_VLIM
                vlim = budget
            else:
                aerr = abs(pos_err) - DEADBAND
                vlim = 0.0 if aerr <= 0 else min(math.sqrt(2 * DECEL_MAX * aerr), OUT_MAX)

            # ---- hpm_mcl_position_pid() + FF 包装 ----
            pos_int = lim(pos_int + pos_err, -INT_MAX, INT_MAX)
            pid_out = KP_POS * pos_err + KI_POS * pos_int
            pid_out = lim(pid_out, -vlim, vlim)
            out = lim(pid_out + omega_ff * FF_GAIN, -OUT_MAX, OUT_MAX)
            last_ref = out

            # ---- 速度环（同一拍）----
            sens = fb[(fbi + 1) % (DLY_N + 1)]
            sp_int = lim(sp_int + KI_SPD * (out - sens), -SPD_INT_MAX, SPD_INT_MAX)
            iq = lim(KP_SPD * (out - sens) + sp_int, -SPD_OUT_MAX, SPD_OUT_MAX)
        else:
            out = last_ref
            sens = fb[(fbi + 1) % (DLY_N + 1)]
            sp_int = lim(sp_int + KI_SPD * (out - sens), -SPD_INT_MAX, SPD_INT_MAX)
            iq = lim(KP_SPD * (out - sens) + sp_int, -SPD_OUT_MAX, SPD_OUT_MAX)

        # ---------------- 机械 + 反馈延迟（每 20kHz 拍）----------------
        tq = KT * iq
        if abs(w) > 1e-3:
            tq -= tc * (1.0 if w > 0 else -1.0)
        else:
            if abs(tq) < tc:
                tq = 0.0
                w = 0.0
            else:
                tq -= tc * (1.0 if tq > 0 else -1.0)
        w += (tq / J) * TS_ISR
        th += w * TS_ISR
        fb[fbi] = w
        fbi = (fbi + 1) % (DLY_N + 1)

        # ---------------- position_loop_post() ----------------
        if fire:
            if vlim <= 0.0:
                if pos_int > LEAK:
                    pos_int -= LEAK
                elif pos_int < -LEAK:
                    pos_int += LEAK
                else:
                    pos_int = 0.0
            else:
                cmp_out = pid_out if fix else out
                if ((cmp_out >= vlim - 0.001) or (cmp_out <= -vlim + 0.001)) \
                        and (pos_err * cmp_out > 0.0):
                    pos_int = pos_int_prv

        if k % int(log_every / TS_ISR) == 0:
            out_log.append((k * TS_ISR, th, w, pos_ref, out, vlim, omega_ff, pos_int))

    for r in out_log:
        print("  t=%5.2f th=%8.4f w=%7.2f ref=%8.4f out=%7.2f vlim=%6.2f ff=%6.2f int=%8.2f" % r)
    print("  END th=%.4f  resid=%+.4f rad (%+.2f deg)  末速=%.3f  失速保护触发=%d\n"
          % (th, th - DELTA, math.degrees(th - DELTA), w, abort_cnt[0]))
    return th


print("=== ENABLE=0（固定目标 + 接近限速）===")
run(False)
print("=== ENABLE=1（S 曲线 + 前馈）修复后 ===")
run(True, fix=True)

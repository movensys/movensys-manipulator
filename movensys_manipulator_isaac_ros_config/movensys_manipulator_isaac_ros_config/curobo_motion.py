"""Joint command generation within velocity, acceleration and jerk limits.

Ported from the CR3A cuRoboV2 demo (rt_motion.track_target): the MPC state runs ahead of
the robot (a carrot), and the command sent to the robot chases it with every joint bounded
by velocity, acceleration and jerk. Pure numpy, no ROS or GPU.
"""
import math

import numpy as np

WALL_DECEL = 0.8
WALL_INSET = 1e-3


def _clip(x, lo, hi):
    x = x if x > lo else lo
    return x if x < hi else hi


def _per_joint(x, n):
    return np.broadcast_to(np.asarray(x, float), (n,)).astype(float)


def _stop_consts(a, j):
    d0 = a ** 3 / j ** 2
    return d0, -a * a / (2.0 * j), a ** 4 / (4.0 * j * j), 2.0 * a


def _stop_speed(d, k, j):
    return math.cbrt(d * d * j) if d <= k[0] else k[1] + math.sqrt(k[2] + k[3] * d)


def stop_distance(v, a, j):
    v = np.abs(v)
    return np.where(v <= a * a / j, v * np.sqrt(v / j), v * v / (2.0 * a) + v * a / (2.0 * j))


def track_target(q0, v0, a0, target, n, h, vmax, amax, jmax, cycle, sub=5,
                 stop_decel=1.0, qlim=None):
    """Return n states (q, v, a), spaced h, that chase a fixed target within the limits.

    The wanted speed covers the remaining distance in one cycle, capped by the speed that can
    still stop within that distance and by vmax; the wanted acceleration is then clipped by the
    acceleration, velocity and jerk bounds. qlim = (lower, upper) treats joint limits as walls.
    """
    q, v, a = np.array(q0, float), np.array(v0, float), np.array(a0, float)
    tgt = np.asarray(target, float)
    nj = q.size
    vm, am, jm = _per_joint(vmax, nj), _per_joint(amax, nj), _per_joint(jmax, nj)
    lo_lim = hi_lim = None
    if qlim is not None:
        lo_lim = np.asarray(qlim[0], float) + WALL_INSET
        hi_lim = np.asarray(qlim[1], float) - WALL_INSET
        tgt = np.clip(tgt, lo_lim, hi_lim)
    hh = h / sub
    den = max(cycle / 4.0, hh)
    out_q, out_v, out_a = np.zeros((n, nj)), np.zeros((n, nj)), np.zeros((n, nj))
    for i in range(nj):
        qi, vi, ai, ti = float(q[i]), float(v[i]), float(a[i]), float(tgt[i])
        vmi, ami, ji = float(vm[i]), float(am[i]), float(jm[i])
        ks = _stop_consts(stop_decel * ami, ji)
        kw = _stop_consts(WALL_DECEL * ami, ji)
        for k in range(n):
            for _ in range(sub):
                e = ti - qi
                vs = _stop_speed(abs(e), ks, ji)
                vd = _clip(_clip(e / cycle, -vs, vs), -vmi, vmi)
                if lo_lim is not None:
                    dl, dh = qi - float(lo_lim[i]), float(hi_lim[i]) - qi
                    vd = _clip(vd, -_stop_speed(max(dl, 0.0), kw, ji),
                               _stop_speed(max(dh, 0.0), kw, ji))
                ad = _clip((vd - vi) / den, -ami, ami)
                hi = min(ami, math.sqrt(2.0 * ji * max(vmi - vi, 0.0)))
                lo = -min(ami, math.sqrt(2.0 * ji * max(vmi + vi, 0.0)))
                an = _clip(_clip(ad, lo, hi), ai - ji * hh, ai + ji * hh)
                qi = qi + vi * hh + 0.5 * an * hh * hh
                vi = _clip(vi + an * hh, -vmi, vmi)
                ai = an
            out_q[k, i], out_v[k, i], out_a[k, i] = qi, vi, ai
    return out_q, out_v, out_a


def stopping_tail(q, v, a, h, vmax, amax, jmax, max_s=3.0, qlim=None, stop_decel=0.8):
    """States after (q, v, a) that bring every joint to rest within the limits.

    Appended to each message, so the robot brakes on its own if the next message never comes.
    """
    q, v, a = np.asarray(q, float), np.asarray(v, float), np.asarray(a, float)
    nj = q.size
    am, jm = _per_joint(amax, nj), _per_joint(jmax, nj)
    hold = q + np.sign(v) * stop_distance(v, stop_decel * am, jm)
    chunk = max(1, int(round(0.1 / h)))
    parts = []
    for _ in range(int(np.ceil(max_s / (chunk * h)))):
        part = track_target(q, v, a, hold, chunk, h, vmax, amax, jmax, cycle=4.0 * h,
                            stop_decel=stop_decel, qlim=qlim)
        parts.append(part)
        q, v, a = part[0][-1], part[1][-1], part[2][-1]
        if np.max(np.abs(v)) < 1e-4 and np.max(np.abs(a)) < 1e-3:
            break
    return tuple(np.vstack([p[k] for p in parts]) for k in range(3))

"""Check the finite-acquisition proposition's assumptions against the runs.

The paper's finite-acquisition proposition guarantees the supervised
controller accumulates the spread threshold S_bar within an explicit time
T* under four hypotheses on the controller u = u_seek + u_exp:

  (i)   packets stored with inter-sample times in [T_, Tbar];
  (ii)  projected seeking during underexcited operation,
        u_seek(t)' * n(t) >= -A*exp(-lambda*Tbar)/pi with
        n(t) = (-1)^floor(omega*t/pi) * e_y, enforced by construction
        (Algorithm 1 projects -k(q - p_hat) onto that half-space);
  (iii) sampling fast relative to the excitation,
        8*omega*Tbar <= exp(-lambda*Tbar);
  (iv)  the stored window is cumulative (no eviction).

Module responsibility: this script does no simulation of its own. It
re-derives every number in the paper's assumptions-versus-implementation
discussion from the committed per-packet run logs in `results/` (written by
the C++ binary) plus the controller constants of `config/simulation.ini`,
so the check is empirical rather than nominal. For row i of a run log, the
control at step i used the pre-update position of row i-1 and the target
estimate of row i; underexcited packets are the rows with retriggered = 1,
and on those packets the epoch was reset, so u_exp acts at full amplitude
with absolute phase omega*(step-1)*dt. The projection invariant (ii) is
reconstructed from u = dq/dt (exact under the Euler update) minus u_exp,
with a tolerance covering the 8-decimal CSV quantization.

Expected findings (asserted at the bottom):
  - (i) holds exactly (fixed dt), (iv) holds by construction;
  - (iii) holds across the entire decay sweep, first failing only beyond
    lambda ~ 15.6 1/s; it also implies the proof's sliver inequality
    2*(omega*Tbar)^2 <= exp(-lambda*Tbar) with a wide margin;
  - (ii) holds on every underexcited packet of every supervised log.
    The projection exists because the unprojected law would violate the
    bound: the steady swirl orbit A/sqrt(k^2 + omega^2) = 19.5 cm gives
    pi*k*||q - p_hat|| = 3.45x the allowance independent of A, i.e. an
    unclipped seeking gain would need k <= 0.127 1/s where 1.2 is
    implemented. The clip binds on 15 of the 31 underexcited packets of
    the no-transient run, and the freed orbit opens to a 23.6 cm plateau;
  - acquisition completes at packet 32 (2.5 s), a factor of 22 inside
    T* <= 55.9 s at lambda = 2 under the two-packets-per-half-period bound
    T* <= (pi/omega)*(ceil(2*S_bar/delta^2) + 2), delta = A*exp(-lambda*
    Tbar)/(2*omega); the remaining conservatism is structural, since the
    proof credits only two stored packets per ~87-packet half-period and
    treats the projected seeking as an adversarial disturbance.
"""

from __future__ import annotations

import csv
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT / "results"

# Controller constants (config/simulation.ini).
K = 1.2        # closed_loop_control_gain [1/s]
A = 0.25       # exploration_amplitude [m/s]
OMEGA = 0.45   # exploration_frequency [rad/s]
DT = 0.08      # closed_loop_dt = Tbar = T_ [s]
SBAR = 0.16    # supervised_spread_threshold [m^2]

SWEEP_LAMBDAS = [0.02, 0.05, 0.10, 0.25, 0.50, 1.0, 2.0]

# One CSV position digit is 1e-8 m; dividing dq by dt = 0.08 s inflates the
# rounding to 1.25e-7 m/s, so 1e-6 cleanly separates real violations from
# quantization while staying five orders below the projection level b.
QUANT_TOL = 1e-6


def load(name: str) -> list[dict[str, float | None]]:
    with (RESULTS / name).open(newline="") as handle:
        return [
            {key: (float(value) if value != "" else None) for key, value in row.items()}
            for row in csv.DictReader(handle)
        ]


def tstar_seconds(lam: float) -> tuple[float, float]:
    """T* bound and its per-half-period displacement delta at decay lam."""
    delta = A * math.exp(-lam * DT) / (2.0 * OMEGA)
    n = math.ceil(2.0 * SBAR / delta**2)
    return (math.pi / OMEGA) * (n + 2), delta


def underexcited_packets(rows: list[dict], lam: float) -> list[dict]:
    """Per retriggered packet: step, ||q_pre - p_hat||, and the projection
    margin u_seek' n + b (>= 0 up to quantization iff (ii) holds)."""
    b = A * math.exp(-lam * DT) / math.pi
    out = []
    for i in range(1, len(rows)):
        if rows[i]["retriggered"] != 1.0:
            continue
        r, prev = rows[i], rows[i - 1]
        t = (r["step"] - 1) * DT
        u_y = (r["robot_y"] - prev["robot_y"]) / DT
        seek_y = u_y - A * math.sin(OMEGA * t)   # epoch reset => full amplitude
        sign = 1.0 if math.floor(OMEGA * t / math.pi) % 2 == 0 else -1.0
        out.append({
            "step": int(r["step"]),
            "dist": math.hypot(prev["robot_x"] - r["target_estimate_x"],
                               prev["robot_y"] - r["target_estimate_y"]),
            "margin": sign * seek_y + b,
            "binding": abs(sign * seek_y + b) <= QUANT_TOL,
        })
    return out


def main() -> None:
    print("Controller constants: "
          f"k={K} 1/s, A={A} m/s, omega={OMEGA} rad/s, Tbar=dt={DT} s, Sbar={SBAR}")
    print("(i)  holds exactly: fixed inter-sample time dt, so T_ = Tbar = dt.")
    print("(iv) holds by construction: the stored window only grows.")

    print("\n(iii) 8*omega*Tbar <= exp(-lambda*Tbar) across the decay sweep:")
    lhs = 8.0 * OMEGA * DT
    for lam in SWEEP_LAMBDAS:
        rhs = math.exp(-lam * DT)
        assert lhs <= rhs
        print(f"  lambda={lam:>5}: {lhs:.3f} <= {rhs:.3f}  PASS")
    lam_break = math.log(1.0 / lhs) / DT
    print(f"  first failure beyond lambda = {lam_break:.2f} 1/s")
    sliver = 2.0 * (OMEGA * DT) ** 2
    print(f"  implied sliver bound 2*(omega*Tbar)^2 = {sliver:.5f} <= "
          f"exp(-lambda*Tbar) >= {math.exp(-SWEEP_LAMBDAS[-1] * DT):.3f}  PASS")
    assert sliver <= math.exp(-SWEEP_LAMBDAS[-1] * DT)

    lam = 2.0  # the no-transient (understimulated) showcase decay
    tstar, delta = tstar_seconds(lam)
    print(f"\nT* bound at lambda={lam}: delta={delta:.3f}, "
          f"T* <= {tstar:.1f} s ({tstar / DT:.0f} packets)")

    rows = load("closed_loop_showcase_understimulated_supervised.csv")
    under = underexcited_packets(rows, lam)
    steps = [p["step"] for p in under]
    assert steps == list(range(1, len(steps) + 1)), "retriggers not a contiguous prefix"
    clear_packet = steps[-1] + 1
    clear_time = (clear_packet - 1) * DT
    b = A * math.exp(-lam * DT) / math.pi
    worst_margin = min(p["margin"] for p in under)
    binding = sum(1 for p in under if p["binding"])
    orbit = A / math.sqrt(K**2 + OMEGA**2)       # unprojected steady orbit
    d_max = max(math.hypot(rows[i - 1]["robot_x"] - rows[i]["target_estimate_x"],
                           rows[i - 1]["robot_y"] - rows[i]["target_estimate_y"])
                for i in range(1, len(rows)))
    structural = math.pi * K / (math.sqrt(K**2 + OMEGA**2) * math.exp(-lam * DT))
    half_period_packets = math.pi / (OMEGA * DT)

    print("\nNo-transient supervised run "
          "(closed_loop_showcase_understimulated_supervised.csv):")
    print(f"  {len(under)} underexcited packets (contiguous prefix), threshold "
          f"clears at packet {clear_packet} ({clear_time:.2f} s), "
          f"{tstar / clear_time:.1f}x inside T*")
    print(f"  (ii) projection level b = A*exp(-lambda*Tbar)/pi = {b:.4f} m/s; "
          f"worst margin u_seek' n + b = {worst_margin:.2e} (>= -{QUANT_TOL:.0e})")
    print(f"  clip binds on {binding}/{len(under)} underexcited packets; "
          f"unprojected orbit A/sqrt(k^2+omega^2) = {orbit * 100:.1f} cm opens to "
          f"a measured {d_max * 100:.1f} cm plateau")
    gain_cap = OMEGA / math.sqrt(math.pi**2 * math.exp(2.0 * lam * DT) - 1.0)
    print(f"  why the projection exists: the unprojected law gives "
          f"pi*k*||q - p_hat|| = {structural:.2f}x the allowance at the steady "
          f"orbit (amplitude-independent), i.e. an unclipped gain cap of "
          f"k <= {gain_cap:.3f} 1/s against the implemented k = {K}")
    print(f"  half-period holds {half_period_packets:.0f} packets; the proof "
          f"credits 2 stored packets per half-period")

    print("\nProjection invariant on every supervised log with retriggers:")
    worst_all = worst_margin
    for name in (["closed_loop_local_1beacon.csv"]
                 + [f"closed_loop_showcase_seeking_ring_{i}.csv" for i in range(6)]):
        lam_run = 0.50 if name == "closed_loop_local_1beacon.csv" else 2.0
        run = underexcited_packets(load(name), lam_run)
        if not run:
            continue
        wm = min(p["margin"] for p in run)
        worst_all = min(worst_all, wm)
        print(f"  {name}: {len(run)} underexcited packets, "
              f"worst margin {wm:.2e}, max ||q - p_hat|| = "
              f"{max(p['dist'] for p in run) * 100:.0f} cm")

    assert worst_all >= -QUANT_TOL, "projection invariant violated"
    assert binding == 15 and len(under) == 31
    assert abs(d_max - 0.236) < 0.005
    assert abs(structural - 3.45) < 0.05
    assert clear_packet == 32
    assert abs(tstar - 55.9) < 0.1
    assert tstar / clear_time > 20.0
    print("\nAll expected findings hold.")


if __name__ == "__main__":
    main()

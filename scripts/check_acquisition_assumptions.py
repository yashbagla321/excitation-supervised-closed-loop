"""Check the finite-acquisition proposition's assumptions against the runs.

The paper's finite-acquisition proposition guarantees the supervised
controller accumulates the spread threshold S_bar in finite time T* under
four assumptions on the controller u = -k(q - p_hat) + u_exp:

  (i)   packets stored with inter-sample times in [T_, Tbar];
  (ii)  bounded seeking during underexcited operation,
        k*||q - p_hat|| <= D with pi*D <= A*exp(-lambda*Tbar);
  (iii) sampling fast relative to the excitation,
        8*omega*Tbar <= exp(-lambda*Tbar);
  (iv)  the stored window is cumulative (no eviction).

Module responsibility: this script does no simulation of its own. It
re-derives every number in the paper's assumptions-versus-implementation
discussion from the committed per-packet run logs in `results/` (written by
the C++ binary) plus the controller constants of `config/simulation.ini`,
so the check is empirical rather than nominal. For row i of a run log, the
control at step i used the pre-update position of row i-1 and the target
estimate of row i; underexcited packets are the rows with retriggered = 1.

Expected findings (asserted at the bottom):
  - (i) holds exactly (fixed dt), (iv) holds by construction;
  - (iii) holds across the entire decay sweep, first failing only beyond
    lambda ~ 15.6 1/s;
  - (ii) fails once the swirl orbit opens: it caps ||q - p_hat|| at
    A*exp(-lambda*Tbar)/(pi*k) = 5.7 cm (lambda = 2), while the steady
    orbit radius is A/sqrt(k^2 + omega^2) = 19.5 cm, so the amplitude
    cancels and (ii) reduces to pi*k <= sqrt(k^2 + omega^2)*exp(-lambda*Tbar),
    a seeking-gain cap of k <= 0.127 1/s that the implemented k = 1.2
    exceeds tenfold. The no-transient run satisfies (ii) on exactly its
    first four underexcited packets and peaks at 3.4x the allowance;
  - acquisition still completes at packet 37 (2.9 s), more than thirty
    times inside T* <= 97.7 s, because the proof treats seeking as an
    adversarial disturbance (the actual term is centripetal about p_hat)
    and credits only two stored packets per ~175-packet excitation period.
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
    return (2.0 * math.pi / OMEGA) * (n + 1), delta


def underexcited_seek_distances(rows: list[dict]) -> list[tuple[int, float]]:
    """(step, ||q_pre - p_hat||) for every retriggered packet of a run."""
    out = []
    for i in range(1, len(rows)):
        if rows[i]["retriggered"] == 1.0:
            out.append((
                int(rows[i]["step"]),
                math.hypot(rows[i - 1]["robot_x"] - rows[i]["target_estimate_x"],
                           rows[i - 1]["robot_y"] - rows[i]["target_estimate_y"]),
            ))
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

    lam = 2.0  # the no-transient (understimulated) showcase decay
    tstar, delta = tstar_seconds(lam)
    print(f"\nT* bound at lambda={lam}: delta={delta:.3f}, "
          f"T* <= {tstar:.1f} s ({tstar / DT:.0f} packets)")

    rows = load("closed_loop_showcase_understimulated_supervised.csv")
    under = underexcited_seek_distances(rows)
    steps = [s for s, _ in under]
    assert steps == list(range(1, len(steps) + 1)), "retriggers not a contiguous prefix"
    clear_packet = steps[-1] + 1
    clear_time = (clear_packet - 1) * DT
    allowance = A * math.exp(-lam * DT)          # cap on pi*D
    d_cap = allowance / (math.pi * K)            # implied cap on ||q - p_hat||
    orbit = A / math.sqrt(K**2 + OMEGA**2)       # steady swirl-orbit radius
    d_max = max(d for _, d in under)
    satisfying = [s for s, d in under if math.pi * K * d <= allowance]
    peak_ratio = math.pi * K * d_max / allowance
    structural = math.pi * K / (math.sqrt(K**2 + OMEGA**2) * math.exp(-lam * DT))
    period_packets = 2.0 * math.pi / (OMEGA * DT)

    print("\nNo-transient supervised run "
          "(closed_loop_showcase_understimulated_supervised.csv):")
    print(f"  {len(under)} underexcited packets (contiguous prefix), threshold "
          f"clears at packet {clear_packet} ({clear_time:.2f} s), "
          f"{tstar / clear_time:.1f}x inside T*")
    print(f"  (ii) caps ||q - p_hat|| at {d_cap * 100:.1f} cm; steady orbit radius "
          f"A/sqrt(k^2+omega^2) = {orbit * 100:.1f} cm; measured plateau "
          f"{d_max * 100:.1f} cm")
    print(f"  (ii) satisfied on packets {satisfying} only; peak "
          f"pi*k*||q - p_hat|| = {math.pi * K * d_max:.2f} against "
          f"A*exp(-lambda*Tbar) = {allowance:.2f} ({peak_ratio:.2f}x)")
    print(f"  steady-orbit prediction pi*k/(sqrt(k^2+omega^2)*exp(-lambda*Tbar)) "
          f"= {structural:.2f}")
    print(f"  gain cap from (ii): k <= "
          f"{math.sqrt(allowance**2 / A**2 * OMEGA**2 / (math.pi**2 - allowance**2 / A**2)):.3f} 1/s"
          f"  (implemented k = {K})")
    print(f"  excitation period holds {period_packets:.0f} packets; the proof "
          f"credits 2; half-period = {period_packets / 2:.0f} packets > "
          f"{len(under)} underexcited packets")

    print("\nTransient runs (for contrast; the transit itself supplies spread):")
    for name in ["closed_loop_local_1beacon.csv"] + [
            f"closed_loop_showcase_seeking_ring_{i}.csv" for i in range(6)]:
        run = underexcited_seek_distances(load(name))
        if run:
            print(f"  {name}: {len(run)} underexcited packets, "
                  f"max ||q - p_hat|| = {max(d for _, d in run) * 100:.0f} cm "
                  "(far outside (ii); certification arrives from the transit)")

    assert satisfying == [1, 2, 3, 4]
    assert abs(peak_ratio - 3.44) < 0.05
    assert abs(structural - 3.45) < 0.05
    assert clear_packet == 37
    assert tstar / clear_time > 30.0
    print("\nAll expected findings hold.")


if __name__ == "__main__":
    main()

"""
Offline PID tuning simulator.

Runs the REAL PIDController class (pid/pid_controller.py) against a generic
lightly-damped second-order plant (mass-spring-damper style), so you can
compare kp/ki/kd combinations and see overshoot / settling time / steady
state error before touching the real robot.

Scope and limits — please read:
- This tests the PIDController class itself (does a given kp/ki/kd combo
  converge nicely against a generic system with inertia?). It does NOT model
  your actual robot's wheels, GPS bearing math, or the sign conventions used
  in move_to_target() — that mapping (distance/cap -> PID error -> PWM) is a
  separate question from "is this a well-behaved set of PID coefficients?".
- The plant model is the same generic 2nd-order system your existing
  pid_plot.py already used (y'' = u - c1*y' - c2*y), just wired to your real
  PIDController instead of a hand-rolled PID formula, and made comparable
  across coefficient sets and noise levels.
- Treat the results as a starting point, not a final answer: validate on the
  real robot afterward, indoors, at low speed, with a way to stop it.

Examples
--------
Compare three coefficient sets on a clean (noise-free) system:

    python3 pid/simulate_pid.py --kp 1,2,4 --ki 0 --kd 0,0.5,2

Same, but with simulated GPS-like measurement noise:

    python3 pid/simulate_pid.py --kp 2 --ki 0.1 --kd 0.8 --noise 0.05

Save the plot instead of (or in addition to) showing it:

    python3 pid/simulate_pid.py --kp 2 --ki 0.1 --kd 0.8 --output run1.png
"""
import argparse
import itertools
import sys

import numpy as np
import matplotlib

import pid_controller
from pid_controller import PIDController


class _FakeClock:
    """Stand-in for time.time() so the simulation can drive PIDController
    with a controlled, reproducible clock instead of the real wall clock
    (PIDController.update() calls time.time() internally to compute dt)."""

    def __init__(self):
        self.now = 0.0

    def time(self):
        return self.now


# Replace the `time` module reference *inside pid_controller* with our fake
# clock before creating any PIDController, so self.last_time starts at 0.0
# instead of the real epoch time.
_fake_clock = _FakeClock()
pid_controller.time = _fake_clock


def parse_float_list(text):
    """Parse "1,2,4" or "1" into a list of floats."""
    return [float(v.strip()) for v in text.split(",") if v.strip() != ""]


def simulate_one(kp, ki, kd, setpoint, y0, duration, dt, c1, c2, noise_std, seed):
    """
    Run one simulation and return (t, y_true) plus computed metrics.

    Plant: y'' = u - c1*y' - c2*y   (u = PID output, a generic 2nd-order
    system with damping c1 and stiffness c2 — same shape as pid_plot.py).
    The PID only ever "sees" a noisy measurement of y, exactly like your
    real distance/angle readings will be noisy.

    y0 is the starting value of y. To mirror your real robot's convention
    (setpoint=0, distance/angle shrinking toward it as the robot approaches),
    use --y0 5 --setpoint 0, for example, instead of the generic --y0 0
    --setpoint 10 demo shape.
    """
    rng = np.random.default_rng(seed)
    n_steps = int(duration / dt)

    _fake_clock.now = 0.0
    pid = PIDController(setpoint=setpoint, kp=kp, ki=ki, kd=kd)

    y = y0
    v = 0.0
    t = 0.0

    ts = np.zeros(n_steps)
    ys = np.zeros(n_steps)

    for i in range(n_steps):
        measured = y + (rng.normal(0.0, noise_std) if noise_std > 0 else 0.0)

        t += dt
        _fake_clock.now = t  # PIDController.update() reads this via time.time()
        u = pid.update(measured)

        # Second-order plant integration (semi-implicit Euler).
        v_dot = u - c1 * v - c2 * y
        v += v_dot * dt
        y += v * dt

        ts[i] = t
        ys[i] = y

    metrics = compute_metrics(ts, ys, setpoint, y0)
    return ts, ys, metrics


def compute_metrics(ts, ys, setpoint, y0, band=0.02):
    """Overshoot %, settling time (within +/-band of setpoint), steady-state error.

    Overshoot is expressed relative to the initial error (setpoint - y0), so
    it is meaningful whether you approach from below (y0=0, setpoint=10) or
    from above (y0=5, setpoint=0, as in your real distance/angle loops)."""
    initial_error = setpoint - y0
    if abs(initial_error) < 1e-9:
        # Already at the setpoint at t=0; report absolute drift instead.
        overshoot_pct = float(np.max(np.abs(ys - setpoint)))
        tolerance = band
    elif initial_error > 0:
        # Rising step: overshoot = going above the setpoint.
        peak = float(np.max(ys))
        overshoot_pct = max(0.0, (peak - setpoint) / initial_error * 100.0)
        tolerance = abs(initial_error) * band
    else:
        # Falling step: overshoot = going below the setpoint.
        peak = float(np.min(ys))
        overshoot_pct = max(0.0, (setpoint - peak) / (-initial_error) * 100.0)
        tolerance = abs(initial_error) * band

    settled_mask = np.abs(ys - setpoint) <= max(tolerance, 1e-6)
    settling_time = None
    for i in range(len(ys) - 1, -1, -1):
        if not settled_mask[i]:
            if i + 1 < len(ts):
                settling_time = ts[i + 1]
            break
    else:
        settling_time = ts[0]
    if settling_time is None:
        settling_time = float("inf")  # never settled within the simulated duration

    steady_state_error = abs(setpoint - ys[-1])

    return {
        "overshoot_pct": overshoot_pct,
        "settling_time_s": settling_time,
        "steady_state_error": steady_state_error,
    }


def main():
    parser = argparse.ArgumentParser(description="Simulate and compare PID coefficient sets.")
    parser.add_argument("--kp", type=str, default="1", help="Comma-separated kp values, e.g. 1,2,4")
    parser.add_argument("--ki", type=str, default="0", help="Comma-separated ki values")
    parser.add_argument("--kd", type=str, default="0", help="Comma-separated kd values")
    parser.add_argument("--setpoint", type=float, default=10.0, help="Target value (default: 10)")
    parser.add_argument("--y0", type=float, default=0.0,
                         help="Starting value of y (default: 0). Use --y0 5 --setpoint 0 to "
                              "mirror your real distance/angle loops, which start away from "
                              "the target and are driven down to a setpoint of 0.")
    parser.add_argument("--ylabel", type=str, default="Valeur (y)",
                         help='Y-axis label, e.g. "Distance restante (m)" or "Ecart angle (deg)"')
    parser.add_argument("--title", type=str, default="Comparaison de reglages PID (simulation)",
                         help="Plot title")
    parser.add_argument("--duration", type=float, default=10.0, help="Simulated seconds (default: 10)")
    parser.add_argument("--dt", type=float, default=0.01, help="Time step in seconds (default: 0.01)")
    parser.add_argument("--c1", type=float, default=2.0, help="Plant damping coefficient (default: 2)")
    parser.add_argument("--c2", type=float, default=3.0, help="Plant stiffness coefficient (default: 3)")
    parser.add_argument("--noise", type=float, default=0.0, help="Std dev of measurement noise (default: 0, clean)")
    parser.add_argument("--seed", type=int, default=42, help="Random seed for noise (default: 42)")
    parser.add_argument("--output", type=str, default=None, help="Save plot to this PNG file instead of only showing it")
    args = parser.parse_args()

    kps = parse_float_list(args.kp)
    kis = parse_float_list(args.ki)
    kds = parse_float_list(args.kd)

    # If lists have different lengths, either they must all match, or the
    # shorter ones are broadcast (single value repeated).
    lengths = {len(kps), len(kis), len(kds)}
    lengths.discard(1)
    n_runs = max(lengths) if lengths else 1
    if len(kps) == 1:
        kps = kps * n_runs
    if len(kis) == 1:
        kis = kis * n_runs
    if len(kds) == 1:
        kds = kds * n_runs
    if not (len(kps) == len(kis) == len(kds) == n_runs):
        print("[ERREUR] --kp/--ki/--kd must have the same length, or length 1 to broadcast.", file=sys.stderr)
        sys.exit(1)

    if args.output:
        matplotlib.use("Agg")  # headless-safe backend for saving to file
    import matplotlib.pyplot as plt  # noqa: E402  (backend must be set first)

    plt.figure(figsize=(9, 5))
    plt.axhline(args.setpoint, color="gray", linestyle="--", linewidth=1, label="setpoint")

    print(f"{'kp':>6} {'ki':>6} {'kd':>6} | {'overshoot %':>12} {'settling (s)':>13} {'steady err':>11}")
    print("-" * 62)

    for kp, ki, kd in zip(kps, kis, kds):
        ts, ys, metrics = simulate_one(
            kp, ki, kd, args.setpoint, args.y0, args.duration, args.dt,
            args.c1, args.c2, args.noise, args.seed,
        )
        label = f"kp={kp:g} ki={ki:g} kd={kd:g}"
        plt.plot(ts, ys, label=label)
        settling_str = f"{metrics['settling_time_s']:.2f}" if metrics["settling_time_s"] != float("inf") else "never"
        print(f"{kp:6g} {ki:6g} {kd:6g} | {metrics['overshoot_pct']:12.1f} {settling_str:>13} {metrics['steady_state_error']:11.3f}")

    plt.xlabel("Temps (s)")
    plt.ylabel(args.ylabel)
    plt.title(args.title)
    plt.legend()
    plt.grid(True, alpha=0.3)

    if args.output:
        plt.savefig(args.output, dpi=150)
        print(f"\nGraphique enregistré dans : {args.output}")
    else:
        plt.show()


if __name__ == "__main__":
    main()

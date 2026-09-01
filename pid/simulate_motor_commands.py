"""
Full-pipeline PID simulator: distance PID + angle PID -> motor PWM commands.

Unlike simulate_pid.py (which tunes ONE loop in isolation, on an abstract
y axis), this script reproduces the second half of move_to_target() from
pid_controller.py: it takes the (clamped) speed / angular_velocity coming
out of both PIDs and mixes them into left_speed / right_speed exactly the
way move_to_target() does today, including the +/-255 clamp applied to
left_speed/right_speed after mixing. The +/-255 reference lines are kept on
the plot as a sanity check -- if a curve still touches them, the PID gains
themselves are asking for full power, which is worth knowing even though the
code no longer sends an out-of-range value to motor_control/pwm.py.

The distance and angle channels are each driven by the same generic
2nd-order plant used in simulate_pid.py (inertia + light damping), but here
the forcing term is the CLAMPED speed / angular_velocity (post move_to_target
saturation), not the raw PID output -- so you also see the effect of the
+/-2.0 m/s and +/-360 deg/s limits, not just the PID gains.

Example
-------
    python3 pid/simulate_motor_commands.py \\
        --dist-kp 1 --dist-kd 0.5 \\
        --angle-kp 0.5 --angle-kd 1 \\
        --output motor_commands.png
"""
import argparse

import numpy as np
import matplotlib

import pid_controller
from pid_controller import PIDController


class _FakeClock:
    """Same trick as simulate_pid.py: PIDController.update() calls time.time()
    internally, so we give it a controlled, reproducible clock instead."""

    def __init__(self):
        self.now = 0.0

    def time(self):
        return self.now


_fake_clock = _FakeClock()
pid_controller.time = _fake_clock

# Constants copied from move_to_target() in pid_controller.py, so the mixing
# math here matches the real robot code exactly.
MAX_SPEED = 2.0               # m/s
MAX_ANGULAR_VELOCITY = 360.0  # deg/s
PWM_LIMIT = 255.0


def simulate(dist_gains, angle_gains, dist_y0, angle_y0,
             duration, dt, c1, c2, noise_std, seed):
    """
    dist_gains / angle_gains: (kp, ki, kd) tuples.
    Returns a dict of numpy arrays: t, distance, angle, speed,
    angular_velocity, left_speed, right_speed.
    """
    rng = np.random.default_rng(seed)
    n_steps = int(duration / dt)

    _fake_clock.now = 0.0
    dist_pid = PIDController(setpoint=0.0, kp=dist_gains[0], ki=dist_gains[1], kd=dist_gains[2])
    angle_pid = PIDController(setpoint=0.0, kp=angle_gains[0], ki=angle_gains[1], kd=angle_gains[2])

    distance, distance_v = dist_y0, 0.0
    angle, angle_v = angle_y0, 0.0
    t = 0.0

    out = {k: np.zeros(n_steps) for k in
           ["t", "distance", "angle", "speed", "angular_velocity", "left_speed", "right_speed"]}

    for i in range(n_steps):
        t += dt
        _fake_clock.now = t

        meas_distance = distance + (rng.normal(0.0, noise_std) if noise_std > 0 else 0.0)
        meas_angle = angle + (rng.normal(0.0, noise_std) if noise_std > 0 else 0.0)

        # --- this block mirrors move_to_target() in pid_controller.py ---
        speed = dist_pid.update(meas_distance)
        angular_velocity = angle_pid.update(meas_angle)
        speed = max(min(speed, MAX_SPEED), -MAX_SPEED)
        angular_velocity = max(min(angular_velocity, MAX_ANGULAR_VELOCITY), -MAX_ANGULAR_VELOCITY)

        speed_pwm = (speed / MAX_SPEED) * 255
        angular_pwm = (angular_velocity / MAX_ANGULAR_VELOCITY) * 255

        left_speed = speed_pwm + angular_pwm
        right_speed = speed_pwm - angular_pwm
        left_speed = max(min(left_speed, PWM_LIMIT), -PWM_LIMIT)
        right_speed = max(min(right_speed, PWM_LIMIT), -PWM_LIMIT)
        # --- end of move_to_target() logic (now clamped to +/-255, matching
        #     the fix applied in pid_controller.py) ---

        # Generic 2nd-order plant per channel (same shape as simulate_pid.py),
        # forced by the CLAMPED speed / angular_velocity.
        distance_v += (speed - c1 * distance_v - c2 * distance) * dt
        distance += distance_v * dt

        angle_v += (angular_velocity - c1 * angle_v - c2 * angle) * dt
        angle += angle_v * dt

        out["t"][i] = t
        out["distance"][i] = distance
        out["angle"][i] = angle
        out["speed"][i] = speed
        out["angular_velocity"][i] = angular_velocity
        out["left_speed"][i] = left_speed
        out["right_speed"][i] = right_speed

    return out


def parse_gains(kp, ki, kd):
    return (kp, ki, kd)


def main():
    parser = argparse.ArgumentParser(
        description="Simulate distance+angle PIDs mixed into left/right motor PWM commands."
    )
    parser.add_argument("--dist-kp", type=float, default=1.0)
    parser.add_argument("--dist-ki", type=float, default=0.0)
    parser.add_argument("--dist-kd", type=float, default=0.0)
    parser.add_argument("--angle-kp", type=float, default=0.5)
    parser.add_argument("--angle-ki", type=float, default=0.0)
    parser.add_argument("--angle-kd", type=float, default=0.0)
    parser.add_argument("--dist-y0", type=float, default=5.0, help="Starting distance in meters (default: 5)")
    parser.add_argument("--angle-y0", type=float, default=45.0, help="Starting angle error in degrees (default: 45)")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--dt", type=float, default=0.01)
    parser.add_argument("--c1", type=float, default=2.0, help="Plant damping (default: 2)")
    parser.add_argument("--c2", type=float, default=3.0, help="Plant stiffness (default: 3)")
    parser.add_argument("--noise", type=float, default=0.0, help="Std dev of measurement noise (default: 0)")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--output", type=str, default=None, help="Save plot to this PNG instead of showing it")
    args = parser.parse_args()

    if args.output:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt  # noqa: E402

    out = simulate(
        parse_gains(args.dist_kp, args.dist_ki, args.dist_kd),
        parse_gains(args.angle_kp, args.angle_ki, args.angle_kd),
        args.dist_y0, args.angle_y0,
        args.duration, args.dt, args.c1, args.c2, args.noise, args.seed,
    )

    max_left = float(np.max(np.abs(out["left_speed"])))
    max_right = float(np.max(np.abs(out["right_speed"])))
    sat_time_left = float(np.sum(np.abs(out["left_speed"]) > PWM_LIMIT) * args.dt)
    sat_time_right = float(np.sum(np.abs(out["right_speed"]) > PWM_LIMIT) * args.dt)

    print(f"max |left_speed|  = {max_left:.1f}  (limite : {PWM_LIMIT:.0f})")
    print(f"max |right_speed| = {max_right:.1f}  (limite : {PWM_LIMIT:.0f})")
    if max_left > PWM_LIMIT or max_right > PWM_LIMIT:
        print(f"[ATTENTION] Saturation detectee : left hors limite pendant {sat_time_left:.2f}s, "
              f"right hors limite pendant {sat_time_right:.2f}s "
              f"(sur {args.duration:.0f}s simulees). left_speed/right_speed ne sont pas "
              f"bornes dans move_to_target() -- voir le README.")
    else:
        print("Pas de saturation : left_speed/right_speed restent dans +/-255 sur toute la simulation.")

    fig, axes = plt.subplots(3, 1, figsize=(9, 10), sharex=True)

    axes[0].axhline(0, color="gray", linestyle="--", linewidth=1, label="setpoint")
    axes[0].plot(out["t"], out["distance"], color="tab:blue")
    axes[0].set_ylabel("Distance restante (m)")
    axes[0].set_title(f"Distance -- kp={args.dist_kp:g} ki={args.dist_ki:g} kd={args.dist_kd:g}")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()

    axes[1].axhline(0, color="gray", linestyle="--", linewidth=1, label="setpoint")
    axes[1].plot(out["t"], out["angle"], color="tab:orange")
    axes[1].set_ylabel("Ecart angle (deg)")
    axes[1].set_title(f"Angle -- kp={args.angle_kp:g} ki={args.angle_ki:g} kd={args.angle_kd:g}")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()

    axes[2].axhline(PWM_LIMIT, color="red", linestyle="--", linewidth=1, label="limite +/-255")
    axes[2].axhline(-PWM_LIMIT, color="red", linestyle="--", linewidth=1)
    axes[2].plot(out["t"], out["left_speed"], color="tab:green", label="left_speed")
    axes[2].plot(out["t"], out["right_speed"], color="tab:purple", label="right_speed")
    axes[2].set_ylabel("Commande moteur (PWM)")
    axes[2].set_xlabel("Temps (s)")
    axes[2].set_title("Commandes moteur (left_speed / right_speed) -- bornees a +/-255")
    axes[2].grid(True, alpha=0.3)
    axes[2].legend()

    fig.tight_layout()

    if args.output:
        fig.savefig(args.output, dpi=150)
        print(f"\nGraphique enregistre dans : {args.output}")
    else:
        plt.show()


if __name__ == "__main__":
    main()

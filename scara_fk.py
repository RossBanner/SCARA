"""Task 1.B: SCARA forward kinematics, independent of Webots.

Geometry: Coursework_1_2627.pdf, pp. 15-16. All positions are in metres
relative to the top-level ScaraT6 frame. Active joint order is q1, q2, q4.
"""

import argparse
import json
import math

JOINT_NAMES = ("base_arm_motor", "arm_motor", "shaft_linear_motor")
JOINT_COLUMNS = ("q1_rad", "q2_rad", "q4_m")
JOINT_LIMITS = ((-0.73, 0.73), (-0.83, 0.83), (-0.200, 0.0))


def finite_vector(values, length=3):
    """Convert a vector and reject wrong sizes, NaN and infinity."""
    result = tuple(float(value) for value in values)
    if len(result) != length or not all(math.isfinite(v) for v in result):
        raise ValueError(f"Expected {length} finite numbers.")
    return result


def validate_command(joints):
    """Validate physical limits and the coursework's positive-elbow rule."""
    q = finite_vector(joints)
    for name, value, (lower, upper) in zip(JOINT_NAMES, q, JOINT_LIMITS):
        if not lower <= value <= upper:
            raise ValueError(f"{name}: {value} is outside [{lower}, {upper}].")
    if q[1] <= 0:
        raise ValueError("arm_motor (q2) must be strictly positive for this coursework.")
    return q


def forward_kinematics(joints):
    """Return (x, y, z) in the robot base frame for (q1, q2, q4).

    This mathematical function also accepts zero/negative elbow angles for
    diagnostics and sensor readings. Commands use validate_command separately.
    The ignored shaft_rotation_motor is not an input.

    p = [0.060, 0, 0.220] + Rz(q1) *
        ([0.310, 0.005, -0.004] + Rz(q2) * [0.2545, -0.005, q4-0.034])
    """
    q1, q2, q4 = finite_vector(joints)
    c1, s1 = math.cos(q1), math.sin(q1)
    c12, s12 = math.cos(q1 + q2), math.sin(q1 + q2)
    return (
        0.060 + 0.310 * c1 - 0.005 * s1 + 0.2545 * c12 + 0.005 * s12,
        0.310 * s1 + 0.005 * c1 + 0.2545 * s12 - 0.005 * c12,
        0.182 + q4,
    )


def world_to_base(point_world, base_position, base_orientation):
    """Transform a Webots world point using the base's measured pose.

    getOrientation() gives a row-major 3x3 rotation from base to world.
    Its transpose is the inverse: p_base = R.T * (p_world - t).
    """
    p = finite_vector(point_world)
    t = finite_vector(base_position)
    r = finite_vector(base_orientation, 9)
    delta = tuple(p[i] - t[i] for i in range(3))
    return tuple(sum(r[3 * j + i] * delta[j] for j in range(3)) for i in range(3))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("q1", type=float, help="Base angle in radians")
    parser.add_argument("q2", type=float, help="Positive elbow angle in radians")
    parser.add_argument("q4", type=float, help="Shaft displacement in metres")
    args = parser.parse_args()
    try:
        q = validate_command((args.q1, args.q2, args.q4))
    except ValueError as exc:
        parser.error(str(exc))
    print(json.dumps({"joint_order": JOINT_COLUMNS, "joints": q,
                      "frame": "ScaraT6 base", "tcp_m": forward_kinematics(q)}, indent=2))


if __name__ == "__main__":
    main()

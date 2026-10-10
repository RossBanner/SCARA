"""Task 1.B: SCARA TCP in metres, relative to the robot base."""
import math


def forward_kinematics(joints):
    q1, q2, q4 = joints
    return (
        0.060 + 0.310 * math.cos(q1) - 0.005 * math.sin(q1)
        + 0.2545 * math.cos(q1 + q2) + 0.005 * math.sin(q1 + q2),
        0.310 * math.sin(q1) + 0.005 * math.cos(q1)
        + 0.2545 * math.sin(q1 + q2) - 0.005 * math.cos(q1 + q2),
        0.182 + q4,
    )


if __name__ == "__main__":
    print(forward_kinematics((0.4, 0.6, -0.10)))

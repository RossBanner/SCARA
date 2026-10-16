import csv
from pathlib import Path
import random

from scara_fk import forward_kinematics

LIMITS = ((-0.73, 0.73), (0.0, 0.83), (-0.2, 0.0))
SIZES = (("small", 1000), ("medium", 5000), ("large", 20000))
HEADER = ("q1n", "q2n", "q3n", "x", "y", "z")


def main():
    rng = random.Random(42)
    seen_positions = set()

    def sample(count):
        rows = []
        while len(rows) < count:

            joints = (rng.uniform(-0.73, 0.73),
                      0.83 * (1.0 - rng.random()), rng.uniform(-0.2, 0.0))
            xyz = forward_kinematics(joints)
            if xyz in seen_positions:
                continue
            seen_positions.add(xyz)
            normalised = tuple(2 * (q - low) / (high - low) - 1
                               for q, (low, high) in zip(joints, LIMITS))
            rows.append((*normalised, *xyz))
        return rows

    pool = sample(20000)
    rng.shuffle(pool)

    train, validation = pool[:16000], pool[16000:]
    test = sample(1000)
    output = Path(__file__).resolve().parents[1] / "datasets"
    output.mkdir(exist_ok=True)

    def save(name, rows):
        with (output / name).open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(HEADER)
            writer.writerows(rows)

    for name, size in SIZES:
        save(f"scara_{name}_train.csv", train[:size * 4 // 5])
        save(f"scara_{name}_val.csv", validation[:size // 5])
    save("scara_test.csv", test)


if __name__ == "__main__":
    main()

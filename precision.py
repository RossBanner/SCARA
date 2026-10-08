"""Task 1.C: compare FK with genuine supervisor measurements.

No synthetic TCP values are substituted. Command-based error includes tracking
error; sensor-based error isolates geometry/frame mismatch more closely.
"""

import argparse
import csv
import json
import math
from statistics import fmean, pstdev

from scara_fk import finite_vector, forward_kinematics


def comparison_row(commanded, measured, tcp_base, settled, simulation_time):
    commanded = finite_vector(commanded)
    measured = finite_vector(measured)
    tcp_base = finite_vector(tcp_base)
    predicted = forward_kinematics(commanded)
    sensor_predicted = forward_kinematics(measured)
    row = {"status": "settled" if settled else "timeout",
           "simulation_time_s": simulation_time}
    for prefix, values, labels in (
        ("command", commanded, ("q1_rad", "q2_rad", "q4_m")),
        ("measured", measured, ("q1_rad", "q2_rad", "q4_m")),
        ("tcp_base", tcp_base, ("x_m", "y_m", "z_m")),
        ("fk_command", predicted, ("x_m", "y_m", "z_m")),
        ("fk_measured", sensor_predicted, ("x_m", "y_m", "z_m")),
    ):
        row.update({f"{prefix}_{label}": value for label, value in zip(labels, values)})
    row["command_position_error_m"] = math.dist(predicted, tcp_base)
    row["sensor_fk_position_error_m"] = math.dist(sensor_predicted, tcp_base)
    return row


def summarise(rows):
    rows = list(rows)
    if not rows:
        raise ValueError("No measurement rows found.")
    if any(row["status"] not in ("settled", "timeout") for row in rows):
        raise ValueError("Unknown measurement status.")
    usable = [row for row in rows if row["status"] == "settled"]
    result = {"samples": len(rows), "settled": len(usable),
              "timeouts": len(rows) - len(usable), "std_definition": "population (ddof=0)"}
    for key in ("command_position_error_m", "sensor_fk_position_error_m"):
        values = [float(row[key]) for row in usable]
        if any(not math.isfinite(v) or v < 0 for v in values):
            raise ValueError(f"Invalid values in {key}.")
        result[key] = ({"mean": fmean(values), "std": pstdev(values), "max": max(values)}
                       if values else None)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("measurements", help="CSV produced by the Webots controller")
    args = parser.parse_args()
    try:
        with open(args.measurements, newline="", encoding="utf-8") as stream:
            summary = summarise(csv.DictReader(stream))
    except (OSError, ValueError, KeyError) as exc:
        parser.error(str(exc))
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()

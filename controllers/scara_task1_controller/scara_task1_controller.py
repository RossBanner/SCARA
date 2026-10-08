"""Run SCARA Task 1 checks from Webots. Uses standard Python + Webots only."""

import argparse
import csv
from datetime import datetime, timezone
import json
from pathlib import Path
import sys

PROJECT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT))

from controller import Supervisor
from precision import comparison_row, summarise
from scara_fk import JOINT_COLUMNS, validate_command
from webots_scara import ScaraRig


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    inputs = parser.add_mutually_exclusive_group()
    inputs.add_argument("--joints", type=float, nargs=3, metavar=("Q1", "Q2", "Q4"))
    inputs.add_argument("--poses", default=None, help="CSV path relative to scara_task1")
    parser.add_argument("--output", help="New CSV path relative to scara_task1")
    parser.add_argument("--quit", action="store_true", help="Exit Webots after the check")
    args = parser.parse_args()
    robot = Supervisor()
    try:
        if args.joints is not None:
            poses = [validate_command(args.joints)]
        else:
            pose_path = PROJECT / (args.poses or "inputs/course_demo_poses.csv")
            with pose_path.open(newline="", encoding="utf-8-sig") as stream:
                poses = [validate_command([row[key] for key in JOINT_COLUMNS])
                         for row in csv.DictReader(stream)]
        if not poses:
            raise ValueError("The joint-input file contains no poses.")
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
        output = PROJECT / (args.output or f"results/development_check_{stamp}.csv")
        output.parent.mkdir(parents=True, exist_ok=True)
        rig = ScaraRig(robot)
        rows = []
        # Exclusive creation prevents accidentally replacing earlier measurements.
        with output.open("x", newline="", encoding="utf-8") as stream:
            writer = None
            for index, target in enumerate(poses, 1):
                measured, tcp, settled = rig.move_and_measure(target)
                row = comparison_row(target, measured, tcp, settled, robot.getTime())
                if writer is None:
                    writer = csv.DictWriter(stream, fieldnames=list(row))
                    writer.writeheader()
                writer.writerow(row)
                stream.flush()
                rows.append(row)
                print(f"[{index}/{len(poses)}] {row['status']} | "
                      f"command error={row['command_position_error_m']:.6g} m | "
                      f"sensor FK error={row['sensor_fk_position_error_m']:.6g} m", flush=True)
        summary = summarise(rows)
        report = {"purpose": "Task 1 development check; not Task 4 evaluation",
                  "tcp_frame": "ScaraT6 base", "measurement_source": "Webots Supervisor",
                  "python_version": sys.version,
                  "joint_order": list(JOINT_COLUMNS),
                  "input_file": args.poses or (None if args.joints else "inputs/course_demo_poses.csv"),
                  "summary": summary}
        with output.with_suffix(".json").open("x", encoding="utf-8") as stream:
            json.dump(report, stream, indent=2)
        print(f"Saved measurements: {output}", flush=True)
        print(json.dumps(summary, indent=2), flush=True)
        if args.quit:
            robot.simulationQuit(0 if summary["timeouts"] == 0 else 2)
        else:
            robot.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)
    except Exception as exc:
        print(f"SCARA check failed: {exc}", file=sys.stderr, flush=True)
        if args.quit:
            robot.simulationQuit(1)
        else:
            robot.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)
        raise


if __name__ == "__main__":
    main()

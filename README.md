# SCARA

SCARA contribution to Coursework 1, Task 1.A (Webots control), 1.B (forward
kinematics), and 1.C (precision comparison). This is an initial implementation,
not the complete group coursework submission. Broader validation and group
review remain necessary; Ned, datasets, neural-network training, Task 4,
poster, video and the final AI statement are not included.

## Run

Use Webots R2023b and Python 3.11 or later. The code uses Python's standard
library and the Webots controller API; it needs no additional Python packages.
Keep the folder structure intact.

1. Open `worlds/scara_coursework.wbt` in Webots and press Run.
2. The arm visits the four commands in `inputs/course_demo_poses.csv`.
3. The simulation pauses and saves measured positions and errors to new CSV
   and JSON files in `results/`. Reload the world to repeat the run.

The world downloads official R2023b PROTO models/assets on its first load
unless they are already cached, so an internet connection may be needed.

Run the independent Task 1.B calculation from the repository root:

```powershell
python scara_fk.py 0.4 0.6 -0.10
```

Recalculate a saved measurement summary using its actual filename:

```powershell
python precision.py results/development_check_YOUR_TIMESTAMP.csv
```

## Files

- `controllers/scara_task1_controller/scara_task1_controller.py`: orchestrates
  motor commands, measurements and output files.
- `webots_scara.py`: motor/sensor access, settling checks and measured TCP.
- `scara_fk.py`: standalone forward kinematics and joint validation.
- `precision.py`: compares FK with measured TCP and summarises errors.
- `controllers/tcp_pose/tcp_pose.py`: unchanged university supervisor script.
- `worlds/scara_coursework.wbt`: adapted university world with both controllers.
- `inputs/course_demo_poses.csv`: four development commands, not a Task 2 dataset.

## Inputs and measurements

Joint order is `(q1, q2, q4)`: `base_arm_motor` in radians, `arm_motor` in
radians, and `shaft_linear_motor` in metres. Command limits are [-0.73, 0.73],
(0, 0.83], and [-0.2, 0], respectively. Shaft rotation stays at zero. Negative
q4 lowers the shaft. Physical limits do not guarantee collision-free poses.

Positions and errors are in metres, relative to the SCARA base. The controller
reads the simulated TCP through the Supervisor API and transforms it into that
frame. The supplied supervisor independently prints the TCP position; the
logger does not scrape that rounded output.

The CSV records commanded/measured joints, measured TCP, FK predictions and
Euclidean errors. Command-based error includes motor tracking; sensor-based
FK error compares against actual joint readings. Summaries report settled-pose
mean, population standard deviation and maximum, with timeouts counted
separately. These are development checks, not Task 4 evaluation results.

Generated outputs are ignored by Git. Retain genuine results locally and
select the required evidence when preparing the final coursework submission.

## Sources and AI assistance

Geometry and requirements: Coursework_1_2627.pdf, pp. 9 and 15-16.
The world and supplied `tcp_pose.py` originate from the university repository:
https://gitlab-student.macs.hw.ac.uk/f2021ro_2627/nn-ik/
(commit `19d3d6836f0e37260e84b0d612c85ce42fd7bffe`).
The world changes assign the new arm controller, supply its pose-file argument,
and enable supervisor access on the arm; the TCP marker and observer remain.

The new Python implementation and this technical run guide were AI-assisted
using Codex. The group must review and understand the work and disclose the
assistance in its own required AI statement. This README is not that statement.

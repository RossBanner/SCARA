# SCARA Task 1

Requires Webots R2023b and Python 3.11 or later. No extra Python packages.

- Task 1.A: `controllers/scara_task1_controller/scara_task1_controller.py`.
- Task 1.B: `scara_fk.py` (standalone forward kinematics).
- Task 1.C: the controller compares FK with the measured TCP and writes
  `precision_results.csv` in the project root. Each run replaces that file.
- `controllers/tcp_pose/tcp_pose.py` is the unchanged supplied supervisor.
- `worlds/scara_coursework.wbt` connects the robot and controllers.

Open the world in Webots and press Run. It tests four poses and pauses.
Edit `POSES` in the controller to test other joint values. The order is base
angle (radians), positive elbow angle (radians), shaft displacement (metres).
Shaft rotation stays zero. Choose poses that avoid collisions with the floor.
Run `python scara_fk.py` from the project root for the separate FK example;
edit its example joint values as needed.

TCP positions and errors are in metres in the arm base frame. The controller
uses the same `TCP.getPose(ARM)` measurement as the supplied supervisor.
Joint limits and settling checks are retained so invalid or unfinished motion
is not reported as a successful precision check. Mean and maximum errors are
printed; the CSV contains each predicted and measured position and its error.
The four poses are an initial check, not validation of the entire workspace.

The world may need internet access on first load for official Webots assets.
This covers SCARA Task 1 only, not Ned or the remaining coursework tasks.

Sources: Coursework_1_2627.pdf pp. 9, 15-16 and the supplied world/supervisor
from https://gitlab-student.macs.hw.ac.uk/f2021ro_2627/nn-ik/
(commit 19d3d6836f0e37260e84b0d612c85ce42fd7bffe).
The new controller and FK code are AI-assisted (Codex). Review and disclose
this assistance in the group's required AI statement.

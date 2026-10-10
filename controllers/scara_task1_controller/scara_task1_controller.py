"""Task 1.A and 1.C: command SCARA and compare FK with the measured TCP."""
import csv
import math
from pathlib import Path
import sys
from controller import Supervisor

PROJECT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT))
from scara_fk import forward_kinematics

# Active joints: base angle (rad), positive elbow angle (rad), shaft (m).
POSES = [(0.0, 0.2, -0.05), (0.4, 0.6, -0.10),
         (-0.4, 0.4, -0.15), (0.7, 0.8, -0.16)]

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())
arm = robot.getFromDef("ARM")
tcp = robot.getFromDef("TCP")
motors = [robot.getDevice(name) for name in
          ("base_arm_motor", "arm_motor", "shaft_linear_motor", "shaft_rotation_motor")]
sensors = [motor.getPositionSensor() for motor in motors]
for sensor in sensors:
    sensor.enable(timestep)
for motor, speed in zip(motors, (1.0, 1.0, 0.05, 1.0)):
    motor.setVelocity(min(speed, motor.getMaxVelocity()))

# Use the same TCP and base-relative pose as the supplied tcp_pose supervisor.
def measured_tcp():
    pose = tcp.getPose(arm)
    return (pose[3], pose[7], pose[11])


errors = []
with (PROJECT / "precision_results.csv").open("w", newline="") as output:
    writer = csv.writer(output)
    writer.writerow(["q1_rad", "q2_rad", "q4_m", "fk_x_m", "fk_y_m", "fk_z_m",
                     "tcp_x_m", "tcp_y_m", "tcp_z_m", "error_m"])
    for q1, q2, q4 in POSES:
        if not (-0.73 <= q1 <= 0.73 and 0 < q2 <= 0.83 and -0.2 <= q4 <= 0):
            raise ValueError("Joint values exceed coursework limits.")
        target = (q1, q2, q4, 0.0)
        for motor, value in zip(motors, target):
            motor.setPosition(value)
        start = robot.getTime()
        previous = None
        stable = 0
        while robot.step(timestep) != -1:
            actual = [sensor.getValue() for sensor in sensors]
            position = measured_tcp()
            close = all(abs(a - b) <= tolerance for a, b, tolerance in
                        zip(actual, target, (0.001, 0.001, 0.0005, 0.001)))
            still = previous is not None and math.dist(position, previous) <= 0.000001
            stable = stable + 1 if close and still else 0
            if stable * timestep >= 100:
                break
            if robot.getTime() - start >= 15:
                raise RuntimeError("Arm did not settle; precision check incomplete.")
            previous = position
        else:
            raise RuntimeError("Simulation stopped; precision check incomplete.")
        predicted = forward_kinematics((q1, q2, q4))
        error = math.dist(predicted, position)
        writer.writerow([q1, q2, q4, *predicted, *position, error])
        errors.append(error)
        print(f"Joints {(q1, q2, q4)}: FK={predicted}, TCP={position}, error={error:.6g} m")

print(f"Mean error: {sum(errors) / len(errors):.6g} m; maximum: {max(errors):.6g} m")
robot.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)

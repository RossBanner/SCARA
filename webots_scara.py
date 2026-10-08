"""Task 1.A: Webots motor control and independent supervisor TCP observation.

The SCARA Robot must have supervisor TRUE. A DEF SCARA_TCP (standalone world)
or DEF TCP (course world) Pose must be at the handSlot origin. The official
course tcp_pose supervisor remains present as an independent console readout.
"""

import math

from scara_fk import JOINT_NAMES, finite_vector, validate_command, world_to_base


class ScaraRig:
    def __init__(self, supervisor):
        self.robot = supervisor
        self.step_ms = int(supervisor.getBasicTimeStep())
        self.base = supervisor.getSelf()
        self.tcp = supervisor.getFromDef("SCARA_TCP")
        if self.tcp is None:
            self.tcp = supervisor.getFromDef("TCP")
        if self.tcp is None:
            raise RuntimeError("A DEF SCARA_TCP or DEF TCP Pose is required at handSlot origin.")
        names = JOINT_NAMES + ("shaft_rotation_motor",)
        self.motors = [supervisor.getDevice(name) for name in names]
        if any(motor is None for motor in self.motors):
            raise RuntimeError("SCARA motors were not found. Check the controller's robot.")
        self.sensors = [motor.getPositionSensor() for motor in self.motors]
        if any(sensor is None for sensor in self.sensors):
            raise RuntimeError("Position sensors are required to verify settling.")
        for sensor in self.sensors:
            sensor.enable(self.step_ms)
        for motor, speed in zip(self.motors, (1.0, 1.0, 0.05, 1.0)):
            motor.setVelocity(min(speed, motor.getMaxVelocity()))
        self.motors[3].setPosition(0.0)
        self.step()

    def step(self):
        if self.robot.step(self.step_ms) == -1:
            raise RuntimeError("Simulation stopped before the measurement completed.")

    def read_tcp_base(self):
        # This reads the simulated scene, never the analytical FK calculation.
        return world_to_base(self.tcp.getPosition(), self.base.getPosition(),
                             self.base.getOrientation())

    def move_and_measure(self, joints, timeout_s=15.0):
        target = validate_command(joints)
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError("timeout_s must be positive and finite.")
        for motor, value in zip(self.motors[:3], target):
            lower, upper = motor.getMinPosition(), motor.getMaxPosition()
            if lower != upper and not lower <= value <= upper:
                raise ValueError("Command is outside the loaded robot's motor limits.")
            motor.setPosition(value)
        self.motors[3].setPosition(0.0)
        start = self.robot.getTime()
        previous_tcp = None
        stable_steps = 0
        # Require target convergence AND low tip motion for at least 0.1 seconds.
        required_steps = max(3, math.ceil(0.1 / (self.step_ms / 1000)))
        tolerances = (1e-3, 1e-3, 5e-4, 1e-3)
        while self.robot.getTime() - start < timeout_s:
            self.step()
            measured = finite_vector([sensor.getValue() for sensor in self.sensors], 4)
            tcp = self.read_tcp_base()
            near_target = all(abs(actual - desired) <= tolerance for actual, desired, tolerance
                              in zip(measured, target + (0.0,), tolerances))
            still = previous_tcp is not None and math.dist(tcp, previous_tcp) <= 1e-6
            stable_steps = stable_steps + 1 if near_target and still else 0
            if stable_steps >= required_steps:
                return measured[:3], tcp, True
            previous_tcp = tcp
        # Preserve a failed measurement as timeout; do not pretend it settled.
        return measured[:3], tcp, False

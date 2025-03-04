from __future__ import annotations
import commands2

class run(commands2.Command):
    def __init__(self, speed, motor):
        super().__init__()

        self.speed = speed
        if not callable(speed):
            self.speed = lambda: speed

        self.motor = motor
        self.addRequirements(motor)

    def initialize(self):
        pass

    def isFinished(self) -> bool:
        return False  # never finishes, you should use it with "withTimeout(...)"

    def execute(self):
        speed = self.speed()
        self.motor.run(speed)

    def end(self, interrupted: bool):
        self.motor.run(0)  # stop at the end

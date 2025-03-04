from rev import SparkFlex
from commands2 import Subsystem

class Motor(Subsystem):
    def __init__(
        self,
        motorCANId: int,
    ) -> None:
        super().__init__()

        self.motor = SparkFlex(motorCANId, SparkFlex.MotorType.kBrushless)

    def setSpeed(self, speed: float) -> None:
        self.motor.set(speed)

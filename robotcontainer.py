from __future__ import annotations

import math

import commands2
import wpimath
import wpilib
import typing

from commands2 import cmd, InstantCommand, RunCommand, SequentialCommandGroup
from commands2.button import JoystickButton
from wpilib import XboxController
from wpimath.controller import PIDController, ProfiledPIDControllerRadians, HolonomicDriveController
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator

from constants import AutoConstants, DriveConstants, OIConstants, MotorCanId
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.motor import Motor

from commands.reset_xy import ResetXY, ResetSwerveFront

class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:

        # The robot's subsystems
        from subsystems.limelight_camera import LimelightCamera
        self.camera = LimelightCamera("limelight")  # name of your camera goes in parentheses
        self.motor = Motor(MotorCanId.motorCanId)

        from rev import SparkFlex
        from subsystems.elevator import Elevator
        self.elevator = Elevator(leadMotorCANId=12, presetSwitchPositions=(0, 10), motorClass=SparkFlex)

        self.elevator.setDefaultCommand(
                commands2.RunCommand(lambda: self.elevator.drive(self.justinController.getRawAxis(XboxController.Axis.kRightY)), self.elevator)
        )

        # The robot's subsystems
        self.robotDrive = DriveSubsystem()

        # The driver's controller
        self.driverController = wpilib.XboxController(0)
        self.justinController = wpilib.XboxController(1)

        # Configure the button bindings and autos
        self.configureButtonBindings()
        self.configureAutos()

        # Configure default commands
        self.robotDrive.setDefaultCommand(
            # The left stick controls translation of the robot.
            # Turning is controlled by the X axis of the right stick.
            commands2.RunCommand(
                lambda: self.robotDrive.drive(
                    wpimath.applyDeadband(
                        self.driverController.getLeftY(), OIConstants.kDriveDeadband
                    ),
                    wpimath.applyDeadband(
                        self.driverController.getLeftX(), OIConstants.kDriveDeadband
                    ),
                    -wpimath.applyDeadband(
                        self.driverController.getRightX(), OIConstants.kDriveDeadband
                    ),
                    True,
                    True,
                    square=True,
                ),
                self.robotDrive,
            )
        )

        self.motor.setDefaultCommand(
            commands2.RunCommand(
                lambda: self.motor.setSpeed(
                    -0.20 * (self.driverController.getRightTriggerAxis() -
                    self.driverController.getLeftTriggerAxis())
                ),
                self.motor
            )
        )

    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """

        from commands2.button import JoystickButton
        from commands2 import RunCommand

        xButton = JoystickButton(self.driverController, XboxController.Button.kX)
        xButton.onTrue(ResetXY(x=7.0, y=4.025, headingDegrees=180.0, drivetrain=self.robotDrive))
        xButton.whileTrue(RunCommand(self.robotDrive.setX, self.robotDrive))  # use the swerve X brake when "X" is pressed

        yButton = JoystickButton(self.driverController, XboxController.Button.kY)
        yButton.onTrue(ResetSwerveFront(self.robotDrive))

        bButton = JoystickButton(self.driverController, XboxController.Button.kB)
        bButton.whileTrue(self.approachAprilTagAndShoot(pipeline=0))

    def disablePIDSubsystems(self) -> None:
        """Disables all ProfiledPIDSubsystem and PIDSubsystem instances.
        This should be called on robot disable to prevent integral windup."""

    def getAutonomousCommand(self) -> commands2.Command:
        """
        :returns: the command to run in autonomous
        """
        command = self.chosenAuto.getSelected()
        return command()

    def configureAutos(self):
        self.chosenAuto = wpilib.SendableChooser()
        # you can also set the default option, if needed
        self.chosenAuto.setDefaultOption("center auto", self.centerAuto)
        self.chosenAuto.addOption("jc left", self.leftAuto)
        self.chosenAuto.addOption("jc null", self.rightAuto)

        #self.chosenAuto.setDefaultOption("trajectory example", self.getAutonomousTrajectoryExample)
        #self.chosenAuto.addOption("left blue", self.getAutonomousLeftBlue)
        #self.chosenAuto.addOption("left red", self.getAutonomousLeftRed)

        wpilib.SmartDashboard.putData("Chosen Auto", self.chosenAuto)

    def centerAuto(self):
        setStart = ResetXY(x=7.025, y=4.025, headingDegrees=180, drivetrain=self.robotDrive)
        return SequentialCommandGroup(setStart, self.approachAprilTagAndShoot(pipeline=2))

    def leftAuto(self):
        setStart = ResetXY(x=6.98, y=6.177, headingDegrees=-142, drivetrain=self.robotDrive)
        return SequentialCommandGroup(setStart, self.approachAprilTagAndShoot(pipeline=1))

    def rightAuto(self):
        setStart = ResetXY(x=6.98, y=8.05-6.177, headingDegrees=+142, drivetrain=self.robotDrive)
        return SequentialCommandGroup(setStart, self.approachAprilTagAndShoot(pipeline=3))

    def approachAprilTagAndShoot(self, pipeline=0):
        from commands.approach import ApproachTag

        # our Limelight has 4 pipelines:
        #   0 = any tag
        #   1 = tags 20,11
        #   2 = tags 21,10
        #   3 = tags 22,9

        def roundToMultipleOf60():
            return 60 * round(self.robotDrive.getHeading().degrees() / 60)

        from commands.setcamerapipeline import SetCameraPipeline
        from commands.motorcommand import Run

        setCameraPipeline = SetCameraPipeline(self.camera, pipeline)

        shoot = Run(speed=-0.1, motor=self.motor)


        approach = ApproachTag(
            self.camera,
            self.robotDrive,
            specificHeadingDegrees=roundToMultipleOf60,
            settings={"GainTran":0.5},
            speed=1.0,
            pushForwardSeconds=0.3,
            finalApproachObjSize=15,
        )  # tuning this at speed=0.5, should be comfortable setting speed=1.0 instead

        # or you can do this, if you want to score the coral 15 centimeters to the right and two centimeters back from the AprilTag
        # stepToSide = SwerveToSide(drivetrain=self.robotDrive, metersToTheLeft=-0.15, metersBackwards=0.02, speed=0.2)
        # alignToScore = approach.andThen(stepToSide)

        return SequentialCommandGroup(setCameraPipeline, approach, shoot.withTimeout(2.0) )


    def getAutonomousLeftBlue(self):
        setStartPose = ResetXY(x=0.783, y=6.686, headingDegrees=+60, drivetrain=self.robotDrive)
        driveForward = commands2.RunCommand(lambda: self.robotDrive.arcadeDrive(xSpeed=1.0, rot=0.0), self.robotDrive)
        stop = commands2.InstantCommand(lambda: self.robotDrive.arcadeDrive(0, 0))

        command = setStartPose.andThen(driveForward.withTimeout(1.0)).andThen(stop)
        return command

    def getAutonomousLeftRed(self):
        setStartPose = ResetXY(x=15.777, y=4.431, headingDegrees=-120, drivetrain=self.robotDrive)
        driveForward = commands2.RunCommand(lambda: self.robotDrive.arcadeDrive(xSpeed=1.0, rot=0.0), self.robotDrive)
        stop = commands2.InstantCommand(lambda: self.robotDrive.arcadeDrive(0, 0))

        command = setStartPose.andThen(driveForward.withTimeout(2.0)).andThen(stop)
        return command

    def getAutonomousTrajectoryExample(self) -> commands2.Command:
        # Create config for trajectory
        config = TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared,
        )
        # Add kinematics to ensure max speed is actually obeyed
        config.setKinematics(DriveConstants.kDriveKinematics)

        # An example trajectory to follow. All units in meters.
        exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            # Start at the origin facing the +X direction
            Pose2d(0, 0, Rotation2d(0)),
            # Pass through these two interior waypoints, making an 's' curve path
            [Translation2d(0.5, 0.5), Translation2d(1, -0.5)],
            # End 1.5 meters straight ahead of where we started, facing forward
            Pose2d(1.5, 0, Rotation2d(0)),
            config,
        )

        thetaController = ProfiledPIDControllerRadians(
            AutoConstants.kPThetaController,
            0,
            0,
            AutoConstants.kThetaControllerConstraints,
        )
        thetaController.enableContinuousInput(-math.pi, math.pi)

        driveController = HolonomicDriveController(
            PIDController(AutoConstants.kPXController, 0, 0),
            PIDController(AutoConstants.kPXController, 0, 0),
            thetaController,
        )

        swerveControllerCommand = commands2.SwerveControllerCommand(
            exampleTrajectory,
            self.robotDrive.getPose,  # Functional interface to feed supplier
            DriveConstants.kDriveKinematics,
            driveController,
            self.robotDrive.setModuleStates,
            (self.robotDrive,),
        )

        # Reset odometry to the starting pose of the trajectory.
        self.robotDrive.resetOdometry(exampleTrajectory.initialPose())

        # Run path following command, then stop at the end.
        return swerveControllerCommand.andThen(
            cmd.run(
                lambda: self.robotDrive.drive(0, 0, 0, False, False),
                self.robotDrive,
            )
        )

    def getTestCommand(self) -> typing.Optional[commands2.Command]:
        """
        :returns: the command to run in test mode (to exercise all systems)
        """
        return None

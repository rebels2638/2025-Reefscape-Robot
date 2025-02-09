package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.AbsoluteFieldDrive;
import frc.robot.lib.input.XboxController;
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.vision.Vision;
import frc.robot.commands.autoAlignment.*;
import frc.robot.commands.elevator.RunElevatorRaw;
import frc.robot.commands.elevator.simple.MoveElevatorL1;
import frc.robot.commands.elevator.simple.MoveElevatorL2;
import frc.robot.commands.elevator.simple.MoveElevatorL3;
import frc.robot.commands.elevator.simple.MoveElevatorStow;
import frc.robot.commands.roller.*;
import frc.robot.commands.roller.simple.*;

import frc.robot.constants.Constants.AlignmentConstants;

public class RobotContainer {
  public static RobotContainer instance = null;

  public static RobotContainer getInstance() {
    if (instance == null) {
      instance = new RobotContainer();
    }

    return instance;
  }

  private final SwerveDrive swerveDrive;
  private final Vision vision;
  private final RobotState robotState;

  // private final Elevator elevator;
  // private final Roller roller;

  // private final AutoRunner autoRunner;

  private final XboxController xboxTester;
  private final XboxController xboxDriver;
  private final XboxController xboxOperator;

  private RobotContainer() {
    this.xboxTester = new XboxController(1);
    this.xboxOperator = new XboxController(2);
    this.xboxDriver = new XboxController(3);

    swerveDrive = SwerveDrive.getInstance();
    vision = Vision.getInstance();
    robotState = RobotState.getInstance();

    // roller = Roller.getInstance();
    // elevator = Elevator.getInstance();

    // autoRunner = AutoRunner.getInstance();

    swerveDrive.setDefaultCommand(new AbsoluteFieldDrive(xboxDriver));
    xboxDriver.getXButton().onTrue(new InstantCommand(() -> robotState.zeroGyro()));

    xboxDriver.getLeftBumper().whileTrue(
        new ConditionalCommand(
            new LinearDriveToPose(() -> robotState.getClosestLeftBranchPose(), () -> new ChassisSpeeds()),
            new InstantCommand(),
            () -> robotState.getClosestLeftBranchPose().getTranslation().getDistance(
                robotState.getEstimatedPose().getTranslation()) <= AlignmentConstants.kMAX_ALIGNMENT_DIST_METERS));

    xboxDriver.getRightBumper().whileTrue(
        new ConditionalCommand(
            new LinearDriveToPose(() -> robotState.getClosestRightBranchPose(), () -> new ChassisSpeeds()),
            new InstantCommand(),
            () -> robotState.getClosestRightBranchPose().getTranslation().getDistance(
                robotState.getEstimatedPose().getTranslation()) <= AlignmentConstants.kMAX_ALIGNMENT_DIST_METERS));

    xboxDriver.getYButton().whileTrue(
        new ConditionalCommand(
            new LinearDriveToPose(() -> robotState.getClosestAlgayPose(), () -> new ChassisSpeeds()),
            new InstantCommand(),
            () -> robotState.getClosestAlgayPose().getTranslation().getDistance(
                robotState.getEstimatedPose().getTranslation()) <= AlignmentConstants.kMAX_ALIGNMENT_DIST_METERS));
        
    // xboxOperator.getLeftBumper().onTrue(new IntakeCoral());
    // xboxOperator.getRightBumper().onTrue(new EjectCoral());

    // // elevator.setDefaultCommand(new RunElevatorRaw(xboxOperator));
    // xboxOperator.getAButton().onTrue(new MoveElevatorStow());
    // xboxOperator.getBButton().onTrue(new MoveElevatorL1());
    // xboxOperator.getYButton().onTrue(new MoveElevatorL2());
    // xboxOperator.getXButton().onTrue(new MoveElevatorL3());

    // xboxDriver.getLeftBumper().whileTrue(new LinearDriveToPose(() ->
    // robotState.getClosestLeftBranchPose(), () -> new ChassisSpeeds()));
    // xboxDriver.getRightBumper().whileTrue(new LinearDriveToPose(() ->
    // robotState.getClosestRightBranchPose(), () -> new ChassisSpeeds()));
    // xboxDriver.getYButton().whileTrue(new LinearDriveToPose(() ->
    // robotState.getClosestAlgayPose(), () -> new ChassisSpeeds()));

    // xboxDriver.getYButton().onTrue(new
    // PathplanToPose(RobotState.getInstance().alignmentPoseSearch()));
  }

  public Command getAutonomousCommand() {
    // return autoRunner.getAutonomousCommand();
    return null;
  }

}

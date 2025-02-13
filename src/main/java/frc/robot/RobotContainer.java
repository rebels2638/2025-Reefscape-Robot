package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AbsoluteFieldDrive;
import frc.robot.commands.AutoRunner;
import frc.robot.commands.DoSomething;
import frc.robot.lib.input.XboxController;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.vision.Vision;
import frc.robot.commands.autoAlignment.*;
import frc.robot.commands.autoAlignment.reef.AlignToAlgay;
import frc.robot.commands.autoAlignment.reef.AlignToLeftBranch;
import frc.robot.commands.autoAlignment.reef.AlignToRightBranch;
import frc.robot.commands.claw.simple.RunClawIntake;
import frc.robot.commands.claw.simple.StopClaw;
import frc.robot.commands.elevator.RunElevatorRaw;
import frc.robot.commands.elevator.simple.MoveElevatorL1;
import frc.robot.commands.elevator.simple.MoveElevatorL2;
import frc.robot.commands.elevator.simple.MoveElevatorL3;
import frc.robot.commands.elevator.simple.MoveElevatorL4;
import frc.robot.commands.elevator.simple.MoveElevatorStow;
import frc.robot.commands.pivot.RunPivotRaw;
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

// private final Pivot pivot;
// private final Claw claw;

  private final Elevator elevator;
  private final Roller roller;

  private final AutoRunner autoRunner;

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
    // pivot = Pivot.getInstance();
    // claw = Claw.getInstance();

    roller = Roller.getInstance();
    elevator = Elevator.getInstance();

    autoRunner = AutoRunner.getInstance();

    swerveDrive.setDefaultCommand(new AbsoluteFieldDrive(xboxDriver));
    xboxDriver.getXButton().onTrue(new InstantCommand(() -> robotState.zeroGyro()));

    xboxDriver.getLeftBumper().whileTrue(new AlignToLeftBranch());
    xboxDriver.getRightBumper().whileTrue(new AlignToRightBranch());
    xboxDriver.getYButton().whileTrue(new AlignToAlgay());
        
    // xboxOperator.getLeftBumper().onTrue(new IntakeCoral());
    // xboxOperator.getRightBumper().onTrue(new EjectCoral());
    xboxOperator.getLeftBumper().onTrue(new RunRollerIntake());
    xboxOperator.getRightBumper().onTrue(new StopRoller());

    // xboxOperator.getLeftBumper().onTrue(new RunClawIntake(claw));
    // xboxOperator.getRightBumper().onTrue(new StopClaw(claw));

    // elevator.setDefaultCommand(new RunElevatorRaw(xboxOperator));
    xboxOperator.getAButton().onTrue(new MoveElevatorStow());
    xboxOperator.getBButton().onTrue(new MoveElevatorL2());
    // xboxOperator.getYButton().onTrue(new SequentialCommandGroup(
    //   new IntakeCoral(),
    //   new LinearDriveToPose(
    //     () -> AlignmentUtil.getClosestLeftBranchPose(), 
    //     () -> new ChassisSpeeds()),
    //   new MoveElevatorL3(),
    //   new EjectCoral(),
    //   new MoveElevatorStow()
    // ));
    xboxOperator.getYButton().onTrue(new MoveElevatorL3());
    xboxOperator.getXButton().onTrue(new MoveElevatorL4());

        // pivot.setDefaultCommand(new RunPivotRaw(xboxOperator));

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
    return autoRunner.getAutonomousCommand();
  }

}

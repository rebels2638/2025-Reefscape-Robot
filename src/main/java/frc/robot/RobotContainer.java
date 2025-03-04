package frc.robot;

import java.util.jar.Attributes.Name;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AbsoluteFieldDrive;
import frc.robot.commands.AutoRunner;
import frc.robot.lib.input.XboxController;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.height;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.vision.Vision;
import frc.robot.commands.autoAlignment.*;
import frc.robot.commands.autoAlignment.reef.AlignToAlgayLinear;
import frc.robot.commands.autoAlignment.reef.AlignToLeftBranchLinearAndScore;
import frc.robot.commands.autoAlignment.reef.AlignToRightBranchLinearAndScore;
import frc.robot.commands.autoAlignment.source.AlignToClosestSourcePathfind;
import frc.robot.commands.claw.simple.RunClawIntake;
import frc.robot.commands.claw.simple.StopClaw;
import frc.robot.commands.complex.superstructure.Score;
import frc.robot.commands.complex.superstructure.ScoreL1Superstructure;
import frc.robot.commands.complex.superstructure.ScoreL2Superstructure;
import frc.robot.commands.complex.superstructure.ScoreL3Superstructure;
import frc.robot.commands.complex.superstructure.ScoreL4Superstructure;
import frc.robot.commands.claw.simple.RunClawEject;
import frc.robot.commands.elevator.RunElevatorRaw;
import frc.robot.commands.elevator.simple.DequeueElevatorAction;
import frc.robot.commands.elevator.simple.QueueL1Action;
import frc.robot.commands.elevator.simple.QueueL2Action;
import frc.robot.commands.elevator.simple.QueueL3Action;
import frc.robot.commands.elevator.simple.QueueL4Action;
import frc.robot.commands.elevator.simple.QueueStowAction;
import frc.robot.commands.pivot.RunPivotRaw;
import frc.robot.commands.pivot.simple.MovePivotAlgay;
import frc.robot.commands.pivot.simple.MovePivotStow;
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
    // private final Vision vision;
    private final RobotState robotState;

    // private final Pivot pivot;
    // private final Claw claw;

    private final Elevator elevator;
    // private final Roller roller;
    // private final MechanismVisualizer mechanismVisualizer;

    // private final AutoRunner autoRunner;

    private final XboxController xboxTester;
    private final XboxController xboxDriver;
    private final XboxController xboxOperator;

    private RobotContainer() {
        this.xboxTester = new XboxController(1);
        this.xboxOperator = new XboxController(2);
        this.xboxDriver = new XboxController(3);

        swerveDrive = SwerveDrive.getInstance();
        // vision = Vision.getInstance();
        robotState = RobotState.getInstance();
        // pivot = Pivot.getInstance();
        // claw = Claw.getInstance();

        // roller = Roller.getInstance();
        elevator = Elevator.getInstance();

        // mechanismVisualizer = MechanismVisualizer.getInstance();
        
        // NamedCommands.registerCommand("Intake", new IntakeCoral());

        // autoRunner = AutoRunner.getInstance();

        swerveDrive.setDefaultCommand(new AbsoluteFieldDrive(xboxDriver));
        // elevator.setDefaultCommand(new RunElevatorRaw(xboxOperator));
        // pivot.setDefaultCommand(new RunPivotRaw(xboxOperator));
        xboxDriver.getXButton().onTrue(new InstantCommand(() -> robotState.zeroGyro()));

        // xboxDriver.getLeftTriggerButton(0.94).whileTrue(new AlignToLeftBranchLinearAndScore()).toggleOnFalse(new SequentialCommandGroup(new QueueStowAction(), new DequeueElevatorAction()));
        // xboxDriver.getRightTriggerButton(0.94).whileTrue(new AlignToRightBranchLinearAndScore()).toggleOnFalse(new SequentialCommandGroup(new QueueStowAction(), new DequeueElevatorAction()));
        // xboxDriver.getRightBumper().whileFalse()
        // xboxDriver.getYButton().whileTrue(new AlignToAlgayLinear()).toggleOnFalse(new SequentialCommandGroup(new QueueStowAction(), new DequeueElevatorAction()));
        // CommandScheduler.getInstance().onCommandInterrupt(
        //     CommandScheduler.getInstance().isScheduled(new AlignToAlgayLinear(), new AlignToLeftBranchLinearAndScore(), new AlignToRightBranchLinearAndScore()) ?
            
        // )
        // new Trigger(() -> (xboxOperator.getRightTriggerButton(0.94).getAsBoolean() && xboxOperator.getLeftTriggerButton(0.94).getAsBoolean())).onTrue(new AlignToAlgayLinear());
        // xboxDriver.getAButton().onTrue(new Score());
        // xboxDriver.getYButton().whileTrue(new AlignToClosestSourcePathfind(xboxDriver));

        // xboxDriver.getAButton().whileTrue(new RunClawEject());

        // xboxDriver.getLeftMiddleButton().onTrue(new MovePivotAlgay());
        // xboxDriver.getRightMiddleButton().onTrue(new MovePivotStow());

        // xboxDriver.getYButton().onTrue(new IntakeCoral());

        // xboxOperator.getLeftBumper().onTrue(new IntakeCoral());
        // xboxOperator.getRightBumper().onTrue(new EjectCoral());
        // xboxOperator.getLeftBumper().onTrue(new RunRollerIntake());
        // xboxOperator.getRightBumper().onTrue(new StopRoller());

        // xboxOperator.getRightMiddleButton().onTrue(new RunClawIntake());
        // xboxOperator.getLeftMiddleButton().onTrue(new RunClawEject());

        // elevator.setDefaultCommand(new RunElevatorRaw(xboxOperator));
        xboxOperator.getAButton().onTrue(new QueueStowAction().andThen(new DequeueElevatorAction()));
        xboxOperator.getBButton().onTrue(new QueueL2Action().andThen(new DequeueElevatorAction()));
        xboxOperator.getYButton().onTrue(new QueueL3Action().andThen(new DequeueElevatorAction()));
        xboxOperator.getXButton().onTrue(new QueueL4Action().andThen(new DequeueElevatorAction()));
        xboxOperator.getRightBumper().onTrue(new IntakeCoral());
        
        xboxOperator.getLeftBumper().onTrue(new EjectCoral());
        // pivot.setDefaultCommand(new RunPivotRaw(xboxOperator));

        // xboxDriver.getLeftBumper().whileTrue(new LinearDriveToPose(() ->
        // robotState.getClosestLe`tBranchPose(), () -> new ChassisSpeeds()));
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


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
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AbsoluteFieldDrive;
import frc.robot.commands.AutoRunner;
import frc.robot.lib.input.XboxController;
import frc.robot.lib.util.AlignmentUtil;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.vision.Vision;
import frc.robot.commands.autoAlignment.*;
import frc.robot.commands.autoAlignment.reef.AlignToAlgayLinearAndRemoveTelop;
import frc.robot.commands.autoAlignment.reef.AlignToLeftBranchLinearAndScoreTelop;
import frc.robot.commands.autoAlignment.reef.AlignToRightBranchLinearAndScoreTelop;
import frc.robot.commands.autoAlignment.source.AlignToClosestSourcePathfind;
import frc.robot.commands.claw.simple.RunClawIntake;
import frc.robot.commands.claw.simple.StopClaw;
import frc.robot.commands.climber.simple.MoveClimberStow;
import frc.robot.commands.climber.simple.MoveDeepCage;
import frc.robot.commands.complex.superstructure.ScoreL1Superstructure;
import frc.robot.commands.complex.superstructure.ScoreL2Superstructure;
import frc.robot.commands.complex.superstructure.ScoreL3Superstructure;
import frc.robot.commands.complex.superstructure.ScoreL4Superstructure;
import frc.robot.commands.claw.simple.BargeScoringManualShot;
import frc.robot.commands.claw.simple.DecideBargeScoringFlick;
import frc.robot.commands.claw.simple.RunClawEject;
import frc.robot.commands.elevator.CancelScoreCoral;
import frc.robot.commands.elevator.RunElevatorRaw;
import frc.robot.commands.elevator.simple.CancelScoreAlgay;
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
    private final Vision vision;
    private final RobotState robotState;

    // private final Pivot pivot;
    private final Claw claw;
    private final Climber climber;

    private final Elevator elevator;
    private final Roller roller;

    // private final MechanismVisualizer mechanismVisualizer;

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
        claw = Claw.getInstance();
        climber = Climber.getInstance();

        roller = Roller.getInstance();
        elevator = Elevator.getInstance();

        // mechanismVisualizer = MechanismVisualizer.getInstance();
        // NamedCommands.registerCommand("Intake", new IntakeCoral());
        autoRunner = AutoRunner.getInstance();

        swerveDrive.setDefaultCommand(new AbsoluteFieldDrive(xboxDriver));
        xboxDriver.getXButton().onTrue(new InstantCommand(() -> robotState.zeroGyro()));

        xboxDriver.getLeftTriggerButton(0.94).whileTrue(new AlignToLeftBranchLinearAndScoreTelop(xboxDriver)).toggleOnFalse(new CancelScoreCoral()); // ScoreLeft
        xboxDriver.getRightTriggerButton(0.94).whileTrue(new AlignToRightBranchLinearAndScoreTelop(xboxDriver)).toggleOnFalse(new CancelScoreCoral()); // ScoreRight
        new Trigger(() -> (xboxDriver.getRightTriggerButton(0.94).getAsBoolean() && xboxDriver.getLeftTriggerButton(0.94).getAsBoolean())).onTrue(new AlignToAlgayLinearAndRemoveTelop(xboxDriver)).toggleOnFalse(new CancelScoreAlgay()); // DescoreAlgay
        // xboxDriver.getBButton().whileTrue(new AlignToBargeAxisLocked(xboxDriver)).toggleOnFalse(new DecideBargeScoringFlick()); // BargeAxisLockAndScoreOnRelease
        // xboxDriver.getYButton().onTrue(new BargeScoringManualShot()).toggleOnFalse(new CancelScoreAlgay());
        // xboxDriver.getRightBumper().whileTrue(new AlignToClosestSourceLinearAndIntake()).toggleOnFalse(new StopRoller());
        // xboxDriver.getLeftBumper().onTrue(new AlignToProcessorAndScore()).toggleOnFalse(new MovePivotStow()); // processor
        // new Trigger(() -> (xboxDriver.getRightBumper().getAsBoolean() && xboxDriver.getLeftBumper().getAsBoolean())).onTrue(); // AutoAlignToTargetCageAndClimb

        xboxOperator.getAButton().onTrue(new QueueStowAction());
        xboxOperator.getBButton().onTrue(new QueueL2Action());
        xboxOperator.getYButton().onTrue(new QueueL3Action());
        xboxOperator.getXButton().onTrue(new QueueL4Action());
        xboxOperator.getRightBumper().onTrue(new IntakeCoral());
        xboxOperator.getLeftBumper().onTrue(new StopRoller());

        // xboxOperator.getAButton().onTrue(new QueueStowAction().andThen(new DequeueElevatorAction()));
        // xboxOperator.getBButton().onTrue(new QueueL2Action().andThen(new DequeueElevatorAction()));
        // xboxOperator.getYButton().onTrue(new QueueL3Action().andThen(new DequeueElevatorAction()));
        // xboxOperator.getXButton().onTrue(new QueueL4Action().andThen(new DequeueElevatorAction()));
        // xboxOperator.getLeftBumper().onTrue(new MovePivotAlgay());

    }

    public Command getAutonomousCommand() {
        return autoRunner.getAutonomousCommand();
    }
}

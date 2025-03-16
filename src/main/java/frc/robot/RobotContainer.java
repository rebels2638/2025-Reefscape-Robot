
package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
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
import frc.robot.subsystems.pneumatics.Pneumatics;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.vision.Vision;
import frc.robot.commands.autoAlignment.barge.AlignToCageAndClimb;
import frc.robot.commands.autoAlignment.barge.AlignToClosestBargePoint;
import frc.robot.commands.autoAlignment.reef.AlignToAlgayLinearAndRemove;
import frc.robot.commands.autoAlignment.reef.AlignToLeftBranchLinearAndScore;
import frc.robot.commands.autoAlignment.reef.AlignToRightBranchLinearAndScore;
import frc.robot.commands.autoAlignment.source.AlignToClosestSourceLinearAndIntake;
import frc.robot.commands.claw.DecideBargeScoringFlick;
import frc.robot.commands.elevator.CancelScoreAlgay;
import frc.robot.commands.elevator.CancelScoreCoral;
import frc.robot.commands.elevator.simple.QueueL2Action;
import frc.robot.commands.elevator.simple.QueueL3Action;
import frc.robot.commands.elevator.simple.QueueL4Action;
import frc.robot.commands.elevator.simple.QueueStowAction;
import frc.robot.commands.roller.IntakeCoral;
import frc.robot.commands.roller.simple.*;
import frc.robot.commands.complex.FloorEjectAlgay;

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

    private final Pivot pivot;
    private final Claw claw;
    private final Climber climber;

    private final Elevator elevator;
    private final Roller roller;

    private final MechanismVisualizer mechanismVisualizer;

    private final AutoRunner autoRunner;

    private final XboxController xboxTester;
    private final XboxController xboxDriver;
    private final XboxController xboxOperator;

    private final Debouncer leftAlignDebouncer = new Debouncer(0.25, edu.wpi.first.math.filter.Debouncer.DebounceType.kRising);
    private final Debouncer rightAlignDebouncer = new Debouncer(0.25, edu.wpi.first.math.filter.Debouncer.DebounceType.kRising);

    private RobotContainer() {
        this.xboxTester = new XboxController(1);
        this.xboxOperator = new XboxController(2);
        this.xboxDriver = new XboxController(3);

        swerveDrive = SwerveDrive.getInstance();
        vision = Vision.getInstance();
        robotState = RobotState.getInstance();
        pivot = Pivot.getInstance();
        claw = Claw.getInstance();
        climber = Climber.getInstance();
        roller = Roller.getInstance();
        elevator = Elevator.getInstance();
        mechanismVisualizer = MechanismVisualizer.getInstance();
        autoRunner = AutoRunner.getInstance();
        Pneumatics.getInstance();

        swerveDrive.setDefaultCommand(new AbsoluteFieldDrive(xboxDriver));

        new Trigger(
            () -> (
                leftAlignDebouncer.calculate(
                    xboxDriver.getLeftTriggerButton(0.94).getAsBoolean() && !xboxDriver.getRightTriggerButton(0.94).getAsBoolean()
                )
            )
        ).whileTrue(new AlignToLeftBranchLinearAndScore(xboxDriver)).onFalse(new CancelScoreCoral());

        new Trigger(
            () -> (
                rightAlignDebouncer.calculate(
                    xboxDriver.getRightTriggerButton(0.94).getAsBoolean() && !xboxDriver.getLeftTriggerButton(0.94).getAsBoolean()
                )
            )
        ).whileTrue(new AlignToRightBranchLinearAndScore(xboxDriver)).onFalse(new CancelScoreCoral());

        new Trigger(
            () -> (
                xboxDriver.getRightTriggerButton(0.94).getAsBoolean() && 
                xboxDriver.getLeftTriggerButton(0.94).getAsBoolean()
            )
        ).whileTrue(new AlignToAlgayLinearAndRemove(xboxDriver)).onFalse(new CancelScoreAlgay()); // DescoreAlgay

        new Trigger(
            () -> (
                xboxDriver.getRightBumper().getAsBoolean() &&
                !xboxDriver.getLeftBumper().getAsBoolean()
            )
        ).whileTrue(new AlignToCageAndClimb(xboxDriver)).onFalse(new InstantCommand()); // climb TODO: reset condition and implement an "already at pose" check inside command

        new Trigger(
            () -> (
                xboxDriver.getYButton().getAsBoolean() && !xboxDriver.getBButton().getAsBoolean()
            )
        ).whileTrue(new AlignToClosestBargePoint(xboxDriver)).onFalse(new DecideBargeScoringFlick());
        
        new Trigger(
            () -> (
                xboxDriver.getBButton().getAsBoolean() && !xboxDriver.getYButton().getAsBoolean()
            )
        ).whileTrue(new AlignToClosestSourceLinearAndIntake(xboxDriver)).onFalse(new StopRoller());

        // new Trigger(
        //     () -> (
        //         xboxDriver.getBButton().getAsBoolean() &&
        //         xboxDriver.getYButton().getAsBoolean()
        //     )
        // ).whileTrue(new InstantCommand()); // Processor

        xboxDriver.getXButton().onTrue(new InstantCommand(() -> robotState.zeroGyro()));

        xboxOperator.getAButton().onTrue(new QueueStowAction());
        xboxOperator.getBButton().onTrue(new QueueL2Action());
        xboxOperator.getYButton().onTrue(new QueueL3Action());
        xboxOperator.getXButton().onTrue(new QueueL4Action());
        xboxOperator.getRightBumper().onTrue(new IntakeCoral().withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

        // xboxTester.getBButton().onTrue(new InstantCommand(() -> Pneumatics.getInstance().pushFunnel()));
        // xboxTester.getAButton().onTrue(new InstantCommand(() -> Pneumatics.getInstance().pullFunnel()));
        // xboxTester.getXButton().onTrue(new InstantCommand(() -> Pneumatics.getInstance().pushRatchet()));
        // xboxTester.getYButton().onTrue(new InstantCommand(() -> Pneumatics.getInstance().pullRatchet()));

    }

    public Command getAutonomousCommand() {
        return autoRunner.getAutonomousCommand();
    }
}

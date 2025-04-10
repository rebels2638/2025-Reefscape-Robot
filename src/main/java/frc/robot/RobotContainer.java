
package frc.robot;

import org.ejml.equation.Sequence;

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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AbsoluteFieldDrive;
import frc.robot.commands.AutoRunner;
import frc.robot.commands.RumbleDriver;
import frc.robot.commands.ZeroPose;
import frc.robot.lib.input.XboxController;
import frc.robot.lib.util.AlignmentUtil;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.vision.Vision;
import frc.robot.commands.autoAlignment.barge.AlignToCageAndClimb;
import frc.robot.commands.autoAlignment.barge.ClimbSequence;
import frc.robot.commands.autoAlignment.barge.PrepareForClimbSequence;
import frc.robot.commands.autoAlignment.barge.PrepareMoveSuperstructureBargeSequence;
import frc.robot.commands.autoAlignment.barge.ScoreMoveSuperstrucutreBargeSequence;
import frc.robot.commands.autoAlignment.reef.AlignToAlgayLinearAndRemove;
import frc.robot.commands.autoAlignment.reef.AlignToLeftBranchLinearAndScore;
import frc.robot.commands.autoAlignment.reef.AlignToRightBranchLinearAndScore;
import frc.robot.commands.autoAlignment.source.AlignToClosestSourceLinearAndIntake;
import frc.robot.commands.claw.simple.HoldAlgayClaw;
import frc.robot.commands.claw.simple.InClaw;
import frc.robot.commands.claw.simple.RunClawEject;
import frc.robot.commands.claw.simple.RunClawIntake;
import frc.robot.commands.elevator.CancelScoreAlgay;
import frc.robot.commands.elevator.CancelScoreCoral;
import frc.robot.commands.elevator.simple.DequeueElevatorAction;
import frc.robot.commands.elevator.simple.QueueL2Action;
import frc.robot.commands.elevator.simple.QueueL3Action;
import frc.robot.commands.elevator.simple.QueueL4Action;
import frc.robot.commands.elevator.simple.QueueStowAction;
import frc.robot.commands.pivot.simple.MovePivotAlgay;
import frc.robot.commands.pivot.simple.MovePivotBargeForwards;
import frc.robot.commands.pivot.simple.MovePivotProcessor;
import frc.robot.commands.pivot.simple.MovePivotStow;
import frc.robot.commands.roller.EjectCoral;
import frc.robot.commands.roller.IntakeCoral;
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

    private final Debouncer bargeDebouncer = new Debouncer(0.25, edu.wpi.first.math.filter.Debouncer.DebounceType.kRising);
    private final Debouncer processerDebouncer = new Debouncer(0.25, edu.wpi.first.math.filter.Debouncer.DebounceType.kRising);

    private final Debouncer climbDebouncer = new Debouncer(0.25, edu.wpi.first.math.filter.Debouncer.DebounceType.kBoth);

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

        swerveDrive.setDefaultCommand(new AbsoluteFieldDrive(xboxDriver));
        claw.setDefaultCommand(new HoldAlgayClaw());

        new Trigger(
            () -> (
                leftAlignDebouncer.calculate(
                    xboxDriver.getLeftTriggerButton(0.5).getAsBoolean() && !xboxDriver.getRightTriggerButton(0.5).getAsBoolean()
                )
            )
        ).whileTrue(new AlignToLeftBranchLinearAndScore(xboxDriver)).onFalse(new CancelScoreCoral());

        new Trigger(
            () -> (
                rightAlignDebouncer.calculate(
                    xboxDriver.getRightTriggerButton(0.5).getAsBoolean() && !xboxDriver.getLeftTriggerButton(0.5).getAsBoolean()
                )
            )
        ).whileTrue(new AlignToRightBranchLinearAndScore(xboxDriver)).onFalse(new CancelScoreCoral());

        new Trigger(
            () -> (
                xboxDriver.getRightTriggerButton(0.5).getAsBoolean() && 
                xboxDriver.getLeftTriggerButton(0.5).getAsBoolean()
            )
        ).whileTrue(new AlignToAlgayLinearAndRemove(xboxDriver)).onFalse(new CancelScoreAlgay()); // DescoreAlgay

        new Trigger(
            () -> (
                climbDebouncer.calculate(
                    xboxDriver.getRightBumper().getAsBoolean() &&
                    xboxDriver.getLeftBumper().getAsBoolean() 
                )
            )
        ).whileTrue(new PrepareForClimbSequence()).onFalse(new ClimbSequence());

        // new Trigger(
        //     () -> (
        //         xboxDriver.getYButton().getAsBoolean() && !xboxDriver.getBButton().getAsBoolean()
        //     )
        // ).whileTrue(new AlignToClosestBargePointAndScore(xboxDriver)).onFalse(new CancelScoreAlgay());

        new Trigger(
            () -> (
                bargeDebouncer.calculate(
                    xboxDriver.getYButton().getAsBoolean() && !xboxDriver.getBButton().getAsBoolean()
                )
            )
        ).onTrue(new PrepareMoveSuperstructureBargeSequence()).onFalse(
            new ConditionalCommand(
                new ScoreMoveSuperstrucutreBargeSequence(),
                new CancelScoreAlgay(),
                () -> Elevator.getInstance().getHeight() > 0.5
            )
        );
        
        new Trigger(
            () -> (
                xboxDriver.getBButton().getAsBoolean() && xboxDriver.getYButton().getAsBoolean()
            )
        ).whileTrue(
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    // new SequentialCommandGroup(
                    //     new InClaw(),
                    //     new WaitCommand(0.5)
                    // ), 
                    new MovePivotProcessor(),
                    new RunClawIntake()
                )
            )
        ).onFalse(new CancelScoreAlgay());

        new Trigger(
            () -> (
                processerDebouncer.calculate(
                    xboxDriver.getBButton().getAsBoolean() &&
                    !xboxDriver.getYButton().getAsBoolean()
                )
            )
        ).whileTrue(
            new MovePivotProcessor()
        ).onFalse(
            new ParallelDeadlineGroup(
                new WaitCommand(1),
                new RunClawEject()
            ).andThen(new WaitCommand(1)).andThen(new CancelScoreAlgay())
        ); // Processor


        xboxDriver.getXButton().onTrue(new ZeroPose(() -> new Pose2d(RobotState.getInstance().getEstimatedPose().getTranslation(), new Rotation2d(0))));

        xboxOperator.getAButton().onTrue(new QueueStowAction());
        xboxOperator.getBButton().onTrue(new QueueL2Action());
        xboxOperator.getYButton().onTrue(new QueueL3Action());
        xboxOperator.getXButton().onTrue(new QueueL4Action());

        xboxOperator.getRightMiddleButton().onTrue(new EjectCoral().andThen(new QueueL2Action()).andThen(new DequeueElevatorAction()));
        xboxOperator.getLeftMiddleButton().onTrue(new QueueL4Action().andThen(new DequeueElevatorAction())).onFalse(new QueueStowAction().andThen(new DequeueElevatorAction()));

        xboxOperator.getRightBumper().whileTrue(
            new IntakeCoral().withInterruptBehavior(InterruptionBehavior.kCancelIncoming).andThen(
                new ParallelDeadlineGroup(
                    new WaitCommand(0.7),
                    new RumbleDriver(xboxOperator)
                )
            )
        ).onFalse(new StopRoller());

        xboxOperator.getLeftBumper().onTrue(
            new SequentialCommandGroup(
                new EjectCoral(),
                new QueueStowAction(),
                new DequeueElevatorAction(),
                new InstantCommand(() -> RobotState.getInstance().requestGlobalVisionEstimateScale())
            )
        );

        xboxTester.getRightBumper().whileTrue(
            new IntakeCoral().withInterruptBehavior(InterruptionBehavior.kCancelIncoming).andThen(
                new ParallelDeadlineGroup(
                    new WaitCommand(0.7),
                    new RumbleDriver(xboxOperator)
                )
            )
        ).onFalse(new StopRoller());
        
        xboxTester.getLeftBumper().onTrue(
            new SequentialCommandGroup(
                new EjectCoral(),
                new QueueStowAction(),
                new DequeueElevatorAction(),
                new InstantCommand(() -> RobotState.getInstance().requestGlobalVisionEstimateScale())
            )
        );

        xboxTester.getRightMiddleButton().onTrue(new QueueL2Action().andThen(new DequeueElevatorAction()));
        xboxTester.getLeftMiddleButton().onTrue(new RunRollerReverse()).onFalse(new StopRoller());

        xboxTester.getAButton().onTrue(new QueueStowAction().andThen(new DequeueElevatorAction()));
        xboxTester.getBButton().onTrue(new QueueL2Action().andThen(new DequeueElevatorAction()));
        xboxTester.getYButton().onTrue(new QueueL3Action().andThen(new DequeueElevatorAction()));
        xboxTester.getXButton().onTrue(new QueueL4Action().andThen(new DequeueElevatorAction()));

        new Trigger(
            () -> (
                xboxTester.getLeftTriggerButton(0.94).getAsBoolean() && !xboxTester.getRightTriggerButton(0.5).getAsBoolean()
            )
        ).whileTrue(
            new MovePivotProcessor()
        ).onFalse(
            new ParallelDeadlineGroup(
                new WaitCommand(1.3),
                new RunClawEject()
            ).andThen(new CancelScoreAlgay())
        );

        new Trigger(
            () -> (
                xboxTester.getRightTriggerButton(0.5).getAsBoolean() && !xboxTester.getLeftTriggerButton(0.5).getAsBoolean()
            )
        ).whileTrue(
            new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                    new SequentialCommandGroup(
                        new InClaw(),
                        new WaitCommand(0.5)
                    ), 
                    new MovePivotProcessor(),
                    new RunClawIntake()
                )
            )
        ).onFalse(new CancelScoreAlgay());

        new Trigger(
            () -> (
                xboxTester.getRightTriggerButton(0.5).getAsBoolean() && 
                xboxTester.getLeftTriggerButton(0.5).getAsBoolean()
            )
        ).onTrue(new PrepareMoveSuperstructureBargeSequence()).onFalse(
            new ConditionalCommand(
                new ScoreMoveSuperstrucutreBargeSequence(),
                new CancelScoreAlgay(),
                () -> Elevator.getInstance().getHeight() > 0.5
            )
        );
    }

    public Command getAutonomousCommand() {
        return autoRunner.getAutonomousCommand(); // was static
    }
}

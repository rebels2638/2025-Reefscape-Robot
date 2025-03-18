package frc.robot.commands;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotState;
import frc.robot.commands.autoAlignment.LinearAlignFace;
import frc.robot.commands.claw.simple.RunClawEject;
import frc.robot.commands.claw.simple.RunClawIntake;
import frc.robot.commands.elevator.simple.DequeueElevatorAction;
import frc.robot.commands.elevator.simple.QueueL1Action;
import frc.robot.commands.elevator.simple.QueueL2Action;
import frc.robot.commands.elevator.simple.QueueL3Action;
import frc.robot.commands.elevator.simple.QueueL4Action;
import frc.robot.commands.elevator.simple.QueueStowAction;
import frc.robot.commands.roller.EjectCoral;
import frc.robot.commands.roller.IntakeCoral;
import frc.robot.constants.Constants;
import frc.robot.lib.util.AlignmentUtil;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.Height;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.roller.Roller;
import frc.robot.commands.pivot.simple.MovePivotAlgay;
import frc.robot.commands.pivot.simple.MovePivotStow;

public class Autos {
    public static final Command start_right_2xL4 = 
        new SequentialCommandGroup(
            resetPose("PS_TR_RB"),
            cycleCoral("PS_TR_RB", "TR_RB_RST", Height.L4, Branch.RIGHT),
            cycleCoral("RST_BR_RB", null, Height.L4, Branch.RIGHT)
        );
    
    public enum Branch {
        LEFT,
        RIGHT
    }

    public static final Command cycleCoral(String toReefPath, String toSourcePath, Height level, Branch branch) {
        Command sourceCommand = 
            toSourcePath != null ? 
                new ParallelDeadlineGroup(
                    new ParallelCommandGroup(
                        new SequentialCommandGroup(
                            new QueueStowAction(),
                            new DequeueElevatorAction()
                        ),
                        followPath(toSourcePath)
                    ),
                    new IntakeCoral()
                ) :
                new SequentialCommandGroup(
                    new QueueStowAction(),
                    new DequeueElevatorAction()
                );

        return 
            new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                    new ParallelCommandGroup(
                        waitForAlign(toReefPath),
                        new IntakeCoral(),
                        new SequentialCommandGroup(
                            waitForQueueLocalEstimate(toReefPath),
                            new InstantCommand(() -> RobotState.getInstance().requestLocalVisionEstimateScale(getEndPose(toReefPath)))
                        ),
                        new SequentialCommandGroup(
                            waitForElevatorExtension(toReefPath),
                            queueElevatorCommand(level),
                            new DequeueElevatorAction()
                        ),
                        new MovePivotStow()
                    ),
                    followPath(toReefPath)
                ),
                new LinearAlignFace(
                    branchAlignmentPoseSupplier(branch, getEndPose(toReefPath)),
                    () -> new ChassisSpeeds(),
                    5.0
                ),
                new ConditionalCommand(
                    new WaitCommand(0.5),
                    new InstantCommand(),
                    () -> level == Height.L4
                ),
                new EjectCoral(),
                new InstantCommand(() -> RobotState.getInstance().requestGlobalVisionEstimateScale()),
                sourceCommand
            );
    }

    public static final Command cycleAlgay(String toReefPath, String toBargePath, Height level) {
        Command sourceCommand = 
            toBargePath != null ? 
                new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        followPath(toBargePath),
                        new SequentialCommandGroup(
                            new WaitCommand(1.2),
                            new MovePivotStow(),
                            new QueueStowAction(),
                            new DequeueElevatorAction()
                        )
                    ),
                    new SequentialCommandGroup(
                        waitForElevatorExtension(toReefPath),
                        queueElevatorCommand(Height.L4),
                        new DequeueElevatorAction()
                    ), 
                    new ParallelCommandGroup(
                        new MovePivotAlgay(),
                        new SequentialCommandGroup(
                            new WaitUntilCommand(() -> Pivot.getInstance().getAngle().getDegrees() < 90),
                            new ParallelDeadlineGroup(
                                new WaitUntilCommand(0.8),
                                new RunClawEject()
                            )
                        )
                    ),
                    new MovePivotStow(),
                    new QueueStowAction(),
                    new DequeueElevatorAction() 
                ):
                new SequentialCommandGroup(
                    new LinearAlignFace(
                        () -> AlignmentUtil.getClosestAlgayRecessedPose(getEndPose(toReefPath)),
                        () -> new ChassisSpeeds(),
                        5
                    ),
                    new MovePivotStow(),
                    new QueueStowAction(),
                    new DequeueElevatorAction()
                );

        return 
            new SequentialCommandGroup(
                new InstantCommand(() -> RobotState.getInstance().requestGlobalVisionEstimateScale()),
                new ParallelDeadlineGroup(
                    new ParallelCommandGroup(
                        waitForAlign(toReefPath),
                        new SequentialCommandGroup(
                            waitForElevatorExtension(toReefPath),
                            queueElevatorCommand(level),
                            new DequeueElevatorAction()
                        ),
                        new MovePivotAlgay()
                    ),
                    new RunClawIntake(),
                    followPath(toReefPath)
                ),
                new ParallelDeadlineGroup(
                    new SequentialCommandGroup(
                        new LinearAlignFace(
                            () -> AlignmentUtil.getClosestAlgayPose(),
                            () -> new ChassisSpeeds(),
                            5
                        ),
                        new WaitCommand(0.5)
                    ),
                    new RunClawIntake()
                ),
                sourceCommand
            );
    }

    public static final Supplier<Pose2d> branchAlignmentPoseSupplier(Branch branch, Translation2d pose) {
        switch (branch) {
            case RIGHT:
                return () -> AlignmentUtil.getClosestRightBranchPose(pose);
        
            default:
                return () -> AlignmentUtil.getClosestLeftBranchPose(pose);
        }
    }

    public static final Command queueElevatorCommand(Height level) {
        switch (level) {
            case STOW:
                return new QueueStowAction();
        
            case L1:
                return new QueueL1Action();

            case L2:
                return new QueueL2Action();

            case L3:
                return new QueueL3Action();

            default:
                return new QueueL4Action();
        }

    }
    public static final Command waitForQueueLocalEstimate(String name) {
        return 
            new WaitUntilCommand(
                () -> RobotState.getInstance().getEstimatedPose().getTranslation().getDistance(getEndPose(name)) <= 1.3
            );
    }

    public static final Command waitForElevatorExtension(String name) {
        return
            new WaitUntilCommand(
                () -> 
                    RobotState.getInstance().getEstimatedPose().getTranslation().getDistance(getEndPose(name)) <= 1.6 &&
                    RobotState.getInstance().getIsElevatorExtendable() &&
                    Roller.getInstance().inRoller()
            );
    }

    public static final Command waitForAlign(String name) {
        return 
            new WaitUntilCommand(
                () -> 
                    RobotState.getInstance().getEstimatedPose().getTranslation().getDistance(getEndPose(name)) <= 0.4 
            );
    }

    public static final PathPlannerPath loadPath(String name) {
        try {
            // Load the path you want to follow using its name in the GUI
            return PathPlannerPath.fromChoreoTrajectory(name);

        } catch (Exception e) {
            DriverStation.reportError("Oh shit, ur fucked haha " + e.getMessage(), e.getStackTrace());
            return null;
        }
    }

    public static final Command followPath(String name) {
        return AutoBuilder.followPath(loadPath(name));
    }

    public static final Translation2d getEndPose(String name) {
        PathPlannerPath path = loadPath(name);
        return
            Constants.shouldFlipPath() ?
                FlippingUtil.flipFieldPose(
                    new Pose2d(
                        path.getPoint(path.getAllPathPoints().size() - 1).position,
                        new Rotation2d()
                    )
                ).getTranslation():
                path.getPoint(path.getAllPathPoints().size() - 1).position;
    }

    public static final Command resetPose(String name) {
        PathPlannerPath path = loadPath(name);
        return
            Constants.shouldFlipPath() ?
                new InstantCommand(  // flip starting pose
                    () -> RobotState.getInstance().resetPose(
                        FlippingUtil.flipFieldPose(
                            path.getStartingHolonomicPose().get()
                        )
                    )
                ) :
                new InstantCommand(  // flip starting pose
                    () -> RobotState.getInstance().resetPose(
                        path.getStartingHolonomicPose().get()
                    )
                );
    }

    private Autos() {}
}

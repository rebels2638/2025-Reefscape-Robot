package frc.robot.commands.autoAlignment.reef;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotState;
import frc.robot.commands.AbsoluteFieldDrive;
import frc.robot.commands.autoAlignment.LinearAlignFace;
import frc.robot.commands.claw.simple.HoldAlgayClaw;
import frc.robot.commands.claw.simple.InClaw;
import frc.robot.commands.claw.simple.RunClawIntake;
import frc.robot.commands.elevator.simple.DequeueElevatorAction;
import frc.robot.commands.elevator.simple.QueueStowAction;
import frc.robot.commands.elevator.simple.WaitForNonStowState;
import frc.robot.commands.elevator.simple.isElevatorExtendable;
import frc.robot.commands.pivot.simple.MovePivotAlgay;
import frc.robot.commands.pivot.simple.MovePivotMidwayAlgay;
import frc.robot.commands.pivot.simple.MovePivotStow;
import frc.robot.lib.input.XboxController;
import frc.robot.lib.util.AlignmentUtil;
import frc.robot.subsystems.claw.Claw;

public class AlignToAlgayLinearAndRemove extends SequentialCommandGroup {
    public AlignToAlgayLinearAndRemove(XboxController controller) {
        addCommands(
            new InstantCommand(() -> RobotState.getInstance().requestGlobalVisionEstimateScale()),
            new ParallelDeadlineGroup(
                new WaitUntilCommand( // we wait for this ot be true to allow continual scheduling
                    () -> AlignmentUtil.getClosestLeftBranchPose().getTranslation().getDistance( // check for the correct max distance from target
                    RobotState.getInstance().getEstimatedPose().getTranslation()) <= 5 &&
                    !Claw.getInstance().inClaw()
                ),
                new AbsoluteFieldDrive(controller)
            ),
            new ParallelCommandGroup(
                new MovePivotAlgay(),
                new SequentialCommandGroup(
                    new isElevatorExtendable(),
                    new WaitForNonStowState(),
                    new DequeueElevatorAction()
                ),
                new LinearAlignFace(
                    () -> AlignmentUtil.getClosestAlgayRecessedPose(),
                    () -> new ChassisSpeeds(),
                    5
                )
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
            new LinearAlignFace(
                () -> AlignmentUtil.getClosestAlgayRecessedPose(),
                () -> new ChassisSpeeds(),
                5
            ),
            new MovePivotStow(),
            new QueueStowAction(),
            new DequeueElevatorAction()
        );
    }
}
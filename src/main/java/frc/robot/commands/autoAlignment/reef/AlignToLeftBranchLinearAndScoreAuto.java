package frc.robot.commands.autoAlignment.reef;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotState;
import frc.robot.commands.AbsoluteFieldDrive;
import frc.robot.commands.isElevatorExtendable;
import frc.robot.commands.autoAlignment.LinearAlign;
import frc.robot.commands.autoAlignment.reef.waitForCommands.WaitUntillAlignAuto;
import frc.robot.commands.elevator.simple.DequeueElevatorAction;
import frc.robot.commands.elevator.simple.QueueStowAction;
import frc.robot.commands.elevator.simple.WaitForNonStowState;
import frc.robot.commands.roller.EjectCoral;
import frc.robot.lib.input.XboxController;
import frc.robot.lib.util.AlignmentUtil;
import frc.robot.subsystems.roller.Roller;

public class AlignToLeftBranchLinearAndScoreAuto extends SequentialCommandGroup {
    public AlignToLeftBranchLinearAndScoreAuto(Translation2d pathEndPose) {
        addCommands(
            new InstantCommand(() -> RobotState.getInstance().requestLocalVisionEstimateScale(pathEndPose)),
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new SequentialCommandGroup(
                        new isElevatorExtendable(),
                        new WaitForNonStowState(),
                        new DequeueElevatorAction()
                    ),
                    new LinearAlign(
                        () -> AlignmentUtil.getClosestLeftBranchPose(),
                        () -> new ChassisSpeeds(),
                        2
                    )
                ),
                new EjectCoral(),
                new QueueStowAction(),
                new DequeueElevatorAction()
            ),
            new InstantCommand(() -> RobotState.getInstance().requestGlobalVisionEstimateScale())
        );
    }
}
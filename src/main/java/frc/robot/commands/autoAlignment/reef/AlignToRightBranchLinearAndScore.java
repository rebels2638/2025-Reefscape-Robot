package frc.robot.commands.autoAlignment.reef;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotState;
import frc.robot.commands.isElevatorExtendable;
import frc.robot.commands.autoAlignment.LinearAlign;
import frc.robot.commands.complex.superstructure.Score;
import frc.robot.commands.elevator.simple.DequeueElevatorAction;
import frc.robot.commands.elevator.simple.WaitForNonStowState;
import frc.robot.lib.util.AlignmentUtil;

public class AlignToRightBranchLinearAndScore extends SequentialCommandGroup {
    public AlignToRightBranchLinearAndScore() {
        addCommands(
            new ConditionalCommand(
                new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        new SequentialCommandGroup(
                            new isElevatorExtendable(),
                            new WaitForNonStowState(),
                            new DequeueElevatorAction()
                        ),
                        new LinearAlign(
                            () -> AlignmentUtil.getClosestRightBranchPose(),
                            () -> new ChassisSpeeds(),
                            2
                        )
                    ),
                    new Score()
                ),
                new InstantCommand(), 
                () -> AlignmentUtil.getClosestRightBranchPose().getTranslation().getDistance( // check for the correct max distance from target
                RobotState.getInstance().getEstimatedPose().getTranslation()) <= 2
            )
        );
    }
}
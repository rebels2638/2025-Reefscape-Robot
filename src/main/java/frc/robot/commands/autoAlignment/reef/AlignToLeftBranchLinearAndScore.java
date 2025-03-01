package frc.robot.commands.autoAlignment.reef;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.isElevatorExtendable;
import frc.robot.commands.autoAlignment.LinearAlign;
import frc.robot.commands.complex.superstructure.Score;
import frc.robot.commands.elevator.simple.DequeueElevatorAction;
import frc.robot.lib.util.AlignmentUtil;
public class AlignToLeftBranchLinearAndScore extends SequentialCommandGroup {
    public AlignToLeftBranchLinearAndScore() {
        addCommands(
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new isElevatorExtendable(),
                    new DequeueElevatorAction()
                ),
                new LinearAlign(
                    () -> AlignmentUtil.getClosestLeftBranchPose(),
                    () -> new ChassisSpeeds(),
                    2
                )
            ),
            new Score()
        );
    }
}
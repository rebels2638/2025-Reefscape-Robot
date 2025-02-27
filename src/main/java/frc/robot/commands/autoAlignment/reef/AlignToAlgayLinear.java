package frc.robot.commands.autoAlignment.reef;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.isElevatorExtendable;
import frc.robot.commands.autoAlignment.LinearAlign;
import frc.robot.commands.elevator.simple.DequeueElevatorAction;
import frc.robot.lib.util.AlignmentUtil;
public class AlignToAlgayLinear extends SequentialCommandGroup {
    public AlignToAlgayLinear() {
        addCommands(
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new isElevatorExtendable(),
                    new DequeueElevatorAction()
                ),
                new LinearAlign(
                    () -> AlignmentUtil.getClosestAlgayPose(),
                    () -> new ChassisSpeeds(),
                    2
                )
            )
        );
    }
}
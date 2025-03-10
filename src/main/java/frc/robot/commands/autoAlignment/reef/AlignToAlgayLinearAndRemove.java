package frc.robot.commands.autoAlignment.reef;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.isElevatorExtendable;
import frc.robot.commands.autoAlignment.LinearAlign;
import frc.robot.commands.elevator.simple.DequeueElevatorAction;
import frc.robot.commands.elevator.simple.WaitForNonStowState;
import frc.robot.commands.pivot.RemoveAlgay;
import frc.robot.lib.util.AlignmentUtil;
import frc.robot.subsystems.claw.Claw;

public class AlignToAlgayLinearAndRemove extends SequentialCommandGroup {
    public AlignToAlgayLinearAndRemove() {
        addCommands(
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new isElevatorExtendable(),
                    new WaitForNonStowState(),
                    new ConditionalCommand(
                        new DequeueElevatorAction(), 
                        new InstantCommand(), 
                        () -> Claw.getInstance().inClaw()
                    )
                ),
                new LinearAlign(
                    () -> AlignmentUtil.getClosestAlgayPose(),
                    () -> new ChassisSpeeds(),
                    2
                ),
                new RemoveAlgay()
            )
        );
    }
}
package frc.robot.commands.autoAlignment.reef;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotState;
import frc.robot.commands.isElevatorExtendable;
import frc.robot.commands.autoAlignment.LinearAlign;
import frc.robot.commands.claw.simple.HoldAlgayClaw;
import frc.robot.commands.claw.simple.RunClawIntake;
import frc.robot.commands.elevator.simple.DequeueElevatorAction;
import frc.robot.commands.elevator.simple.QueueStowAction;
import frc.robot.commands.elevator.simple.WaitForNonStowState;
import frc.robot.commands.pivot.simple.MovePivotAlgay;
import frc.robot.commands.pivot.simple.MovePivotMidwayAlgay;
import frc.robot.commands.pivot.simple.MovePivotStow;
import frc.robot.lib.util.AlignmentUtil;
import frc.robot.subsystems.claw.Claw;

public class AlignToAlgayLinearAndRemove extends SequentialCommandGroup {
    public AlignToAlgayLinearAndRemove() {
        addCommands(
            new ParallelCommandGroup(
                new MovePivotAlgay(),
                new RunClawIntake(),
                new SequentialCommandGroup(
                    new isElevatorExtendable(),
                    new WaitForNonStowState(),
                    new DequeueElevatorAction()
                ),
                new LinearAlign(
                    () -> AlignmentUtil.getClosestAlgayRecessedPose(),
                    () -> new ChassisSpeeds(),
                    2
                )
            ),
            new LinearAlign(
                () -> AlignmentUtil.getClosestAlgayPose(),
                () -> new ChassisSpeeds(),
                2
            ),
            new HoldAlgayClaw(),
            new ParallelCommandGroup(
                new LinearAlign(
                    () -> AlignmentUtil.getClosestAlgayRecessedPose(),
                    () -> new ChassisSpeeds(),
                    2
                ),
                new MovePivotMidwayAlgay()
            ),
            new MovePivotStow(),
            new QueueStowAction(),
            new DequeueElevatorAction()
        );
    }
}
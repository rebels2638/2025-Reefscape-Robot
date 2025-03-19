package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.claw.simple.HoldAlgayClaw;
import frc.robot.commands.claw.simple.StopClaw;
import frc.robot.commands.elevator.simple.DequeueElevatorAction;
import frc.robot.commands.elevator.simple.QueueStowAction;
import frc.robot.commands.pivot.simple.MovePivotStow;

public class CancelScoreAlgay extends SequentialCommandGroup {
    public CancelScoreAlgay() {
        addCommands(
            new QueueStowAction(),
            new MovePivotStow(),
            new DequeueElevatorAction()

            // new ParallelCommandGroup(
            //     new HoldAlgayClaw()
            //     new DequeueElevatorAction()
            // )
        );
    }
    
}

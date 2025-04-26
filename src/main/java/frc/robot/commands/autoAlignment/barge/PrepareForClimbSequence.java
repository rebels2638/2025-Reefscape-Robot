package frc.robot.commands.autoAlignment.barge;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.climber.simple.MoveDeepCage;
import frc.robot.commands.elevator.simple.DequeueElevatorAction;
import frc.robot.commands.elevator.simple.QueueL2Action;
import frc.robot.commands.pivot.simple.MovePivotClimb;
import frc.robot.commands.pivot.simple.MovePivotStow;

public class PrepareForClimbSequence extends SequentialCommandGroup{
    public PrepareForClimbSequence() {
        addCommands(
            new QueueL2Action(),
            new ParallelCommandGroup(
                new MovePivotClimb(),
                new SequentialCommandGroup(
                    new DequeueElevatorAction(),
                    new MoveDeepCage()
                )
            )
        );
    }
}
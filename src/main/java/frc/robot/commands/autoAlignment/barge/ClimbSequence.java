package frc.robot.commands.autoAlignment.barge;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.climber.simple.MoveClimberStow;
import frc.robot.commands.elevator.simple.DequeueElevatorAction;
import frc.robot.commands.elevator.simple.QueueL2Action;
import frc.robot.commands.elevator.simple.QueuePostClimbAction;
import frc.robot.commands.pivot.simple.MovePivotIdle;
import frc.robot.commands.pivot.simple.MovePivotStow;

public class ClimbSequence extends SequentialCommandGroup{
    public ClimbSequence() {
        addCommands(
            new QueueL2Action(),
            new DequeueElevatorAction(),
            new MoveClimberStow(),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new QueuePostClimbAction(),
                    new DequeueElevatorAction()
                ),
                new MovePivotIdle()
            )
        );
    }
}
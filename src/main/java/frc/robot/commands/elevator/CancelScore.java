package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.simple.DequeueElevatorAction;
import frc.robot.commands.elevator.simple.QueueStowAction;
import frc.robot.commands.pivot.simple.MovePivotStow;
import frc.robot.commands.roller.simple.StopRoller;

public class CancelScore extends SequentialCommandGroup {
    public CancelScore() {
        addCommands(
            new MovePivotStow(),
            new QueueStowAction(),
            new DequeueElevatorAction(),
            new StopRoller()
        );
    }
    
}

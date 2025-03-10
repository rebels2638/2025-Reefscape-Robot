package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.simple.DequeueElevatorAction;
import frc.robot.commands.elevator.simple.QueueStowAction;
import frc.robot.commands.roller.simple.StopRoller;

public class CancelScoreCoral extends SequentialCommandGroup {
    public CancelScoreCoral() {
        addCommands(
            new QueueStowAction(),
            new ParallelCommandGroup(
                new StopRoller(),
                new DequeueElevatorAction()
            )
        );
    }
    
}

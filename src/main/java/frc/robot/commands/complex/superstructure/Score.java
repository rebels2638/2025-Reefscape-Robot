package frc.robot.commands.complex.superstructure;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.simple.DequeueElevatorAction;
import frc.robot.commands.elevator.simple.QueueStowAction;
import frc.robot.commands.roller.EjectCoral;

public class Score extends SequentialCommandGroup {
    public Score() {
        addCommands(
            new EjectCoral(),
            new QueueStowAction(),
            new DequeueElevatorAction()
        );
    }
}

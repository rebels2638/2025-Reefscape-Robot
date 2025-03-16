package frc.robot.commands.climber.simple;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.simple.DequeueElevatorAction;
import frc.robot.commands.elevator.simple.QueueStowAction;

public class ResetClimb extends SequentialCommandGroup {
    public ResetClimb() {
        addCommands(
            new EnableOppositeRotation(),
            new MoveDeepCage(),
            new QueueStowAction(),
            new DequeueElevatorAction()
        );
    }
}

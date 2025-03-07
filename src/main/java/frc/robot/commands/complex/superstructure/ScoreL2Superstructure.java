package frc.robot.commands.complex.superstructure;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.simple.DequeueElevatorAction;
import frc.robot.commands.elevator.simple.QueueL2Action;
import frc.robot.commands.elevator.simple.QueueStowAction;
import frc.robot.commands.pivot.simple.MovePivotStow;
import frc.robot.commands.roller.EjectCoral;

public class ScoreL2Superstructure extends SequentialCommandGroup {
    public ScoreL2Superstructure() {
        addCommands(
            // new WarmUpElevatorReef(),
            new QueueL2Action(),
            new DequeueElevatorAction(),
            new EjectCoral(),
            new QueueStowAction(),
            new DequeueElevatorAction()
        );
    }
}

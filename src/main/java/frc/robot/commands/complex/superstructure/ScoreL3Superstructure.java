package frc.robot.commands.complex.superstructure;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autoAlignment.reef.WarmUpElevatorReef;
import frc.robot.commands.elevator.simple.DequeueElevatorAction;
import frc.robot.commands.elevator.simple.QueueL3Action;
import frc.robot.commands.elevator.simple.QueueStowAction;
import frc.robot.commands.pivot.simple.MovePivotStow;
import frc.robot.commands.roller.EjectCoral;

public class ScoreL3Superstructure extends SequentialCommandGroup {
    public ScoreL3Superstructure() {
        addCommands(
            // new WarmUpElevatorReef(),
            new QueueL3Action(),
            new DequeueElevatorAction(),
            new EjectCoral(),
            new QueueStowAction(),
            new DequeueElevatorAction()
        );
    }
}

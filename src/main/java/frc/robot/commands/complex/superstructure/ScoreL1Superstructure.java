package frc.robot.commands.complex.superstructure;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autoAlignment.reef.WarmUpElevatorReef;
import frc.robot.commands.elevator.simple.QueueL1Action;
import frc.robot.commands.elevator.simple.QueueStowAction;
import frc.robot.commands.pivot.simple.MovePivotStow;
import frc.robot.commands.roller.EjectCoral;

public class ScoreL1Superstructure extends SequentialCommandGroup {
    public ScoreL1Superstructure() {
        addCommands(
            // new WarmUpElevatorReef(),
            new QueueL1Action(),
            new EjectCoral(),
            new QueueStowAction()
        );
    }
}

package frc.robot.commands.complex.superstructure;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.simple.DequeueElevatorAction;
import frc.robot.commands.elevator.simple.QueueStowAction;
import frc.robot.commands.roller.EjectCoral;
import frc.robot.subsystems.roller.Roller;

public class Score extends SequentialCommandGroup {
    public Score() {
        addCommands(
            new ConditionalCommand(
                new EjectCoral(), 
                new InstantCommand(), 
                () -> Roller.getInstance().inRoller()
            ),
            new QueueStowAction(),
            new DequeueElevatorAction()
        );
    }
}

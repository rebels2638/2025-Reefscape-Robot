package frc.robot.commands.complex.superstructure;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autoAlignment.reef.WarmUpElevatorReef;
import frc.robot.commands.elevator.simple.MoveElevatorL4;
import frc.robot.commands.elevator.simple.MoveElevatorStow;
import frc.robot.commands.roller.EjectCoral;

public class ScoreL4Superstructure extends SequentialCommandGroup {
    public ScoreL4Superstructure() {
        addCommands(
            new WarmUpElevatorReef(),
            new MoveElevatorL4(),
            new EjectCoral(),
            new MoveElevatorStow()
        );
    }
}

package frc.robot.commands.complex.superstructure;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autoAlignment.reef.WarmUpElevatorReef;
import frc.robot.commands.elevator.simple.MoveElevatorL3;
import frc.robot.commands.elevator.simple.MoveElevatorStow;
import frc.robot.commands.roller.EjectCoral;

public class ScoreL3Superstructure extends SequentialCommandGroup {
    public ScoreL3Superstructure() {
        addCommands(
            // new WarmUpElevatorReef(),
            new MoveElevatorL3(),
            new EjectCoral(),
            new MoveElevatorStow()
        );
    }
}

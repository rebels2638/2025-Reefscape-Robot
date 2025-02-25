package frc.robot.commands.complex.superstructure;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autoAlignment.reef.WarmUpElevatorReef;
import frc.robot.commands.elevator.simple.MoveElevatorL1;
import frc.robot.commands.elevator.simple.MoveElevatorStow;
import frc.robot.commands.roller.EjectCoral;

public class ScoreL1Superstructure extends SequentialCommandGroup {
    public ScoreL1Superstructure() {
        addCommands(
            // new WarmUpElevatorReef(),
            new MoveElevatorL1(),
            new EjectCoral(),
            new MoveElevatorStow()
        );
    }
}

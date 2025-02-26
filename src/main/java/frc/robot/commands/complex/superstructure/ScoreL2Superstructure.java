package frc.robot.commands.complex.superstructure;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autoAlignment.reef.WarmUpElevatorReef;
import frc.robot.commands.elevator.simple.MoveElevatorL2;
import frc.robot.commands.elevator.simple.MoveElevatorStow;
import frc.robot.commands.pivot.simple.MovePivotStow;
import frc.robot.commands.roller.EjectCoral;

public class ScoreL2Superstructure extends SequentialCommandGroup {
    public ScoreL2Superstructure() {
        addCommands(
            new MovePivotStow(),
            new WarmUpElevatorReef(),
            new MoveElevatorL2(),
            new EjectCoral(),
            new MoveElevatorStow()
        );
    }
}

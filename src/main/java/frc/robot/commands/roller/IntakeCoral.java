package frc.robot.commands.roller;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.roller.simple.InRoller;
import frc.robot.commands.roller.simple.RunRoller;
import frc.robot.commands.roller.simple.StopRoller;

public class IntakeCoral extends SequentialCommandGroup{
    public IntakeCoral() {
        addCommands(
            new ParallelDeadlineGroup(
                new InRoller(),
                new RunRoller()
            ),
            new StopRoller()
        );
    }
}

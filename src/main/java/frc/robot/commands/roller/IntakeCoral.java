package frc.robot.commands.roller;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.roller.simple.InRoller;
import frc.robot.commands.roller.simple.RunRollerIntake;
import frc.robot.commands.roller.simple.StopRoller;

public class IntakeCoral extends SequentialCommandGroup{
    public IntakeCoral() {
        addCommands(
            new InstantCommand(()->System.out.println("new coral")),
            new ParallelDeadlineGroup(
                new InRoller(),
                new RunRollerIntake()
            ),
            new StopRoller()
        );
    }
}

package frc.robot.commands.roller;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.roller.simple.InRoller;
import frc.robot.commands.roller.simple.RunRollerIntake;
import frc.robot.commands.roller.simple.StopRoller;
import frc.robot.subsystems.roller.Roller;

public class IntakeCoral extends ConditionalCommand{
    public IntakeCoral() {
        super(
            new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                    new InRoller(),
                    new RunRollerIntake()
                ),
                new StopRoller()   
            ),
            new InstantCommand(),
            () -> !Roller.getInstance().inRoller()
        );
    }
}

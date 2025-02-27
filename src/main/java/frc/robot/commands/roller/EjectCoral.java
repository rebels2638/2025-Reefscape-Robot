package frc.robot.commands.roller;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.roller.simple.OutRoller;
import frc.robot.commands.roller.simple.RunRollerIntake;
import frc.robot.commands.roller.simple.RunRollerEject;
import frc.robot.commands.roller.simple.StopRoller;

public class EjectCoral extends SequentialCommandGroup{
    public EjectCoral() {
        addCommands(
            new ParallelDeadlineGroup(
                new OutRoller(),
                new RunRollerEject()
            ),
            new WaitCommand(.3),
            new StopRoller()
        );
    }
}

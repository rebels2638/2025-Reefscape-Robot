package frc.robot.commands.roller;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.roller.simple.RunRollerEjectL4;
import frc.robot.commands.roller.simple.OutRoller;
import frc.robot.commands.roller.simple.RunRollerEject;
import frc.robot.commands.roller.simple.StopRoller;

public class EjectCoralL4 extends SequentialCommandGroup{
    public EjectCoralL4() {
        addCommands(
            new ParallelDeadlineGroup(
                new OutRoller(),
                new RunRollerEjectL4()
            ),
            new WaitCommand(0.15),
            new StopRoller()
        );
    }
}

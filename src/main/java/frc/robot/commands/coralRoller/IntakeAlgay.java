package frc.robot.commands.coralRoller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.roller.Roller;

public class IntakeAlgay extends SequentialCommandGroup {
    private final Roller roller = new Roller();

    public IntakeAlgay() {
        addCommands(
            new ParallelDeadlineGroup(new InIntake(roller), new RollerRun(roller)),
            new RollerStop(roller));
    }
}

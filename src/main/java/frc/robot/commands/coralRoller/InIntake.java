package frc.robot.commands.coralRoller;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.roller.Roller;

public class InIntake extends Command {
    private final Roller roller;

    public InIntake(Roller rollerSubsystem) {
        this.roller = rollerSubsystem;
        addRequirements(rollerSubsystem);
    }

    @Override
    public boolean isFinished() {return this.roller.getLimSwitchState();}
}

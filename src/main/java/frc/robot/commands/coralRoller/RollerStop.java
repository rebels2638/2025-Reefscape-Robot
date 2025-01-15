package frc.robot.commands.coralRoller;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.roller.Roller;

public class RollerStop extends Command {
    private final Roller roller;
    public RollerStop(Roller roller) {
        this.roller = roller;

        addRequirements(roller);
    }

    @Override
    public void initialize() {
        roller.setVoltage(0);
    }

    @Override
    public boolean isFinished() {
        //return roller.reachedSetpoint();
        return true;
    }

    @Override
    public void end(boolean interrupted) {}
}

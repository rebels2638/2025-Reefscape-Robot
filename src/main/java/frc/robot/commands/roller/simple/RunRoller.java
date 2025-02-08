package frc.robot.commands.roller.simple;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.roller.Roller;

public class RunRoller extends Command {
    private final Roller roller = Roller.getInstance();

    public RunRoller() {
        addRequirements(roller);
    }

    @Override
    public void initialize() {
        roller.setVoltage(-12);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

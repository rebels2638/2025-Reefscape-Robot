package frc.robot.commands.roller.simple;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.roller.Roller;

public class RunRollerOutake extends Command {
    private final Roller roller = Roller.getInstance();

    public RunRollerOutake() {
        addRequirements(roller);
    }

    @Override
    public void initialize() {
        roller.setVoltage(-7);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

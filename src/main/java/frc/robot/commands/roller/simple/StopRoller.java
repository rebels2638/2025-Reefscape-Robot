package frc.robot.commands.roller.simple;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.roller.Roller;

public class StopRoller extends Command {
    private final Roller roller = Roller.getInstance();

    public StopRoller() {
        addRequirements(roller);
    }
    
    @Override
    public void initialize() {
        roller.setVoltage(0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

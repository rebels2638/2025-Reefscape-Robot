package frc.robot.commands.roller.simple;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.roller.Roller;

public class RunRollerEject extends Command {
    private final Roller roller = Roller.getInstance();

    public RunRollerEject() {
        addRequirements(roller);
    }

    @Override
    public void initialize() {
        roller.setTorqueCurrentFOC(50);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

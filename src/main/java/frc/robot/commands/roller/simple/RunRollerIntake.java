package frc.robot.commands.roller.simple;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.roller.Roller;

public class RunRollerIntake extends Command {
    private final Roller roller = Roller.getInstance();

    public RunRollerIntake() {
        addRequirements(roller);
    }

    @Override
    public void initialize() {
        roller.setVoltage(7);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

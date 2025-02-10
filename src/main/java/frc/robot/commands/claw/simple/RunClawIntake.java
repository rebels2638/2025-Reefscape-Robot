package frc.robot.commands.claw.simple;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.claw.Claw;

public class RunClawIntake extends Command{
    private final Claw claw;
    public RunClawIntake(Claw claw) {
        this.claw = claw;
        addRequirements(claw);
    }

    @Override
    public void initialize() {
        claw.setTorqueCurrentFOC(7);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

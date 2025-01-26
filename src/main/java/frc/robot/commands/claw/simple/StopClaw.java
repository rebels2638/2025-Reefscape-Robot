package frc.robot.commands.claw.simple;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.claw.Claw;

public class StopClaw extends Command{
    private final Claw claw;
    public StopClaw(Claw claw) {
        this.claw = claw;
        addRequirements(claw);
    }

    @Override
    public void initialize() {
        claw.setTorqueCurrentFOC(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

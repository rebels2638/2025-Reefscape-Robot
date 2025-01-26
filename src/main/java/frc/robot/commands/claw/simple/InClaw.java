package frc.robot.commands.claw.simple;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.claw.Claw;

public class InClaw extends Command{
    private final Claw claw;
    public InClaw(Claw claw) {
        this.claw = claw;
    }

    @Override
    public boolean isFinished() {
        return claw.inClaw();
    }
}

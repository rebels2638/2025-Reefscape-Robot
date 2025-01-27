package frc.robot.commands.claw.simple;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.superstructure.Superstructure;

public class InClaw extends Command{
    private final Superstructure superstructure;
    public InClaw(Superstructure superstructure) {
        this.superstructure = superstructure;
    }

    @Override
    public boolean isFinished() {
        return superstructure.inClaw();
    }
}

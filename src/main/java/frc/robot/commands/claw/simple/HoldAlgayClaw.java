package frc.robot.commands.claw.simple;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.claw.Claw;

public class HoldAlgayClaw extends Command{
    private final Claw claw;
    public HoldAlgayClaw() {
        claw = Claw.getInstance();
        addRequirements(claw);
    }

    @Override
    public void execute() {
        if (claw.inClaw()) {
            claw.setVoltage(12);
        }
        else {
            claw.setVoltage(0);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

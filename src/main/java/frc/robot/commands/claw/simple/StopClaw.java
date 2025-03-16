package frc.robot.commands.claw.simple;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.claw.Claw;

public class StopClaw extends Command{
    public StopClaw() {
        addRequirements(Claw.getInstance());
    }

    @Override
    public void initialize() {
        Claw.getInstance().setVoltage(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

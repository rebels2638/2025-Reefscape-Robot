package frc.robot.commands.claw.simple;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.claw.Claw;

public class RunClawHold extends Command{
    public RunClawHold() {
    }

    @Override
    public void initialize() {
        Claw.getInstance().setVoltage(-6);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

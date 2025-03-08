package frc.robot.commands.claw.simple;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.claw.Claw;

public class RunClawEject extends Command{
    public RunClawEject() {
    }

    @Override
    public void initialize() {
        Claw.getInstance().setVoltage(-12);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

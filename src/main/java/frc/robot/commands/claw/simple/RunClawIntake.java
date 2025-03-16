package frc.robot.commands.claw.simple;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.claw.Claw;

public class RunClawIntake extends Command{
    public RunClawIntake() {
        addRequirements(Claw.getInstance());
    }

    @Override
    public void initialize() {
        Claw.getInstance().setVoltage(12);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

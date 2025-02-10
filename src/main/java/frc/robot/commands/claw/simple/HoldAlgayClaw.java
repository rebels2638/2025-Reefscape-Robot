package frc.robot.commands.claw.simple;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.claw.Claw;

public class HoldAlgayClaw extends Command{
    private final Claw claw;
    public HoldAlgayClaw(Claw claw) {
        this.claw = claw;
        addRequirements(claw);
    }

    @Override
    public void initialize() {
        claw.setVoltage(6);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

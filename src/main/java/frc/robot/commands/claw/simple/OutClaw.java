package frc.robot.commands.claw.simple;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.claw.Claw;

public class OutClaw extends Command {
    @Override
    public boolean isFinished() {
        return !Claw.getInstance().inClaw();
    }
}
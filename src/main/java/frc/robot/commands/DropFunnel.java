package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pneumatics.Pneumatics;

public class DropFunnel extends Command {
    public DropFunnel() {}

    @Override
    public void initialize() {
        Pneumatics.getInstance().pullFunnel();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;

public class DoSomething extends Command{
    @Override
    public void initialize() {
        Logger.recordOutput("DoSomething", true);
    }
}

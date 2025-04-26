// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.roller.simple;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.roller.Roller;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunRollerEjectL4 extends Command {
  private final Roller roller = Roller.getInstance();

    public RunRollerEjectL4() {
        addRequirements(roller);
    }

    @Override
    public void initialize() {
        roller.setVoltage(8);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

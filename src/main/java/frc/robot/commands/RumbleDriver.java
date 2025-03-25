package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.input.XboxController;

public class RumbleDriver extends Command {
    private final XboxController controller;
    public RumbleDriver(XboxController controller) {
        this.controller = controller;
    }

    @Override
    public void initialize() {
        controller.setRumble(RumbleType.kBothRumble, 1);
    }

    @Override
    public void end(boolean interrupted) {
        controller.setRumble(RumbleType.kBothRumble, 0);
    }
}

package frc.robot.commands.roller.simple;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.roller.Roller;

public class OutRoller extends Command {
    private final Roller roller = Roller.getInstance();

    @Override
    public boolean isFinished() {
        return !roller.inRoller();
    }
}

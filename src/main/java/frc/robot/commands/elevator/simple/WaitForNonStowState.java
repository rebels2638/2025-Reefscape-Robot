package frc.robot.commands.elevator.simple;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.Height;

public class WaitForNonStowState extends Command {
    @Override
    public boolean isFinished() {
        return Elevator.getInstance().getRequestedLevel() != Height.STOW;
    }
}

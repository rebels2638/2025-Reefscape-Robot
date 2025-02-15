package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;

public class isElevatorExtendable extends Command {
    public isElevatorExtendable() {}

    @Override
    public boolean isFinished() {
        return RobotState.getInstance().getIsElevatorExtendable();
    }
}

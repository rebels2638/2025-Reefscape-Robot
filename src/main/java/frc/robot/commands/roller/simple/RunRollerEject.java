package frc.robot.commands.roller.simple;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.roller.Roller;

public class RunRollerEject extends Command {
    private final Roller roller = Roller.getInstance();

    public RunRollerEject() {
        addRequirements(roller);
    }

    @Override
    public void initialize() {
        roller.setVoltage(-12);
        roller.isScored(RobotState.getInstance().getScoredPosition(RobotState.getInstance().getEstimatedPose()), Elevator.getInstance().getElevatorHeight());
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

package frc.robot.commands.roller.simple;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.roller.Roller;

public class RunRollerIntake extends Command {
    private final Roller roller = Roller.getInstance();

    public RunRollerIntake() {
        addRequirements(roller);
    }

    @Override
    public void initialize() {
        roller.setVoltage(-12);
        Roller.getInstance().isScored(RobotState.getInstance().getScoredPosition(RobotState.getInstance().getEstimatedPose()), Elevator.getInstance().getElevatorHeight());
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

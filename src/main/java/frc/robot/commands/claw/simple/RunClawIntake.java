package frc.robot.commands.claw.simple;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.elevator.Elevator;

public class RunClawIntake extends Command{
    public RunClawIntake() {
    }

    @Override
    public void initialize() {
        Claw.getInstance().setTorqueCurrentFOC(7);
        Claw.getInstance().isScored(RobotState.getInstance().getScoredPosition(RobotState.getInstance().getEstimatedPose()), Elevator.getInstance().getElevatorHeight());
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

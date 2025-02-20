package frc.robot.commands.autoAlignment.reef;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.lib.util.AlignmentUtil;
import frc.robot.subsystems.elevator.Elevator;

public class WarmUpElevatorReef extends Command {
    private final Elevator elevator;
    private final RobotState robotState;

    public WarmUpElevatorReef() {
        this.elevator = Elevator.getInstance();
        this.robotState = RobotState.getInstance();

        addRequirements(elevator);
    }

    @Override
    public boolean isFinished() {
        Logger.recordOutput("WarmUpElevatorReef/closestReefFace", AlignmentUtil.getClosestReefFaceSimple(robotState.getEstimatedPose()));
        boolean isFinished =  
            robotState.getIsElevatorExtendable() && 
            AlignmentUtil.
                getClosestReefFaceSimple(robotState.getEstimatedPose()).
                getTranslation().
                getDistance(robotState.getEstimatedPose().
                getTranslation()) <= 2;

        Logger.recordOutput("WarmUpElevatorReef/a", isFinished);
        return isFinished;
    }
}

package frc.robot.commands.claw.simple;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.simple.DequeueElevatorAction;
import frc.robot.commands.elevator.simple.QueueStowAction;
import frc.robot.commands.pivot.simple.MovePivotAlgay;
import frc.robot.commands.pivot.simple.MovePivotStow;
import frc.robot.RobotState;

public class DecideBargeScoringFlick extends ConditionalCommand {
    public DecideBargeScoringFlick() {
        super(
            new SequentialCommandGroup(
                new MovePivotStow(),
                new DequeueElevatorAction(),
                new MovePivotAlgay(),
                new QueueStowAction(),
                new DequeueElevatorAction()
            ), 
            new SequentialCommandGroup(
                new MovePivotAlgay(),
                new DequeueElevatorAction(),
                new MovePivotStow(),
                new QueueStowAction(),
                new DequeueElevatorAction(),
                new MovePivotAlgay()
            ), 
            () -> RobotState.getInstance().getEstimatedPose().getRotation().getDegrees() == 0
        ); 
    }
}

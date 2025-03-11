package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotState;
import frc.robot.commands.elevator.simple.DequeueElevatorAction;
import frc.robot.commands.elevator.simple.QueueStowAction;
import frc.robot.commands.roller.simple.StopRoller;

public class CancelScoreCoral extends SequentialCommandGroup {
    public CancelScoreCoral() {
        addCommands(
            new InstantCommand(() -> RobotState.getInstance().requestGlobalVisionEstimateScale()),
            new QueueStowAction(),
            new ParallelCommandGroup(
                new StopRoller(),
                new DequeueElevatorAction()
            )
        );
    }
    
}

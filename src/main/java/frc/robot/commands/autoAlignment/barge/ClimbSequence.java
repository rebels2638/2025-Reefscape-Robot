package frc.robot.commands.autoAlignment.barge;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DropFunnel;
import frc.robot.commands.climber.simple.MoveClimberStow;
import frc.robot.commands.elevator.simple.DequeueElevatorAction;
import frc.robot.commands.elevator.simple.QueueL2Action;

public class ClimbSequence extends SequentialCommandGroup{
    public ClimbSequence() {
        addCommands(
            new DropFunnel(),
            new QueueL2Action(),
            new DequeueElevatorAction(),
            new MoveClimberStow()
        );
    }
}
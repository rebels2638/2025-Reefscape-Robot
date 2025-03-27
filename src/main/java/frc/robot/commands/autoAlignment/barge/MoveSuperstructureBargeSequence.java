package frc.robot.commands.autoAlignment.barge;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.claw.simple.HoldAlgayClaw;
import frc.robot.commands.claw.simple.RunClawEject;
import frc.robot.commands.elevator.simple.DequeueElevatorAction;
import frc.robot.commands.elevator.simple.QueueL4Action;
import frc.robot.commands.elevator.simple.QueueStowAction;
import frc.robot.commands.pivot.simple.MovePivotAlgay;
import frc.robot.commands.pivot.simple.MovePivotBargeBackwards;
import frc.robot.commands.pivot.simple.MovePivotBargeForwards;
import frc.robot.commands.pivot.simple.MovePivotStow;
import frc.robot.commands.roller.simple.StopRoller;
import frc.robot.subsystems.pivot.Pivot;

public class MoveSuperstructureBargeSequence extends SequentialCommandGroup {
    public MoveSuperstructureBargeSequence() {
        super(
            new QueueL4Action(),                  
            new SequentialCommandGroup(
                new MovePivotAlgay(),
                new DequeueElevatorAction(),
                new ParallelCommandGroup(
                    new MovePivotBargeBackwards(),
                    new SequentialCommandGroup(
                        new WaitUntilCommand(() -> Pivot.getInstance().getAngle().getDegrees() > 90),
                        new WaitCommand(0.2),
                        new ParallelDeadlineGroup(
                            new RunClawEject(),
                            new WaitCommand(0.7)
                        ),
                        new StopRoller()
                    )
                ),
                new MovePivotStow(),
                new QueueStowAction(),
                new DequeueElevatorAction()
            )
        );
    } 
}

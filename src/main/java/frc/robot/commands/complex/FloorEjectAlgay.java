package frc.robot.commands.complex;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.claw.simple.RunClawEject;
import frc.robot.commands.claw.simple.StopClaw;
import frc.robot.commands.elevator.simple.DequeueElevatorAction;
import frc.robot.commands.elevator.simple.QueueStowAction;
import frc.robot.commands.pivot.simple.MovePivotAlgay;
import frc.robot.commands.pivot.simple.MovePivotStow;

public class FloorEjectAlgay extends SequentialCommandGroup {
    public FloorEjectAlgay() {
        addCommands(
            new QueueStowAction(),
            new DequeueElevatorAction(),
            new MovePivotAlgay(),
            new RunClawEject(),
            new WaitCommand(1.3),
            new StopClaw(),
            new MovePivotStow()
        );
    }
}
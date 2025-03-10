package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.claw.simple.HoldAlgayClaw;
import frc.robot.commands.claw.simple.InClaw;
import frc.robot.commands.claw.simple.RunClawIntake;
import frc.robot.commands.pivot.simple.MovePivotAlgay;
import frc.robot.commands.pivot.simple.MovePivotStow;
import frc.robot.subsystems.claw.Claw;

public class RemoveAlgay extends ConditionalCommand {
    public RemoveAlgay() {
        super(
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new MovePivotAlgay(),
                    new RunClawIntake(),
                    new InClaw()
                ),
                new ParallelCommandGroup(
                    new HoldAlgayClaw(),
                    new MovePivotStow()
                ) 
            ), 
            new InstantCommand(), 
            () -> !Claw.getInstance().inClaw()
        );
    }
}

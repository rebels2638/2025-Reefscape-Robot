package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.claw.simple.HoldAlgayClaw;
import frc.robot.commands.claw.simple.InClaw;
import frc.robot.commands.claw.simple.RunClawIntake;
import frc.robot.subsystems.claw.Claw;

public class IntakeAlgay extends SequentialCommandGroup {
    public IntakeAlgay(Claw claw) {
        addCommands(
            new ParallelDeadlineGroup(
                new InClaw(claw),
                new RunClawIntake(claw)
            ),
            new HoldAlgayClaw(claw)
        );
    }
}

package frc.robot.commands.claw.simple;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class BargeScoringManualShot extends SequentialCommandGroup {
    public BargeScoringManualShot() {
        addCommands(
            new ParallelDeadlineGroup(
                new OutClaw(), 
                new RunClawEject()
            )
        );
    }
    
}

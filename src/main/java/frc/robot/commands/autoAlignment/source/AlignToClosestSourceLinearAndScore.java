package frc.robot.commands.autoAlignment.source;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autoAlignment.LinearAlign;
import frc.robot.lib.util.AlignmentUtil;
public class AlignToClosestSourceLinearAndScore extends SequentialCommandGroup {
    public AlignToClosestSourceLinearAndScore() {
        addCommands(
            new LinearAlign(
                () -> AlignmentUtil.getClosestSourcePose(),
                () -> new ChassisSpeeds(),
                2
            )
        );
    }
}
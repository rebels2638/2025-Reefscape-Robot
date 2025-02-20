package frc.robot.commands.autoAlignment.source;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.commands.autoAlignment.LinearAlign;
import frc.robot.lib.util.AlignmentUtil;
public class AlignToClosestSourceLinear extends LinearAlign {
    public AlignToClosestSourceLinear() {
        super(
            () -> AlignmentUtil.getClosestSourcePose(),
            () -> new ChassisSpeeds(),
            2
        );
    }
}
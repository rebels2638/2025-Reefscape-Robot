package frc.robot.commands.autoAlignment.reef;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.commands.autoAlignment.LinearAlign;
import frc.robot.lib.util.AlignmentUtil;
public class AlignToRightBranchLinear extends LinearAlign {
    public AlignToRightBranchLinear() {
        super(
            () -> AlignmentUtil.getClosestRightBranchPose(),
            () -> new ChassisSpeeds(),
            2
        );
    }
}
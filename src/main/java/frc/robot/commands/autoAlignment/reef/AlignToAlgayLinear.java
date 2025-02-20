package frc.robot.commands.autoAlignment.reef;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.commands.autoAlignment.LinearAlign;
import frc.robot.lib.util.AlignmentUtil;
public class AlignToAlgayLinear extends LinearAlign {
    public AlignToAlgayLinear() {
        super(
            () -> AlignmentUtil.getClosestAlgayPose(),
            () -> new ChassisSpeeds(),
            2
        );
    }
}
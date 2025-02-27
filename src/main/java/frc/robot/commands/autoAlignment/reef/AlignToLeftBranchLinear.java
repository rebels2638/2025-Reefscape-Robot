package frc.robot.commands.autoAlignment.reef;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.commands.autoAlignment.LinearAlign;
import frc.robot.lib.util.AlignmentUtil;
import frc.robot.subsystems.elevator.Elevator;
public class AlignToLeftBranchLinear extends LinearAlign {
    public AlignToLeftBranchLinear() {
        super(
            () -> AlignmentUtil.getClosestLeftBranchPose(),
            () -> new ChassisSpeeds(),
            2
        );
    }
}
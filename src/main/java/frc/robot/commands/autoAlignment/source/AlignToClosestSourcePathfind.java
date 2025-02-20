package frc.robot.commands.autoAlignment.source;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.autoAlignment.OverridePathplannerFeedbackXboxController;
import frc.robot.commands.autoAlignment.PathfindThenAlign;
import frc.robot.lib.input.XboxController;
import frc.robot.lib.util.AlignmentUtil;
public class AlignToClosestSourcePathfind extends ParallelDeadlineGroup {
    public AlignToClosestSourcePathfind(XboxController controller) {
        super(
            new PathfindThenAlign(
                () -> AlignmentUtil.getClosestSourcePose(),
                () -> new ChassisSpeeds(),
                10
            ),
            new OverridePathplannerFeedbackXboxController(controller)
        );
    }
}
package frc.robot.commands.autoAlignment.reef.waitForCommands;

import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotState;
import frc.robot.lib.util.AlignmentUtil;

public class WaitForAlignToTopAuto extends WaitUntilCommand {
    public WaitForAlignToTopAuto() {
        super( // we wait for this ot be true to allow continual scheduling
            () -> AlignmentUtil.getClosestLeftBranchPose().getTranslation().getDistance( // check for the correct max distance from target
            RobotState.getInstance().getEstimatedPose().getTranslation()) <= 2
        );
    }
}
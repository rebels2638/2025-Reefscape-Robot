package frc.robot.commands.autoAlignment.reef.waitForCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotState;
import frc.robot.subsystems.roller.Roller;
import frc.robot.lib.util.AlignmentUtil;

public class WaitUntilAlignAuto extends WaitUntilCommand {
    public WaitUntilAlignAuto(Translation2d pathEndPose) {
        super( 
            () -> AlignmentUtil.getClosestLeftBranchPose().getTranslation().getDistance( // check for the correct max distance from target
            pathEndPose) <= 2 &&
            Roller.getInstance().inRoller()
        );
    }
}
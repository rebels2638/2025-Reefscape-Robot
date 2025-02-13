package frc.robot.commands.autoAlignment.reef;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotState;
import frc.robot.commands.autoAlignment.AlignmentUtil;
import frc.robot.commands.autoAlignment.LinearDriveToPose;
import frc.robot.constants.Constants.AlignmentConstants;
public class AlignToLeftBranch extends ConditionalCommand{
    public AlignToLeftBranch() {
        super(
            new SequentialCommandGroup ( // align
                new ConditionalCommand(
                    new LinearDriveToPose(() -> AlignmentUtil.offsetPoseToPreAlignment(AlignmentUtil.getClosestLeftBranchPose()), () -> new ChassisSpeeds()), // drive to a intermediate pose 
                    new InstantCommand(), // let the robot drive regularly
                    () -> Math.abs(RobotState.getInstance().getEstimatedPose().getRotation().minus(AlignmentUtil.getClosestLeftBranchPose().getRotation()).getDegrees()) > 45 // check if rotating will first increase the bumper profile while driving to the goal
                ),
                new LinearDriveToPose(() -> AlignmentUtil.getClosestLeftBranchPose(), () -> new ChassisSpeeds()) // finally drive to the target end goal
            ),
            new InstantCommand(), // if too far, do nothing
            () -> AlignmentUtil.getClosestLeftBranchPose().getTranslation().getDistance( // check for the correct max distance from target
                RobotState.getInstance().getEstimatedPose().getTranslation()) <= AlignmentConstants.kMAX_ALIGNMENT_DIST_METERS
        );
    }
}
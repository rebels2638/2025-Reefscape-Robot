package frc.robot.commands.autoAlignment.reef;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotState;
import frc.robot.commands.autoAlignment.LinearDriveToPose;
import frc.robot.constants.Constants.AlignmentConstants;
import frc.robot.lib.util.AlignmentUtil;
public class AlignToAlgay extends ConditionalCommand{
    public AlignToAlgay() {
        super(
            new SequentialCommandGroup ( // align
                new ConditionalCommand(
                    new LinearDriveToPose(() -> AlignmentUtil.offsetPoseToPreAlignment(AlignmentUtil.getClosestAlgayPose(0)), () -> new ChassisSpeeds()), // drive to a intermediate pose 
                    new InstantCommand(), // let the robot drive regularly
                    () -> Math.abs(RobotState.getInstance().getEstimatedPose().getRotation().minus(AlignmentUtil.getClosestAlgayPose(0).getRotation()).getDegrees()) > 45 // check if rotating will first increase the bumper profile while driving to the goal
                ),
                new LinearDriveToPose(() -> AlignmentUtil.getClosestAlgayPose(0), () -> new ChassisSpeeds()) // finally drive to the target end goal
            ),
            new InstantCommand(), // if too far, do nothing
            () -> AlignmentUtil.getClosestAlgayPose(0).getTranslation().getDistance( // check for the correct max distance from target
                RobotState.getInstance().getEstimatedPose().getTranslation()) <= AlignmentConstants.kMAX_ALIGNMENT_DIST_METERS
        );
    }
}
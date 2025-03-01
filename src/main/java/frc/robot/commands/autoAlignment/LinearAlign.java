package frc.robot.commands.autoAlignment;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotState;
import frc.robot.lib.util.AlignmentUtil;

public class LinearAlign extends ConditionalCommand {
    public LinearAlign(Supplier<Pose2d> goalPoseSupplier, Supplier<ChassisSpeeds> chassisSpeedsSupplier, double maxDistance) {
        super(
            new SequentialCommandGroup ( // align
                // drive to the closest algay pose
                new ConditionalCommand(
                    new LinearDriveToPose(() -> AlignmentUtil.offsetPoseToPreAlignment(goalPoseSupplier.get()), () -> chassisSpeedsSupplier.get()), // drive to a intermediate pose 
                    new InstantCommand(), // let the robot drive regularly
                    () -> Math.abs(RobotState.getInstance().getEstimatedPose().getRotation().minus(goalPoseSupplier.get().getRotation()).getDegrees()) > 45 // check if rotating will first increase the bumper profile while driving to the goal
                ),
                new LinearDriveToPose(() -> goalPoseSupplier.get(), () -> chassisSpeedsSupplier.get()) // finally drive to the target end goal
            ),
            new InstantCommand(), // if too far, do nothing
            () -> goalPoseSupplier.get().getTranslation().getDistance( // check for the correct max distance from target
                RobotState.getInstance().getEstimatedPose().getTranslation()) <= maxDistance
        );
    }
}
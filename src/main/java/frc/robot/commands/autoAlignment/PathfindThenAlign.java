package frc.robot.commands.autoAlignment;

import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotState;
import frc.robot.lib.util.AlignmentUtil;

public class PathfindThenAlign extends Command {
    private ConditionalCommand command;

    private final Supplier<Pose2d> goalPoseSupplier;
    private final Supplier<ChassisSpeeds> goalEndSpeeds;
    private final Supplier<Boolean> shouldReplan;
    private final double maxDistance;
    
    public PathfindThenAlign(Supplier<Pose2d> goalPoseSupplier, Supplier<ChassisSpeeds> goalEndSpeeds, Supplier<Boolean> shouldReplan, double maxDistance) {
        this.goalPoseSupplier = goalPoseSupplier;
        this.goalEndSpeeds = goalEndSpeeds;
        this.shouldReplan = shouldReplan;
        this.maxDistance = maxDistance;
    }

    // we do this in order to supply a new goal pose to the pathfinder every trigger pull
    @Override
    public void initialize() {
        command = new ConditionalCommand(
            new SequentialCommandGroup ( // align
                new PathfindToPose(
                    goalPoseSupplier,
                    goalEndSpeeds,
                    shouldReplan
                ), // drive to the closest algay pose
                new ConditionalCommand(
                    new LinearDriveToPose(() -> AlignmentUtil.offsetPoseToPreAlignment(goalPoseSupplier.get()), () -> goalEndSpeeds.get()), // drive to a intermediate pose 
                    new InstantCommand(), // let the robot drive regularly
                    () -> Math.abs(RobotState.getInstance().getEstimatedPose().getRotation().minus(goalPoseSupplier.get().getRotation()).getDegrees()) > 45 // check if rotating will first increase the bumper profile while driving to the goal
                ),
                new LinearDriveToPose(() -> goalPoseSupplier.get(), () -> goalEndSpeeds.get()) // finally drive to the target end goal
            ),
            new InstantCommand(), // if too far, do nothing
            () -> goalPoseSupplier.get().getTranslation().getDistance( // check for the correct max distance from target
                RobotState.getInstance().getEstimatedPose().getTranslation()) <= maxDistance
        );

        command.initialize();
    }

    @Override
    public void execute() {
        command.execute();
    }

    @Override
    public boolean isFinished() {
        return command.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        command.end(interrupted);
    }
}
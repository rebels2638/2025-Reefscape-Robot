package frc.robot.commands.autoAlignment;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AutoRunner;
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive;

public class PathfindToPose extends Command {
    private Command command;

    private final Supplier<Pose2d> goalPoseSupplier;
    private final Supplier<ChassisSpeeds> goalEndSpeeds;
    private final Supplier<Boolean> shouldReplan;
    
    public PathfindToPose(Supplier<Pose2d> goalPoseSupplier, Supplier<ChassisSpeeds> goalEndSpeeds, Supplier<Boolean> shouldReplan) {
        this.goalPoseSupplier = goalPoseSupplier;
        this.goalEndSpeeds = goalEndSpeeds;
        this.shouldReplan = shouldReplan;

        this.addRequirements(SwerveDrive.getInstance());
    }

    // we do this in order to supply a new goal pose to the pathfinder every trigger pull
    @Override
    public void initialize() {
        command = AutoBuilder.pathfindToPose(
            goalPoseSupplier.get(),
            AutoRunner.getInstance().getPathConstraints(),
            Math.hypot(goalEndSpeeds.get().vxMetersPerSecond, goalEndSpeeds.get().vyMetersPerSecond)
        );

        command.initialize();
    }

    @Override
    public void execute() {
        if (shouldReplan.get().booleanValue()) {
            command = AutoBuilder.pathfindToPose(
                goalPoseSupplier.get(),
                AutoRunner.getInstance().getPathConstraints(),
                Math.hypot(goalEndSpeeds.get().vxMetersPerSecond, goalEndSpeeds.get().vyMetersPerSecond)
            );

            command.initialize();
        }

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
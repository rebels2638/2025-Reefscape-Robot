package frc.robot.commands.autoAlignment;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AutoRunner;
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive;

/**
 * A wrapper class for PathPlanner pathfindToPose commands that ensures proper cancellation
 * of the internal command when the wrapper ends.
 */
public class PathPlannerPathfindToPoseWrapper extends Command {
    private Command internalCommand;
    private final Pose2d goalPose;
    private final PathConstraints constraints;
    private final double goalEndVel;

    /**
     * Creates a new wrapper for a PathPlanner pathfindToPose command.
     * 
     * @param goalPose The goal pose to pathfind to
     * @param constraints The path constraints to use
     * @param goalEndVel The goal end velocity in meters per second
     */
    public PathPlannerPathfindToPoseWrapper(Pose2d goalPose, PathConstraints constraints, double goalEndVel) {
        this.goalPose = goalPose;
        this.constraints = constraints;
        this.goalEndVel = goalEndVel;
        this.addRequirements(SwerveDrive.getInstance());
    }

    /**
     * Creates a new wrapper for a PathPlanner pathfindToPose command using default constraints.
     * 
     * @param goalPose The goal pose to pathfind to
     * @param goalEndVel The goal end velocity in meters per second
     */
    public PathPlannerPathfindToPoseWrapper(Pose2d goalPose, double goalEndVel) {
        this(goalPose, AutoRunner.getInstance().getPathConstraints(), goalEndVel);
    }

    /**
     * Creates a new wrapper for a PathPlanner pathfindToPose command using default constraints and zero end velocity.
     * 
     * @param goalPose The goal pose to pathfind to
     */
    public PathPlannerPathfindToPoseWrapper(Pose2d goalPose) {
        this(goalPose, 0.0);
    }

    @Override
    public void initialize() {
        internalCommand = AutoBuilder.pathfindToPose(goalPose, constraints, goalEndVel);
        internalCommand.initialize();
    }

    @Override
    public void execute() {
        if (internalCommand != null) {
            internalCommand.execute();
        }
    }

    @Override
    public boolean isFinished() {
        return internalCommand == null || internalCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        if (internalCommand != null) {
            // Explicitly cancel the internal command if interrupted
            if (interrupted) {
                internalCommand.cancel();
            }
            // Call end on the internal command
            internalCommand.end(interrupted);
        }
        
        // Clear any pathfinding state
        PPHolonomicDriveController.clearFeedbackOverrides();
        
        // Always stop the robot when the command ends
        SwerveDrive.getInstance().driveRobotRelative(new ChassisSpeeds());
    }
} 
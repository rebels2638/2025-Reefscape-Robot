package frc.robot.commands.autoAlignment;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive;

/**
 * A wrapper class for PathPlanner follow path commands that ensures proper cancellation
 * of the internal command when the wrapper ends.
 */
public class PathPlannerFollowPathWrapper extends Command {
    private Command internalCommand;
    private final String pathName;
    private final PathPlannerPath path;

    /**
     * Creates a new wrapper for a PathPlanner follow path command.
     * 
     * @param pathName The name of the path to follow
     */
    public PathPlannerFollowPathWrapper(String pathName) {
        this.pathName = pathName;
        this.path = loadPath(pathName);
        this.addRequirements(SwerveDrive.getInstance());
    }

    /**
     * Creates a new wrapper for a PathPlanner follow path command using a pre-loaded path.
     * 
     * @param path The pre-loaded PathPlannerPath to follow
     */
    public PathPlannerFollowPathWrapper(PathPlannerPath path) {
        this.pathName = "Pre-loaded path";
        this.path = path;
        this.addRequirements(SwerveDrive.getInstance());
    }

    /**
     * Loads a PathPlannerPath from a Choreo trajectory file.
     * 
     * @param name The name of the Choreo trajectory file (without extension)
     * @return The loaded PathPlannerPath, or null if loading failed
     */
    private PathPlannerPath loadPath(String name) {
        try {
            return PathPlannerPath.fromChoreoTrajectory(name);
        } catch (Exception e) {
            DriverStation.reportError("Failed to load path: " + name + " - " + e.getMessage(), e.getStackTrace());
            return null;
        }
    }

    @Override
    public void initialize() {
        if (path != null) {
            internalCommand = AutoBuilder.followPath(path);
            internalCommand.initialize();
        } else {
            // If path loading failed, create a command that does nothing
            internalCommand = null;
        }
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
        
        // Always stop the robot when the command ends
        SwerveDrive.getInstance().driveRobotRelative(new ChassisSpeeds());
    }
} 
package frc.robot.commands;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigComp;
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive;

// acts more like a helper class rather than a subsystem or command.
public class AutoRunner {


    private final SendableChooser<String> pathChooser = new SendableChooser<String>();
    private String pathChosen = "taxi";
    private static final HashMap<String, String> PATH_CHOSEN_TO_NAME_HASH_MAP = new HashMap<>();
    // private static PathPlannerAuto pathPlanner = new PathPlannerAuto();
    private static PPHolonomicDriveController scc  = new PPHolonomicDriveController(
        SwerveDrivetrainConfigComp.getInstance().getDrivePIDConfig(),
        SwerveDrivetrainConfigComp.getInstance().getSteerPIDConfig(),
        0.2
    );
    // private static HolonomicDriveController hdc = new HolonomicDriveController(
    //   new PIDController(0.1, 0, 0), new PIDController(.1, 0, 0), new ProfiledPIDController(0.1, 0, 0,new Constraints(Constants.Auton.MAX_SPEED, Constants.Auton.MAX_ACCELERATION)));
    private static PathPlannerPath path;


    static {
        PATH_CHOSEN_TO_NAME_HASH_MAP.put("3PMidTop", "3PMidTop");
        PATH_CHOSEN_TO_NAME_HASH_MAP.put("3PMidMA", "3PMidMA");
    }
    
    public AutoRunner () {

        PATH_CHOSEN_TO_NAME_HASH_MAP.forEach((pathName, pathFile) -> pathChooser.addOption(pathName, pathFile));

        Shuffleboard.getTab("Auto").add("Path Chooser", pathChooser);
        // Shuffleboard.getTab("Auto").add("Update Selected Command Output", 
        //     new InstantCommand( () -> loadPath()));
        
        AutoBuilder.configure(
                // visionSubsystem::getBotPose2d,
                RobotState.getInstance()::getEstimatedPose, // Robot pose supplier
                RobotState.getInstance()::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                RobotState.getInstance()::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> SwerveDrive.getInstance().driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                scc,
                SwerveDrivetrainConfigComp.getInstance().getRobotConfig(),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return true;
                },
                SwerveDrive.getInstance() // Reference to this subsystem to set requirements
        ); 
    }

    private void loadPath() {
        pathChosen = pathChooser.getSelected();
        Shuffleboard.getTab("Auto").add("Selected Path", pathChosen);
    }

        
    public Command getAutonomousCommand() {
        return new PathPlannerAuto("4PMidMASScoreTrans");
    }
} 
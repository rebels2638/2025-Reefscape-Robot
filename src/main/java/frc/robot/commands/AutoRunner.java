package frc.robot.commands;

import java.util.HashMap;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotState;
import frc.robot.constants.Constants;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigBase;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigComp;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigProto;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigSim;
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive;

// acts more like a helper class rather than a subsystem or command.
public class AutoRunner {
    private static AutoRunner instance = null;
    public static AutoRunner getInstance() {
        if (instance == null) {
            instance = new AutoRunner();
        }

        return instance;
    }

    private final SendableChooser<String> pathChooser = new SendableChooser<String>();
    private final HashMap<String, String> PATH_CHOSEN_TO_NAME_HASH_MAP = new HashMap<>();
    private String pathChosen = "taxi";

    private static PPHolonomicDriveController driveController;

    private final SwerveDrivetrainConfigBase drivetrainConfig;
    
    private AutoRunner () {
         switch (Constants.currentMode) {
            case COMP:
                drivetrainConfig = SwerveDrivetrainConfigComp.getInstance();

                break;

            case PROTO:
                drivetrainConfig = SwerveDrivetrainConfigProto.getInstance();

                break;

            case SIM:
                drivetrainConfig = SwerveDrivetrainConfigSim.getInstance();

                break;

            case REPLAY:
                drivetrainConfig = SwerveDrivetrainConfigComp.getInstance();

                break;

            default:
                drivetrainConfig = SwerveDrivetrainConfigComp.getInstance();

                break;
                
        }

        PATH_CHOSEN_TO_NAME_HASH_MAP.put("3PMidTop", "3PMidTop");
        PATH_CHOSEN_TO_NAME_HASH_MAP.put("3PMidMA", "3PMidMA");

        PATH_CHOSEN_TO_NAME_HASH_MAP.forEach((pathName, pathFile) -> pathChooser.addOption(pathName, pathFile));

        Shuffleboard.getTab("Auto").add("Path Chooser", pathChooser);
        Shuffleboard.getTab("Auto").add("Update Selected Command Output", new InstantCommand(() -> loadPath()));
        

        driveController = new PPHolonomicDriveController(
            drivetrainConfig.getPathplannerDrivePIDConfig(),
            drivetrainConfig.getPathplannerSteerPIDConfig(),
            Constants.kLOOP_CYCLE_MS
        );

        AutoBuilder.configure(
                RobotState.getInstance()::getEstimatedPose, // Robot pose supplier
                RobotState.getInstance()::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                RobotState.getInstance()::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> SwerveDrive.getInstance().driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                driveController,
                drivetrainConfig.getRobotConfig(),
                Constants::shouldFlipPath,
                SwerveDrive.getInstance() // Reference to this subsystem to set requirements
        ); 
    }

    private void loadPath() {
        pathChosen = pathChooser.getSelected();
        Shuffleboard.getTab("Auto").add("Selected Path", pathChosen);
    }

        
    public Command getAutonomousCommand() {
    //     try {
    //         // Load the path you want to follow using its name in the GUI
    //         PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory("test");

    //         // Create a path following command using AutoBuilder. This will also trigger event markers.
    //         return new SequentialCommandGroup(
    //             new InstantCommand(() -> RobotState.getInstance().resetPose(path.getStartingHolonomicPose().get())),
    //             AutoBuilder.followPath(path)
    //         );

    //     } catch (Exception e) {
    //         DriverStation.reportError("Oh shit, ur fucked haha " + e.getMessage(), e.getStackTrace());
    //         return Commands.none();
    //     }
  
        return Autos.test4;
    }

} 
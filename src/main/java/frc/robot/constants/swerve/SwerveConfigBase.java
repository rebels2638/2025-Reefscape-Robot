package frc.robot.constants.swerve;

import com.pathplanner.lib.config.RobotConfig;

public class SwerveConfigBase {
    protected SwerveModuleConfig.GeneralConfig kSHARED_GENERAL_CONFIG;

    protected SwerveModuleConfig kFRONT_LEFT_CONFIG;
    protected SwerveModuleConfig kFRONT_RIGHT_CONFIG;
    protected SwerveModuleConfig kBACK_LEFT_CONFIG;
    protected SwerveModuleConfig kBACK_RIGHT_CONFIG;

    protected SwerveDrivetrainConfig kSWERVE_DRIVETRAIN_CONFIG;

    protected SwerveControllerConfig kSWERVE_DRIVETRAIN_CONTROLLER_CONFIG;

    protected RobotConfig kPATHPLANNER_ROBOT_CONFIG;

    public SwerveModuleConfig.GeneralConfig getSharedGeneralConfig() {
        return kSHARED_GENERAL_CONFIG;
    }

    public SwerveModuleConfig getFrontLeftConfig() {
        return kFRONT_LEFT_CONFIG;
    }

    public SwerveModuleConfig getFrontRightConfig() {
        return kFRONT_RIGHT_CONFIG;
    }

    public SwerveModuleConfig getBackLeftConfig() {
        return kBACK_LEFT_CONFIG;
    }

    public SwerveModuleConfig getBackRightConfig() {
        return kBACK_RIGHT_CONFIG;
    }

    public SwerveDrivetrainConfig getSwerveDrivetrainConfig() {
        return kSWERVE_DRIVETRAIN_CONFIG;
    }

    public SwerveControllerConfig getSwerveDrivetrainControllerConfig() {
        return kSWERVE_DRIVETRAIN_CONTROLLER_CONFIG;
    }

    public RobotConfig getPathplannerRobotConfig() {
        return kPATHPLANNER_ROBOT_CONFIG;
    }
}

package frc.robot.constants.swerve;

public class SwerveConfigBase {
    protected SwerveModuleConfig.GeneralConfig kSHARED_GENERAL_CONFIG;

    protected SwerveModuleConfig kFRONT_LEFT_CONFIG;
    protected SwerveModuleConfig kFRONT_RIGHT_CONFIG;
    protected SwerveModuleConfig kBACK_LEFT_CONFIG;
    protected SwerveModuleConfig kBACK_RIGHT_CONFIG;

    protected SwerveDrivetrainConfig K_SWERVE_DRIVETRAIN_CONFIG;

    protected SwerveControllerConfig K_SWERVE_DRIVETRAIN_CONTROLLER_CONFIG;

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
        return K_SWERVE_DRIVETRAIN_CONFIG;
    }

    public SwerveControllerConfig getSwerveDrivetrainControllerConfig() {
        return K_SWERVE_DRIVETRAIN_CONTROLLER_CONFIG;
    }
}

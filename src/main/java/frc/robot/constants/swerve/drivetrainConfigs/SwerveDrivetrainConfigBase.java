package frc.robot.constants.swerve.drivetrainConfigs;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;

public abstract class SwerveDrivetrainConfigBase {

    public abstract double getMaxTranslationalVelocityMetersPerSec();

    public abstract double getMaxTranslationalAccelerationMetersPerSecSec();

    public abstract double getMaxAngularVelocityRadiansPerSec();

    public abstract double getMaxAngularAccelerationRadiansPerSecSec();

    public abstract double getMaxModuleVelocity();

    public abstract Translation2d getFrontLeftPositionMeters();

    public abstract Translation2d getFrontRightPositionMeters();

    public abstract Translation2d getBackLeftPositionMeters();

    public abstract Translation2d getBackRightPositionMeters();

    public abstract RobotConfig getRobotConfig();

    public abstract PIDConstants getPathplannerSteerPIDConfig();

    public abstract PIDConstants getPathplannerDrivePIDConfig();

    public abstract double getRotationCompensationCoefficient();

    public abstract PIDController getAutoAlignProfiledTranslationController();
    public abstract PIDController getAutoAlignProfiledRotationController();

    public abstract double getAutoAlignTranslationTolerance();
    public abstract double getAutoAlignTranslationVeloTolerance();

    public abstract double getAutoAlignRotationTolerance();
    public abstract double getAutoAlignRotationVeloTolerance();

    public abstract double getBumperLengthMeters();
        
    public abstract Translation2d getBranchOffsetFromRobotCenter();
    public abstract Translation2d getAlgayOffsetFromRobotCenter();

    public abstract double getMaxAligmentTranslationVeloMetersPerSec();
    public abstract double getMaxAligmentRotationVeloRadPerSec();

    public abstract double getMaxAligmentTranslationalAcelMetersPerSecPerSec();
    public abstract double getMaxAligmentRotationAcelRadPerSecPerSec();
}

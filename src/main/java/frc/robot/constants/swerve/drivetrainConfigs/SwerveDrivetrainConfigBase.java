package frc.robot.constants.swerve.drivetrainConfigs;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;

public abstract class SwerveDrivetrainConfigBase {

    public abstract double getMaxDrivetrainTranslationalVelocityMetersPerSec();

    public abstract double getMaxDrivetrainTranslationalAccelerationMetersPerSecSec();

    public abstract double getMaxDrivetrainAngularVelocityRadiansPerSec();

    public abstract double getMaxDrivetrainAngularAccelerationRadiansPerSecSec();

    public abstract Translation2d getFrontLeftPositionMeters();

    public abstract Translation2d getFrontRightPositionMeters();

    public abstract Translation2d getBackLeftPositionMeters();

    public abstract Translation2d getBackRightPositionMeters();

    public abstract double getRotationCompensationCoefficient();

    public abstract PIDController getAutoAlignProfiledTranslationController();
    public abstract PIDController getAutoAlignProfiledRotationController();

}

package frc.robot.constants.swerve.moduleConfigs;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public abstract class SwerveModuleGeneralConfigBase {
    public abstract String getCanBusName();

    public abstract double getDriveSupplyCurrentLimit();
    public abstract double getDriveSupplyCurrentLimitLowerTime();
    public abstract double getDriveSupplyCurrentLimitLowerLimit();

    public abstract double getDriveStatorCurrentLimit();

    public abstract double getDrivePeakForwardTorqueCurrent();
    public abstract double getDrivePeakReverseTorqueCurrent();

    public abstract double getDriveKS();
    public abstract double getDriveKV();
    public abstract double getDriveKA();
    public abstract double getDriveKP();
    public abstract double getDriveKI();
    public abstract double getDriveKD();

    public abstract double getDriveMaxVelocityMetersPerSec();

    public abstract double getDriveMotionMagicVelocityAccelerationMetersPerSecSec();
    public abstract double getDriveMotionMagicVelocityDecelerationMetersPerSecSec();
    public abstract double getDriveMotionMagicVelocityJerkMetersPerSecSecSec();

    public abstract boolean getIsDriveNeutralModeBrake();

    public abstract double getDriveMotorToOutputShaftRatio();
    public abstract double getDriveWheelRadiusMeters();

    public abstract double getSteerSupplyCurrentLimit();
    public abstract double getSteerSupplyCurrentLimitLowerTime();
    public abstract double getSteerSupplyCurrentLimitLowerLimit();

    public abstract double getSteerStatorCurrentLimit();

    public abstract double getSteerPeakForwardTorqueCurrent();
    public abstract double getSteerPeakReverseTorqueCurrent();

    public abstract double getSteerKS();
    public abstract double getSteerKV();
    public abstract double getSteerKA();
    public abstract double getSteerKP();
    public abstract double getSteerKI();
    public abstract double getSteerKD();

    public abstract double getSteerMotionMagicExpoKA();
    public abstract double getSteerMotionMagicExpoKV();
    public abstract double getSteerMotionMagicCruiseVelocityRotationsPerSec();

    public abstract boolean getIsSteerNeutralModeBrake();

    public abstract double getSteerMotorToOutputShaftRatio();
    public abstract double getSteerRotorToSensorRatio();

    public abstract FeedbackSensorSourceValue getSteerCancoderFeedbackSensorSource();

    public abstract SensorDirectionValue getCancoderSensorDirection();
    public abstract double getCancoderAbsoluteSensorDiscontinuityPoint();

    public abstract double getDriveMinWallCurrent();
    public abstract double getDriveMaxWallVeloMetersPerSec();
}
package frc.robot.constants.swerve.moduleConfigs.proto;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.constants.swerve.moduleConfigs.SwerveModuleGeneralConfigBase;

public class SwerveModuleGeneralConfigProto extends SwerveModuleGeneralConfigBase {

    public static SwerveModuleGeneralConfigProto instance = null;
    public static SwerveModuleGeneralConfigProto getInstance() {
        if (instance == null) {
            instance = new SwerveModuleGeneralConfigProto();
        }
        return instance;
    }

    private SwerveModuleGeneralConfigProto() {}

    @Override
    public String getCanBusName() {
        return "rio";
    }

    @Override
    public double getDriveSupplyCurrentLimit() {
        return 60.0;
    }

    @Override
    public double getDriveSupplyCurrentLimitLowerTime() {
        return 0.5;
    }

    @Override
    public double getDriveSupplyCurrentLimitLowerLimit() {
        return 20.0;
    }

    @Override
    public double getDriveStatorCurrentLimit() {
        return 60.0;
    }

    @Override
    public double getDrivePeakForwardTorqueCurrent() {
        return 60.0;
    }

    @Override
    public double getDrivePeakReverseTorqueCurrent() {
        return -60.0;
    }

    @Override
    public double getDriveKS() {
        return 2.5;
    }

    @Override
    public double getDriveKV() {
        return 0.2;
    }

    @Override
    public double getDriveKA() {
        return 4.2;
    }

    @Override
    public double getDriveKP() {
        return 15;
    }

    @Override
    public double getDriveKI() {
        return 0.0;
    }

    @Override
    public double getDriveKD() {
        return 3;
    }

    @Override
    public double getDriveMotionMagicVelocityAccelerationMetersPerSecSec() {
        return 7;
    }

    @Override
    public double getDriveMotionMagicVelocityDecelerationMetersPerSecSec() {
        return 14;
    }

    @Override
    public double getDriveMotionMagicVelocityJerkMetersPerSecSecSec() {
        return 40.0;
    }

    @Override
    public double getDriveMaxVelocityMetersPerSec() {
        return 4;
    }

    @Override
    public boolean getIsDriveNeutralModeBrake() {
        return true;
    }

    @Override
    public double getDriveMotorToOutputShaftRatio() {
        return 6.12;
    }

    @Override
    public double getDriveWheelRadiusMeters() {
        return 0.04861;
    }

    @Override
    public double getSteerSupplyCurrentLimit() {
        return 30.0;
    }

    @Override
    public double getSteerSupplyCurrentLimitLowerTime() {
        return 1.5;
    }

    @Override
    public double getSteerSupplyCurrentLimitLowerLimit() {
        return 30.0;
    }

    @Override
    public double getSteerStatorCurrentLimit() {
        return 40.0;
    }

    @Override
    public double getSteerPeakForwardTorqueCurrent() {
        return 40.0;
    }

    @Override
    public double getSteerPeakReverseTorqueCurrent() {
        return -40.0;
    }

    @Override
    public double getSteerKS() {
        return 1.5;
    }

    @Override
    public double getSteerKV() {
        return 1.17;
    }

    @Override
    public double getSteerKA() {
        return 0.50;
    }

    @Override
    public double getSteerKP() {
        return 350;
    }

    @Override
    public double getSteerKI() {
        return 0.0;
    }

    @Override
    public double getSteerKD() {
        return 10;
    }

    @Override
    public double getSteerMotionMagicExpoKA() {
        return 0.25;
    }

    @Override
    public double getSteerMotionMagicExpoKV() {
        return 2.6;
    }

    @Override
    public double getSteerMotionMagicCruiseVelocityRotationsPerSec() {
        return 6;
    }

    @Override
    public boolean getIsSteerNeutralModeBrake() {
        return true;
    }

    @Override
    public double getSteerMotorToOutputShaftRatio() {
        return 21.428;
    }

    @Override
    public double getSteerRotorToSensorRatio() {
        return 21.428;
    }

    @Override
    public FeedbackSensorSourceValue getSteerCancoderFeedbackSensorSource() {
        return FeedbackSensorSourceValue.FusedCANcoder;
    }

    @Override
    public SensorDirectionValue getCancoderSensorDirection() {
        return SensorDirectionValue.CounterClockwise_Positive;
    }

    @Override
    public double getCancoderAbsoluteSensorDiscontinuityPoint() {
        return 1;
    }

    @Override
    public double getDriveMinWallCurrent() {
        return 0;
    }

    @Override
    public double getDriveMaxWallVeloMetersPerSec() {
        return 0.02;
    }
}
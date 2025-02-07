package frc.robot.constants.swerve.moduleConfigs.proto;

import frc.robot.constants.swerve.moduleConfigs.SwerveModuleSpecificConfigBase;

public class SwerveModuleSpecificFLConfigProto extends SwerveModuleSpecificConfigBase {

    public static SwerveModuleSpecificFLConfigProto instance = null;
    public static SwerveModuleSpecificFLConfigProto getInstance() {
        if (instance == null) {
            instance = new SwerveModuleSpecificFLConfigProto();
        }
        return instance;
    }

    private SwerveModuleSpecificFLConfigProto() {}

    @Override
    public int getDriveCanId() {
        return 4;
    }

    @Override
    public int getSteerCanId() {
        return 5;
    }

    @Override
    public boolean getIsDriveInverted() {
        return true;
    }

    @Override
    public boolean getIsSteerInverted() {
        return true;
    }

    @Override
    public boolean isDriveNeutralModeBrake() {
        return true;
    }

    @Override
    public boolean isSteerNeutralModeBrake() {
        return true;
    }

    @Override
    public int getCancoderCanId() {
        return 10;
    }

    @Override
    public double getCancoderOffsetRotations() {
        return -0.86;
    }
}
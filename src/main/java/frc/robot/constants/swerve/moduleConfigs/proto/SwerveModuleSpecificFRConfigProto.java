package frc.robot.constants.swerve.moduleConfigs.proto;

import frc.robot.constants.swerve.moduleConfigs.SwerveModuleSpecificConfigBase;

public class SwerveModuleSpecificFRConfigProto extends SwerveModuleSpecificConfigBase {

    public static SwerveModuleSpecificFRConfigProto instance = null;
    public static SwerveModuleSpecificFRConfigProto getInstance() {
        if (instance == null) {
            instance = new SwerveModuleSpecificFRConfigProto();
        }
        return instance;
    }

    private SwerveModuleSpecificFRConfigProto() {}
    
    @Override
    public int getDriveCanId() {
        return 2;
    }

    @Override
    public int getSteerCanId() {
        return 3;
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
        return 9;
    }

    @Override
    public double getCancoderOffsetRotations() {
        return -0.68;
    }
}
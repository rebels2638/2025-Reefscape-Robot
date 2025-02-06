package frc.robot.constants.swerve.moduleConfigs.proto;

import frc.robot.constants.swerve.moduleConfigs.SwerveModuleSpecificConfigBase;

public class SwerveModuleSpecificBLConfigProto extends SwerveModuleSpecificConfigBase {

    public static SwerveModuleSpecificBLConfigProto instance = null;
    public static SwerveModuleSpecificBLConfigProto getInstance() {
        if (instance == null) {
            instance = new SwerveModuleSpecificBLConfigProto();
        }
        return instance;
    }

    private SwerveModuleSpecificBLConfigProto() {}
    
    @Override
    public int getDriveCanId() {
        return 6;
    }

    @Override
    public int getSteerCanId() {
        return 7;
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
        return 11;
    }

    @Override
    public double getCancoderOffsetRotations() {
        return -0.78;
    }
}
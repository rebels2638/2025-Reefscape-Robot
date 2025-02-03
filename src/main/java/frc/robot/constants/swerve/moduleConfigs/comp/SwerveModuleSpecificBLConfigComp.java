package frc.robot.constants.swerve.moduleConfigs.comp;

import frc.robot.constants.swerve.moduleConfigs.SwerveModuleSpecificConfigBase;

public class SwerveModuleSpecificBLConfigComp extends SwerveModuleSpecificConfigBase {

    public static SwerveModuleSpecificBLConfigComp instance = null;
    public static SwerveModuleSpecificBLConfigComp getInstance() {
        if (instance == null) {
            instance = new SwerveModuleSpecificBLConfigComp();
        }
        return instance;
    }

    private SwerveModuleSpecificBLConfigComp() {}
    
    @Override
    public int getDriveCanId() {
        return 8;
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
        return 12;
    }

    @Override
    public double getCancoderOffsetRotations() {
        return 0.413330078125;
    }
}

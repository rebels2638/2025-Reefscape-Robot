package frc.robot.constants.swerve.moduleConfigs.comp;

import frc.robot.constants.swerve.moduleConfigs.SwerveModuleSpecificConfigBase;

public class SwerveModuleSpecificFRConfigComp extends SwerveModuleSpecificConfigBase {

    public static SwerveModuleSpecificFRConfigComp instance = null;
    public static SwerveModuleSpecificFRConfigComp getInstance() {
        if (instance == null) {
            instance = new SwerveModuleSpecificFRConfigComp();
        }
        return instance;
    }

    private SwerveModuleSpecificFRConfigComp() {}
    
    @Override
    public int getDriveCanId() {
        return 4;
    }

    @Override
    public int getSteerCanId() {
        return 3;
    }

    @Override
    public boolean getIsDriveInverted() {
        return false;
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
        return -0.750977;
    }
}

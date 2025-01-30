package frc.robot.constants.swerve.moduleConfigs.comp;

import frc.robot.constants.swerve.moduleConfigs.SwerveModuleSpecificConfigBase;

public class SwerveModuleSpecificBRConfigComp extends SwerveModuleSpecificConfigBase {

    public static SwerveModuleSpecificBRConfigComp instance = null;
    public static SwerveModuleSpecificBRConfigComp getInstance() {
        if (instance == null) {
            instance = new SwerveModuleSpecificBRConfigComp();
        }
        return instance;
    }

    @Override
    public int getDriveCanId() {
        return 1;
    }

    @Override
    public int getSteerCanId() {
        return 0;
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
        return 8;
    }

    @Override
    public double getCancoderOffsetRotations() {
        return -0.98;
    }
}

package frc.robot.constants.swerve.moduleConfigs;

public abstract class SwerveModuleSpecificConfigBase {
    public abstract int getDriveCanId();
    public abstract int getSteerCanId();

    public abstract boolean getIsDriveInverted();
    public abstract boolean getIsSteerInverted();

    public abstract boolean isDriveNeutralModeBrake();
    public abstract boolean isSteerNeutralModeBrake();
    
    public abstract int getCancoderCanId();
    public abstract double getCancoderOffsetRotations();
}
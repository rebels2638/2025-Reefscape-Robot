package frc.robot.constants.robotState;

public abstract class RobotStateConfigBase {
    public abstract double getOdomTranslationDevBase();
    public abstract double getVisionTranslationDevBase();

    public abstract double getMaxElevatorExtensionVelocityMeterPerSec();
    public abstract double getMaxElevatorExtensionAccelerationMetersPerSecPerSec();
    public abstract double getMaxRotationalVelocityRadPerSecPerSec();
}

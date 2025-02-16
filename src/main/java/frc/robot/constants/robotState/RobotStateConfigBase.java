package frc.robot.constants.robotState;

public abstract class RobotStateConfigBase {
    public abstract double getOdomTranslationDevBase();
    public abstract double getVisionTranslationDevBase();

    public abstract double getMaxElevatorExtensionVelocity();
    public abstract double getMaxElevatorExtensionAcceleration();
    public abstract double getMinElevatorExtensionAcceleration();
}

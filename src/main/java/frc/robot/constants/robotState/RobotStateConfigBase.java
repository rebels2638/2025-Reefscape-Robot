package frc.robot.constants.robotState;

import edu.wpi.first.math.geometry.Translation2d;

public abstract class RobotStateConfigBase {
    public abstract double getOdomTranslationDevBase();
    public abstract double getVisionTranslationDevBase();

    public abstract double getMaxElevatorExtensionVelocity();
    public abstract double getMaxElevatorExtensionAcceleration();
    public abstract double getMinElevatorExtensionAcceleration();
}

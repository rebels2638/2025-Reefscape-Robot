package frc.robot.constants.swerve.controllerConfigs;

import edu.wpi.first.math.controller.PIDController;

public abstract class SwerveControllerConfigBase {

    public abstract PIDController getRotationalVelocityFeedbackController();

    public abstract PIDController getTranslationVelocityFeedbackController();

    public abstract PIDController getRotationalPositionFeedbackController();

    public abstract double getRotationalPositionMaxOutputRadSec();
}
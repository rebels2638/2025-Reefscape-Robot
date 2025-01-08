package frc.robot.constants.swerve;

import java.util.ArrayList;

import edu.wpi.first.math.controller.PIDController;

public class SwerveControllerConfig {
    // drive meters per second, radians per second, mps error
    public final ArrayList<double[]> kDRIVE_FF_POINTS;
    public final PIDController kROTATIONAL_VELOCITY_FEEDBACK_CONTROLLER;
    public final PIDController kTRANSLATION_VELOCITY_FEEDBACK_CONTROLLER;
    public final PIDController kROTATIONAL_POSITION_FEEDBACK_CONTROLLER;

    public SwerveControllerConfig(
        ArrayList<double[]> driveFFPoints,
        PIDController rotationalVelocityFeedbackController,
        PIDController translationalVelocityFeedbackController,
        PIDController rotationalPositionFeedbackController) {
        this.kDRIVE_FF_POINTS = driveFFPoints;
        this.kROTATIONAL_VELOCITY_FEEDBACK_CONTROLLER = rotationalVelocityFeedbackController;
        this.kTRANSLATION_VELOCITY_FEEDBACK_CONTROLLER = translationalVelocityFeedbackController;
        this.kROTATIONAL_POSITION_FEEDBACK_CONTROLLER = rotationalPositionFeedbackController;
    }
}

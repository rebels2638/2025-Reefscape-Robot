package frc.robot.constants.swerve;

import java.util.ArrayList;

import edu.wpi.first.math.controller.PIDController;

public class SwerveControllerConfig {
    // drive meters per second, radians per second, mps error
    public final ArrayList<double[]> kDRIVE_FF_POINTS;
    public final PIDController kROTATIONAL_VELOCITY_FEEDBACK_CONTROLLER;
    public final PIDController kTRANSLATION_VELOCITY_FEEDBACK_CONTROLLER;
    public final PIDController kLOCK_ROTATIONAL_POSITION_FEEDBACK_CONTROLLER;

    public final PIDController kAUTO_ALIGN_ROTATIONAL_POSITION_FEEDBACK_CONTROLLER;
    public final PIDController kAUTO_ALIGN_TRANSLATIONAL_POSITION_FEEDBACK_CONTROLLER;
    public SwerveControllerConfig(
        ArrayList<double[]> driveFFPoints,

        PIDController rotationalVelocityFeedbackController,
        double rotationalVelocityFeedbackControllerTolerance,

        PIDController translationalVelocityFeedbackController,
        double translationalVelocityFeedbackControllerTolerance,

        PIDController lockRotationalPositionFeedbackController,
        double lockRotationalPositionFeedbackControllerTolerance,

        PIDController autoAlignRotationalPositionFeedbackController,
        double autoAlignRotationalPositionFeedbackControllerTolerance,

        PIDController autoAlignTranslationalPositionFeedbackController,
        double autoAlignTranslationalPositionFeedbackControllerTolerance
    ) {
        this.kDRIVE_FF_POINTS = driveFFPoints;

        this.kROTATIONAL_VELOCITY_FEEDBACK_CONTROLLER = rotationalVelocityFeedbackController;
        this.kROTATIONAL_VELOCITY_FEEDBACK_CONTROLLER.setTolerance(rotationalVelocityFeedbackControllerTolerance);

        this.kTRANSLATION_VELOCITY_FEEDBACK_CONTROLLER = translationalVelocityFeedbackController;
        this.kTRANSLATION_VELOCITY_FEEDBACK_CONTROLLER.setTolerance(translationalVelocityFeedbackControllerTolerance);

        this.kLOCK_ROTATIONAL_POSITION_FEEDBACK_CONTROLLER = lockRotationalPositionFeedbackController;
        this.kLOCK_ROTATIONAL_POSITION_FEEDBACK_CONTROLLER.setTolerance(lockRotationalPositionFeedbackControllerTolerance);

        this.kAUTO_ALIGN_ROTATIONAL_POSITION_FEEDBACK_CONTROLLER = autoAlignRotationalPositionFeedbackController;
        this.kAUTO_ALIGN_ROTATIONAL_POSITION_FEEDBACK_CONTROLLER.setTolerance(autoAlignRotationalPositionFeedbackControllerTolerance);

        this.kAUTO_ALIGN_TRANSLATIONAL_POSITION_FEEDBACK_CONTROLLER = autoAlignTranslationalPositionFeedbackController;
        this.kAUTO_ALIGN_TRANSLATIONAL_POSITION_FEEDBACK_CONTROLLER.setTolerance(autoAlignTranslationalPositionFeedbackControllerTolerance);
    }
}

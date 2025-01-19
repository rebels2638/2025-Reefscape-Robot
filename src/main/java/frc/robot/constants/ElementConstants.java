package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

public class ElementConstants {
    // stupid shenanigans work on this bummy class later
    public static final Pose2d poseReefTop  = new Pose2d(6.2, 4, new Rotation2d(Math.toRadians(-180)));
    public static final Pose2d poseReefTopLeft  = new Pose2d(5.36, 5.48, new Rotation2d(Math.toRadians(-120)));
    public static final Pose2d poseReefTopRight  = new Pose2d(5.36, 2.55, new Rotation2d(Math.toRadians(120)));
    public static final Pose2d poseReefBottom  = new Pose2d(2.78, 4.22, new Rotation2d(Math.toRadians(0)));
    public static final Pose2d poseReefBottomLeft = new Pose2d(3.68, 5.51, new Rotation2d(Math.toRadians(-60)));
    public static final Pose2d poseReefBottomRight  = new Pose2d(3.68, 2.5, new Rotation2d(Math.toRadians(60)));

    public static final Pose2d poseSourceTop = new Pose2d(1.3, 7, new Rotation2d(Math.toRadians(-54)));
    public static final Pose2d poseSourceBottom = new Pose2d(1.3, 1, new Rotation2d(Math.toRadians(56)));

    public static final Pose2d redProcessor  = new Pose2d(5.97, .66, new Rotation2d(Math.toRadians(-90)));
    public static final Pose2d blueProcessor  = new Pose2d(11.55, 7.46, new Rotation2d(Math.toRadians(90)));

    public static final Pose2d kPOSE_ARR[] = new Pose2d[] {
        poseReefTop,
        poseReefTopLeft,
        poseReefTopRight,
        poseReefBottom,
        poseReefBottomLeft,
        poseReefBottomRight,
        poseSourceTop,
        poseSourceBottom,
        blueProcessor,
        redProcessor
    };

    public static final Pose3d kPOSE_ALGAE[] = new Pose3d[] {
        new Pose3d(),
        new Pose3d(),
        new Pose3d(),
        new Pose3d(),
        new Pose3d(),
        new Pose3d()
    };
}

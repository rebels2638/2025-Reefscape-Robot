package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

public class ElementConstants {
    // stupid shenanigans work on this bummy class later
    public static final Pose2d poseReefTop  = new Pose2d(5.85, 4.04, new Rotation2d(Math.toRadians(-180)));
    public static final Pose2d poseReefTopLeft  = new Pose2d(5.18, 5.2, new Rotation2d(Math.toRadians(-120)));
    public static final Pose2d poseReefTopRight  = new Pose2d(5.17, 2.88, new Rotation2d(Math.toRadians(120)));
    public static final Pose2d poseReefBottom  = new Pose2d(3.11, 4.04, new Rotation2d(Math.toRadians(0)));
    public static final Pose2d poseReefBottomLeft = new Pose2d(3.83, 5.2, new Rotation2d(Math.toRadians(-60)));
    public static final Pose2d poseReefBottomRight  = new Pose2d(3.83, 2.88, new Rotation2d(Math.toRadians(60)));

    public static final Pose2d poseSourceTop = new Pose2d(1.2, 7.05, new Rotation2d(Math.toRadians(-54)));
    public static final Pose2d poseSourceBottom = new Pose2d(1.2, 1.01, new Rotation2d(Math.toRadians(56)));

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

package frc.robot.lib.util;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotState;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.AlignmentConstants;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigBase;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigComp;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigProto;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigSim;

public class AlignmentUtil {
    public static class Axis {
        public final Double x1, y1, x2, y2;
        public final Double m, b;
        public Axis(Double x1, Double y1, Double x2, Double y2) {
            this.x1 = x1;
            this.y1 = y1;
            this.x2 = x2;
            this.y2 = y2;

            if (x2.equals(x1)) {
                this.m = Double.POSITIVE_INFINITY;
                this.b = x1;
            }
            else {
                this.m = (y2 - y1) / (x2 - x1);
                this.b = y1 - m * x1;
            }
        }

        public Axis(Double m, Double b) {
            this.x1 = Double.NaN;
            this.y1 = Double.NaN;
            this.x2 = Double.NaN;
            this.y2 = Double.NaN;

            this.m = m;
            this.b = b;
        }

        // will constrain between x1 and x2
        public Double getOutput(double input) {
            if (!Double.isNaN(x1) && !Double.isNaN(x2)) {
                input = RebelUtil.constrain(input, Math.min(x1, x2), Math.max(x1, x2));
            }
            if (m.isInfinite()) {
                return Double.POSITIVE_INFINITY;
            }
            return m * input + b;
        }

        public Axis getPerpendicularAxis(Double x3, Double y3) {
            Double m2;
            Double b2;
            if (this.m.isInfinite()) {
                m2 = 0.0;
                b2  = y3;
            }
            else {
                m2 = -1/this.m;
                b2 = y3 - m2 * x3;
            }

            return new Axis(
                m2,
                b2
            );
        }

        public Translation2d getIntersection(Axis other) {
            if (this.m.equals(other.m) || this.m.isInfinite() && other.m.isInfinite()) {
                return null;
            }

            Double x;
            Double y;
            if (this.m.isInfinite()) {
               x = this.b;
               y = other.getOutput(x);
            }
            else if (other.m.isInfinite()) {
                x = other.b;
                y = this.getOutput(x);
            }
            else {
                x = (this.b - other.b) / (other.m - this.m);
                y = getOutput(x);    
            }

            return new Translation2d(x, y);
        }

        public Translation2d getPointOnAxis(Translation2d p) {
            Double x3 = p.getX();
            Double y3 = p.getY();
            
            Axis perpendicularAxis = getPerpendicularAxis(x3, y3);
            Translation2d intersection = getIntersection(perpendicularAxis);
            if (intersection == null) {
                intersection = p;
            }

            if (!Double.isNaN(x1) && !Double.isNaN(x2) &&
                !Double.isNaN(y1) && !Double.isNaN(y2)) {
                return new Translation2d(
                    RebelUtil.constrain(intersection.getX(), Math.min(x1, x2), Math.max(x1, x2)),
                    RebelUtil.constrain(intersection.getY(), Math.min(y1, y2), Math.max(y1, y2))
                );
            }
            else {
                return intersection;
            }
        }

        public Rotation2d getAngle() {
            if (m.isInfinite()) {
                return new Rotation2d(Math.PI/2);
            }
            return new Rotation2d(Math.atan(m.doubleValue()));
        }
    }

    private static final SwerveDrivetrainConfigBase config;

    private static List<Pose2d> leftBranchCandidates = new ArrayList<>();
    private static List<Pose2d> rightBranchCandidates = new ArrayList<>();
    private static List<Pose2d> algayCandidates = new ArrayList<>();
    private static List<Pose2d> sourceCandidates = new ArrayList<>();
    private static List<Pose2d> cageCandidates = new ArrayList<>();

    private static Axis rightSourceAxis = new Axis(0.0, 0.0);
    private static Axis leftSourceAxis = new Axis(0.0, 0.0);
    private static Axis bargeAxis = new Axis(0.0, 0.0);

    static {
        switch (Constants.currentMode) {
            case COMP:
                config = SwerveDrivetrainConfigComp.getInstance();

                break;

            case PROTO:
                config = SwerveDrivetrainConfigProto.getInstance();

                break;

            case SIM:
                config = SwerveDrivetrainConfigSim.getInstance();

                break;

            case REPLAY:
                config = SwerveDrivetrainConfigComp.getInstance();

                break;

            default:
                config = SwerveDrivetrainConfigComp.getInstance();

                break;
        }

        loadCandidates();
    }

    public static void loadCandidates() { // this has to be called on telop / auto init in order to ensure ds is connected
        leftBranchCandidates = new ArrayList<>();
        rightBranchCandidates = new ArrayList<>();
        algayCandidates = new ArrayList<>();
        sourceCandidates = new ArrayList<>();
        cageCandidates = new ArrayList<>();

        if (Constants.shouldFlipPath()) {
            for (int i = 0; i < 6; i++) {
                leftBranchCandidates.add(FlippingUtil
                        .flipFieldPose(offsetBranchPose(AlignmentConstants.kREEF_CENTER_FACES[i], i % 2 == 0)));
            }
            for (int i = 0; i < 6; i++) {
                rightBranchCandidates.add(FlippingUtil
                        .flipFieldPose(offsetBranchPose(AlignmentConstants.kREEF_CENTER_FACES[i], i % 2 != 0)));
            }

            double bumperOffset = config.getBumperLengthMeters() / 2;

            for (Pose2d element : AlignmentConstants.kREEF_CENTER_FACES) {
                algayCandidates.add(
                        FlippingUtil.flipFieldPose(
                                element.transformBy(
                                        new Transform2d(
                                                config.getAlgayOffsetFromRobotCenter().getX() - bumperOffset,
                                                config.getAlgayOffsetFromRobotCenter().getY(),
                                                new Rotation2d(0)))));
            }

            for (Pose2d element : AlignmentConstants.kSOURCE_CENTER_FACES) {
                sourceCandidates.add(
                    FlippingUtil.flipFieldPose(
                        element.transformBy(
                            new Transform2d(
                                -bumperOffset,
                                0,
                                new Rotation2d(0)
                            )
                        )
                    )
                );
            }

            for (int i = 3; i < 6; i++) {
                cageCandidates.add(
                    AlignmentConstants.kCAGE_CENTERS[i].transformBy(
                        new Transform2d(
                            -bumperOffset,
                            0,
                            new Rotation2d(0)
                        )
                    )
                );
            }

            rightSourceAxis = flipAxis(offsetAxis(AlignmentConstants.kRIGHT_SOURCE_AXIS, false));
            leftSourceAxis = flipAxis(offsetAxis(AlignmentConstants.kLEFT_SOURCE_AXIS, true));
            bargeAxis = flipAxis(offsetAxis(AlignmentConstants.kBARGE_AXIS, false)); // TODO: correctness?
        } 

        else {
            for (int i = 0; i < 6; i++) {
                leftBranchCandidates.add(offsetBranchPose(AlignmentConstants.kREEF_CENTER_FACES[i], i % 2 == 0));
            }
            for (int i = 0; i < 6; i++) {
                rightBranchCandidates.add(offsetBranchPose(AlignmentConstants.kREEF_CENTER_FACES[i], i % 2 != 0));
            }

            double bumperOffset = config.getBumperLengthMeters() / 2;

            for (Pose2d element : AlignmentConstants.kREEF_CENTER_FACES) {
                algayCandidates.add(
                        element.transformBy(
                                new Transform2d(
                                        config.getAlgayOffsetFromRobotCenter().getX() - bumperOffset,
                                        config.getAlgayOffsetFromRobotCenter().getY(),
                                        new Rotation2d(0))));
            }

            for (Pose2d element : AlignmentConstants.kSOURCE_CENTER_FACES) {
                sourceCandidates.add(
                    element.transformBy(
                        new Transform2d(
                            bumperOffset,
                            0,
                            new Rotation2d(0)
                        )
                    )
                );
            }

            for (int i = 0; i < 3; i++) {
                cageCandidates.add(
                    AlignmentConstants.kCAGE_CENTERS[i].transformBy(
                        new Transform2d(
                            bumperOffset,
                            0,
                            new Rotation2d(0)
                        )
                    )
                );
            }

            rightSourceAxis = offsetAxis(AlignmentConstants.kRIGHT_SOURCE_AXIS, false);
            leftSourceAxis = offsetAxis(AlignmentConstants.kLEFT_SOURCE_AXIS, true);
            bargeAxis = offsetAxis(AlignmentConstants.kBARGE_AXIS, false);
        }

        for (int i = 0; i < leftBranchCandidates.size(); i++) {
            Logger.recordOutput("AligmentUtil/leftBranchCandidates" + i, leftBranchCandidates.get(i));
        }
    }

    public static Pose2d offsetCoralPoseToPreAlignment(Pose2d pose) {
        return offsetPoseToPreAlignment(pose, config.getBumperLengthMeters() / 2 * Math.sqrt(2));
    }

    public static Pose2d offsetCoralPoseToVisionReading(Pose2d pose) {
        return offsetPoseToPreAlignment(pose, 0.17);
    }

    public static Pose2d offsetPoseToPreAlignment(Pose2d pose, double distance) {
        return pose.transformBy(
                new Transform2d(
                        -distance, // this is the bumper radius
                        0,
                        new Rotation2d(0)));
    }

    public static Pose2d offsetBranchPose(Pose2d pose, boolean isLeftBranch) { // DS relative
        double bumperOffset = config.getBumperLengthMeters() / 2;
        double invert = isLeftBranch ? 1 : -1;

        return pose.transformBy(
                new Transform2d(
                        -bumperOffset + config.getBranchOffsetFromRobotCenter().getX(),
                        invert * (AlignmentConstants.kINTER_BRANCH_DIST_METER / 2)
                                + config.getBranchOffsetFromRobotCenter().getY(),
                        new Rotation2d(0)));
    }

    public static Pose2d getClosestReefFaceSimple(Pose2d curr) {
        curr = Constants.shouldFlipPath() ? FlippingUtil.flipFieldPose(curr) : curr;
        
        int nearest = 0;
        for (int i = 0; i < AlignmentConstants.kREEF_CENTER_FACES.length; i++) {
            if (AlignmentConstants.kREEF_CENTER_FACES[i].getTranslation()
                    .getDistance(curr.getTranslation()) < AlignmentConstants.kREEF_CENTER_FACES[nearest]
                            .getTranslation().getDistance(curr.getTranslation())) {
                nearest = i;
            }
        }
        return Constants.shouldFlipPath() ? 
            FlippingUtil.flipFieldPose(AlignmentConstants.kREEF_CENTER_FACES[nearest]) : 
            AlignmentConstants.kREEF_CENTER_FACES[nearest];
    }

    public static int getClosestReefFace(Pose2d curr, List<Pose2d> candidates) {
        curr = Constants.shouldFlipPath() ? FlippingUtil.flipFieldPose(curr) : curr;

        // int nearest = 0;
        // int penultimateNearest = 0;
        // for (int i = 0; i < AlignmentConstants.kREEF_CENTER_FACES.length; i++) {
        //     if (AlignmentConstants.kREEF_CENTER_FACES[i].getTranslation()
        //             .getDistance(curr.getTranslation()) < AlignmentConstants.kREEF_CENTER_FACES[nearest]
        //                     .getTranslation().getDistance(curr.getTranslation())) {
        //         nearest = i;
        //     }
        // }

        // for (int i = 0; i < AlignmentConstants.kREEF_CENTER_FACES.length; i++) {
        //     if (i == nearest) {
        //         continue;
        //     }
        //     if (AlignmentConstants.kREEF_CENTER_FACES[i].getTranslation()
        //             .getDistance(curr.getTranslation()) < AlignmentConstants.kREEF_CENTER_FACES[nearest]
        //                     .getTranslation().getDistance(curr.getTranslation())) {
        //         penultimateNearest = i;
        //     }
        // }

        // return candidates.get(nearest).getTranslation().getDistance(curr.getTranslation()) < candidates
        //         .get(penultimateNearest).getTranslation().getDistance(curr.getTranslation()) ? nearest
        //                 : penultimateNearest;

        int index = 0;
        for (int i = 1; i < candidates.size(); i++) {
            if (candidates.get(i).getTranslation().getDistance(curr.getTranslation()) < candidates.get(index).getTranslation().getDistance(curr.getTranslation())) {
                index = i;
            }
        }

        return index;
    }

    public static Pose2d getClosestAlgayPose() {
        Pose2d current = RobotState.getInstance().getEstimatedPose();
        Pose2d nearest = algayCandidates.get(getClosestReefFace(current, algayCandidates));

        Logger.recordOutput("AlignmentUtil/alignmentPoseSearch/nearest", nearest);
        return nearest;
    }

    public static Pose2d getClosestAlgayPose(Translation2d pose) {
        Pose2d current = new Pose2d(pose, new Rotation2d());
        Pose2d nearest = algayCandidates.get(getClosestReefFace(current, algayCandidates));

        Logger.recordOutput("AlignmentUtil/alignmentPoseSearch/nearest", nearest);
        return nearest;
    }

    public static Pose2d getClosestAlgayRecessedPose() {
        return offsetPoseToPreAlignment(getClosestAlgayPose(), config.getAlgayRecessPoseOffset());
    }

    public static Pose2d getClosestAlgayRecessedPose(Translation2d pose) {
        return offsetPoseToPreAlignment(getClosestAlgayPose(pose), config.getAlgayRecessPoseOffset());
    }

    public static Axis getBargeAxis() {
        return bargeAxis;
    }

    public static Pose2d getClosestLeftBranchPose() { // relative to blue driver station
        Pose2d current = RobotState.getInstance().getEstimatedPose();
        Pose2d nearest = leftBranchCandidates.get(getClosestReefFace(current, leftBranchCandidates));

        Logger.recordOutput("AlignmentUtil/alignmentPoseSearch/nearest", nearest);
        return nearest;
    }

    public static Pose2d getClosestRightBranchPose() { // relative to blue driver station
        Pose2d current = RobotState.getInstance().getEstimatedPose();
        Pose2d nearest = rightBranchCandidates.get(getClosestReefFace(current, rightBranchCandidates));

        Logger.recordOutput("AlignmentUtil/alignmentPoseSearch/nearest", nearest);
        return nearest;
    }

    public static Pose2d getClosestLeftBranchPose(Translation2d pose) { // relative to blue driver station
        Pose2d current = new Pose2d(pose, new Rotation2d());
        Pose2d nearest = leftBranchCandidates.get(getClosestReefFace(current, leftBranchCandidates));

        Logger.recordOutput("AlignmentUtil/alignmentPoseSearch/nearest", nearest);
        return nearest;
    }

    public static Pose2d getClosestRightBranchPose(Translation2d pose) { // relative to blue driver station
        Pose2d current = new Pose2d(pose, new Rotation2d());
        Pose2d nearest = rightBranchCandidates.get(getClosestReefFace(current, rightBranchCandidates));

        Logger.recordOutput("AlignmentUtil/alignmentPoseSearch/nearest", nearest);
        return nearest;
    }


    public static Pose2d getClosestBargePose() {
        Pose2d curr = RobotState.getInstance().getEstimatedPose();
        double rot = Math.abs(180-Math.abs(curr.getRotation().getDegrees())) <= 90 ? Math.PI : 0;
        Pose2d nearest = new Pose2d(bargeAxis.getPointOnAxis(curr.getTranslation()), new Rotation2d(rot));
        Logger.recordOutput("AlignmentUtil/alignmentPoseSearch/nearest", nearest);
        return nearest;
    }

    public static Pose2d getClosestSourcePose() { 
        Pose2d current = RobotState.getInstance().getEstimatedPose();

        Pose2d rightNearest = new Pose2d(rightSourceAxis.getPointOnAxis(current.getTranslation()), AlignmentConstants.kRIGHT_SOURCE_AXIS_ROTATION);
        Pose2d leftNearest = new Pose2d(leftSourceAxis.getPointOnAxis(current.getTranslation()), AlignmentConstants.kLEFT_SOURCE_AXIS_ROTATION);

        Pose2d nearest = 
            current.getTranslation().getDistance(rightNearest.getTranslation()) < 
            current.getTranslation().getDistance(leftNearest.getTranslation()) ?
                rightNearest :
                leftNearest;

        Logger.recordOutput("AlignmentUtil/alignmentPoseSearch/nearest", nearest);
        return nearest;
    }

    public static Pose2d getClosestCagePose() {
        Pose2d current = RobotState.getInstance().getEstimatedPose();
        Pose2d nearest = current.nearest(cageCandidates);
        Logger.recordOutput("AlignmentUtil/alignmentPoseSearch/nearest", nearest);
        return nearest;
    }

    public static Pose2d getClosestCagePoseRecessed() {
        return offsetPoseToPreAlignment(getClosestCagePose(), config.getAlgayRecessPoseOffset());
    }

    public static Axis flipAxis(Axis axis) {
        if (Double.isNaN(axis.x1) || Double.isNaN(axis.x2) &&
            Double.isNaN(axis.y1) || Double.isNaN(axis.y2)) {
            return axis;
        }

        Pose2d p1 = FlippingUtil.flipFieldPose(new Pose2d(axis.x1, axis.y1, axis.getAngle().plus(new Rotation2d(Math.PI / 2))));
        Pose2d p2 = FlippingUtil.flipFieldPose(new Pose2d(axis.x2, axis.y2, axis.getAngle().plus(new Rotation2d(Math.PI / 2))));

        return new Axis(p1.getX(), p1.getY(), p2.getX(), p2.getY());
    }

    public static Axis offsetAxis(Axis axis, boolean flipOffsetDirection) {
        if (Double.isNaN(axis.x1) || Double.isNaN(axis.x2) &&
            Double.isNaN(axis.y1) || Double.isNaN(axis.y2)) {
            return axis;
        }

        int invert = flipOffsetDirection ? -1 : 1;

        Pose2d p1 = new Pose2d(axis.x1, axis.y1, axis.getAngle().plus(new Rotation2d(Math.PI / 2)));
        Pose2d p2 = new Pose2d(axis.x2, axis.y2, axis.getAngle().plus(new Rotation2d(Math.PI / 2)));

        p1 = p1.transformBy(
            new Transform2d(
                invert * config.getBumperLengthMeters() / 2,
                0,
                new Rotation2d(0)
            )
        );

        p2 = p2.transformBy(
            new Transform2d(
                invert * config.getBumperLengthMeters() / 2, // this is the bumper radius
                0,
                new Rotation2d(0)
            )
        );

        return new Axis(p1.getX(), p1.getY(), p2.getX(), p2.getY());
    }
}
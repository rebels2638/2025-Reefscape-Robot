// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import java.util.ArrayList;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.lib.util.AlignmentUtil.Axis;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final Mode currentMode = Mode.COMP; // TODO: change this if sim
    // public static final boolean isSYSID = true; // TODO: change this if sysid

    public static enum Mode {
        /** Running on a real robot. */
        COMP,

        PROTO,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static final double kLOOP_CYCLE_MS;
    static {
        switch (currentMode) {
            case COMP:
                kLOOP_CYCLE_MS = 0.02;

                break;

            case PROTO:
                kLOOP_CYCLE_MS = 0.02;

                break;

            case SIM:
                kLOOP_CYCLE_MS = 0.02;

                break;

            case REPLAY:
                kLOOP_CYCLE_MS = 0.02;

                break;

            default:
                kLOOP_CYCLE_MS = 0.02;

                break;
        }
    }

    public static final class OperatorConstants {
        public static final int kDRIVER_CONTROLLER_PORT = 0;

        // Joystick Deadband
        // yes, this high
        public static final double LEFT_X_DEADBAND = 0.09;
        public static final double LEFT_Y_DEADBAND = 0.09;

        public static final double RIGHT_X_DEADBAND = 0.12;

        private OperatorConstants() {
        }
    }

    public static final class VisionConstants {
        public static final int[] kALL_TAG_IDS = new int[22];
        static {
            for (int i = 1; i <= 22; i++) {
                kALL_TAG_IDS[i-1] = i;
            }
        }

        public static final int[] kREEF_TAG_IDS = { // in order of the alignment center faces
            18,
            22,
            17,
            21,
            19,
            20,

            7,
            9,
            8,
            10,
            6,
            11
        };

        public static final Translation2d[] kREEF_TAG_POSES = new Translation2d[12];
        static {
            for (int i = 0; i < 6; i++) {
                kREEF_TAG_POSES[i] = AlignmentConstants.kREEF_CENTER_FACES[i].getTranslation();
            }
            for (int i = 6; i < 12; i++) {
                kREEF_TAG_POSES[i] = FlippingUtil.flipFieldPosition(AlignmentConstants.kREEF_CENTER_FACES[i-6].getTranslation());
            }
        }

        private VisionConstants() {}
    }

    public static final class AlignmentConstants {
        // these assume the robot's volume is zero. does not take into account frame
        public static final double kINTER_BRANCH_DIST_METER = 0.34;

        public static final Pose2d[] kREEF_CENTER_FACES = new Pose2d[6]; // Starting facing the driver station in clockwise
                                                                    // order
        static {
            // Initialize faces
            // bottom
            kREEF_CENTER_FACES[0] =
                    new Pose2d(
                            3.642,
                            4.024,
                            Rotation2d.fromDegrees(0));
            // top right
            kREEF_CENTER_FACES[1] = new Pose2d(
                    4.916,
                    3.285,
                    Rotation2d.fromDegrees(120));
            // bottom right
            kREEF_CENTER_FACES[2] = new Pose2d(
                    4.064,
                    3.291,
                    Rotation2d.fromDegrees(60));
            // top
            kREEF_CENTER_FACES[3] = new Pose2d(
                    5.344,
                    4.023,
                    Rotation2d.fromDegrees(180));
            // bottom left
            kREEF_CENTER_FACES[4] = new Pose2d(
                    4.064,
                    4.763,
                    Rotation2d.fromDegrees(-60));
            // top left
            kREEF_CENTER_FACES[5] = new Pose2d(
                    4.912,
                    4.770,
                    Rotation2d.fromDegrees(-120));
        }

        public static final Pose2d[] kSOURCE_CENTER_FACES = new Pose2d[6]; 
        static {
            // right top
            kSOURCE_CENTER_FACES[0] = new Pose2d(
                1.281,
                0.269,
                Rotation2d.fromDegrees(54)
            );

            // right center
            kSOURCE_CENTER_FACES[1] = new Pose2d(
                0.845,
                0.607,
                Rotation2d.fromDegrees(54)
            );

            // right bottom
            kSOURCE_CENTER_FACES[2] = new Pose2d(
                0.423,
                0.911,
                Rotation2d.fromDegrees(54)
            );

            // left top
            kSOURCE_CENTER_FACES[3] = new Pose2d(
                1.281,
                7.758,
                Rotation2d.fromDegrees(-54)
            );

            // left center
            kSOURCE_CENTER_FACES[4] = new Pose2d(
                0.845,
                7.44,
                Rotation2d.fromDegrees(-54)
            );

            // left bottom
            kSOURCE_CENTER_FACES[5] = new Pose2d(
                0.423,
                7.126,
                Rotation2d.fromDegrees(-54)
            );
        }

        public static final Pose2d[] kCAGE_CENTERS = new Pose2d[6];
        static {
            kCAGE_CENTERS[0] = new Pose2d(8.774, 7.284, new Rotation2d(0));
            kCAGE_CENTERS[1] = new Pose2d(8.774, 6.169, new Rotation2d(0));
            kCAGE_CENTERS[2] = new Pose2d(8.774, 5.079, new Rotation2d(0));
            kCAGE_CENTERS[3] = new Pose2d(8.774, 3.000, new Rotation2d(0));
            kCAGE_CENTERS[4] = new Pose2d(8.774, 1.885, new Rotation2d(0));
            kCAGE_CENTERS[5] = new Pose2d(8.774, 0.770, new Rotation2d(0));
        }

        public static final Axis kBARGE_AXIS = 
            new Axis(
                7.642292499542236,
                4.77,
                7.642292499542236,
                7.5 
            );
        public static final Rotation2d kBARGE_ROTATION = Rotation2d.fromDegrees(0);

        public static final Axis kRIGHT_SOURCE_AXIS = 
            new Axis(
                kSOURCE_CENTER_FACES[0].getX(),
                kSOURCE_CENTER_FACES[0].getY(),
                kSOURCE_CENTER_FACES[2].getX(),
                kSOURCE_CENTER_FACES[2].getY()
            );
        public static final Rotation2d kRIGHT_SOURCE_AXIS_ROTATION = Rotation2d.fromDegrees(54);

        public static final Axis kLEFT_SOURCE_AXIS = 
            new Axis(
                kSOURCE_CENTER_FACES[3].getX(),
                kSOURCE_CENTER_FACES[3].getY(),
                kSOURCE_CENTER_FACES[5].getX(),
                kSOURCE_CENTER_FACES[5].getY()
            );
        public static final Rotation2d kLEFT_SOURCE_AXIS_ROTATION = Rotation2d.fromDegrees(-54);

        private AlignmentConstants() {}
    }

    public static boolean shouldFlipPath() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }
}
package frc.robot.constants;

import java.util.ArrayList;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;

public class SwerveConfigBase {
    public static final SwerveModuleConfig.GeneralConfig kSHARED_GENERAL_CONFIG = new SwerveModuleConfig.GeneralConfig(
            "rio",
            40.0,
            0.5,
            20.0,
            40.0,
            60.0,
            60.0,
            0.1,
            0.2,
            0.3,
            0.4,
            0.0,
            0.0,
            3.0,
            5.0,
            6.0,
            true,
            6.75,
            0.0762,
            false,
            30.0,
            0.5,
            15.0,
            30.0,
            40.0,
            40.0,
            0.1,
            0.2,
            0.3,
            0.4,
            0.0,
            0.0,
            5.0,
            0,
            0,
            false,
            12.8,
            1.0,
            false,
            FeedbackSensorSourceValue.FusedCANcoder,
            SensorDirectionValue.CounterClockwise_Positive,
            1);

    public static final SwerveModuleConfig kFRONT_LEFT_CONFIG = new SwerveModuleConfig(
            1,
            2,
            false,
            false,
            true,
            true,
            3,
            0.0,
            kSHARED_GENERAL_CONFIG);

    public static final SwerveModuleConfig kFRONT_RIGHT_CONFIG = new SwerveModuleConfig(
            4,
            5,
            false,
            false,
            true,
            true,
            6,
            0.0,
            kSHARED_GENERAL_CONFIG);

    public static final SwerveModuleConfig kBACK_LEFT_CONFIG = new SwerveModuleConfig(
            7,
            8,
            false,
            false,
            true,
            true,
            9,
            0.0,
            kSHARED_GENERAL_CONFIG);

    public static final SwerveModuleConfig kBACK_RIGHT_CONFIG = new SwerveModuleConfig(
            10,
            11,
            false,
            false,
            true,
            true,
            12,
            0.0,
            kSHARED_GENERAL_CONFIG);

    public static final SwerveDrivetrainConfig K_SWERVE_DRIVETRAIN_CONFIG = new SwerveDrivetrainConfig(
            4,
            3.5,
            3.7,
            12,
            new Translation2d(0.38, -0.38),
            new Translation2d(-0.38, -0.38),
            new Translation2d(0.38, 0.38),
            new Translation2d(-0.38, 0.38));

    private static final ArrayList<double[]> kDRIVE_FF_POINTS = new ArrayList<double[]>();
    static {
        kDRIVE_FF_POINTS.add(new double[] { 0, 0, 0 });
        kDRIVE_FF_POINTS.add(new double[] {
                K_SWERVE_DRIVETRAIN_CONFIG.kMAX_DRIVETRAIN_TRANSLATIONAL_VELOCITY_METERS_PER_SEC / 2, 0, 0 });
        kDRIVE_FF_POINTS
                .add(new double[] { K_SWERVE_DRIVETRAIN_CONFIG.kMAX_DRIVETRAIN_TRANSLATIONAL_VELOCITY_METERS_PER_SEC,
                        0, 0 });
        kDRIVE_FF_POINTS.add(new double[] { 0,
                K_SWERVE_DRIVETRAIN_CONFIG.kMAX_DRIVETRAIN_ANGULAR_VELOCITY_RADIANS_PER_SEC / 2, 0 });
        kDRIVE_FF_POINTS.add(
                new double[] { 0, K_SWERVE_DRIVETRAIN_CONFIG.kMAX_DRIVETRAIN_ANGULAR_VELOCITY_RADIANS_PER_SEC, 0 });
        kDRIVE_FF_POINTS
                .add(new double[] { K_SWERVE_DRIVETRAIN_CONFIG.kMAX_DRIVETRAIN_TRANSLATIONAL_VELOCITY_METERS_PER_SEC,
                        K_SWERVE_DRIVETRAIN_CONFIG.kMAX_DRIVETRAIN_ANGULAR_VELOCITY_RADIANS_PER_SEC, 1.8 }); // 1.6
        kDRIVE_FF_POINTS.add(
                new double[] { K_SWERVE_DRIVETRAIN_CONFIG.kMAX_DRIVETRAIN_TRANSLATIONAL_VELOCITY_METERS_PER_SEC / 2,
                        K_SWERVE_DRIVETRAIN_CONFIG.kMAX_DRIVETRAIN_ANGULAR_VELOCITY_RADIANS_PER_SEC, 0.8 }); // .6
        kDRIVE_FF_POINTS
                .add(new double[] { K_SWERVE_DRIVETRAIN_CONFIG.kMAX_DRIVETRAIN_TRANSLATIONAL_VELOCITY_METERS_PER_SEC,
                        K_SWERVE_DRIVETRAIN_CONFIG.kMAX_DRIVETRAIN_ANGULAR_VELOCITY_RADIANS_PER_SEC / 2, 0.25 });// 0.2
        kDRIVE_FF_POINTS.add(
                new double[] { K_SWERVE_DRIVETRAIN_CONFIG.kMAX_DRIVETRAIN_TRANSLATIONAL_VELOCITY_METERS_PER_SEC / 2,
                K_SWERVE_DRIVETRAIN_CONFIG.kMAX_DRIVETRAIN_ANGULAR_VELOCITY_RADIANS_PER_SEC / 2, 0.2 }); // .2
    }
    public static final SwerveControllerConfig K_SWERVE_DRIVETRAIN_CONTROLLER_CONFIG = new SwerveControllerConfig(
                kDRIVE_FF_POINTS,
                new PIDController(0, 0, 0),
                new PIDController(0, 0, 0),
                new PIDController(0, 0, 0)
    );
}

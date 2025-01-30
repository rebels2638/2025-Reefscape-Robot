package frc.robot.constants.swerve;

import java.util.ArrayList;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;

public final class SwerveCrescendoRobotBaseConfig extends SwerveConfigBase {
    public SwerveCrescendoRobotBaseConfig() {
        this.kSHARED_GENERAL_CONFIG = new SwerveModuleConfig.GeneralConfig(
                "drivetrain",
                60.0,
                0.5,
                20.0,
                60.0,
                60.0,
                -60.0,
                2.5,
                0.2,
                4.2,
                15,
                0.0,
                3,
                7,
                14,
                40.0,
                4,
                5.4,
                true,
                6.12,
                0.04861,
                false,
                30.0,
                1.5,
                30.0,
                40.0,
                40.0,
                -40.0,
                1.5,
                1.17,//0.00156
                0.50, //0.015
                350,//0.01
                0.0,
                10,//0.1
                0.25,
                2.6,
                6,
                true,
                21.428,
                21.428,
                true,
                FeedbackSensorSourceValue.FusedCANcoder,
                SensorDirectionValue.CounterClockwise_Positive,
                1);

        this.kFRONT_LEFT_CONFIG = new SwerveModuleConfig(
                4,
                5,
                true,
                true,
                true,
                true,
                10,
                -0.86,
                kSHARED_GENERAL_CONFIG);

        this.kFRONT_RIGHT_CONFIG = new SwerveModuleConfig(
                2,
                3,
                true,
                true,
                true,
                true,
                9,
                -0.68,
                kSHARED_GENERAL_CONFIG);

        this.kBACK_LEFT_CONFIG = new SwerveModuleConfig(
                6,
                7,
                true,
                true,
                true,
                true,
                11,
                -0.78,
                kSHARED_GENERAL_CONFIG);

        this.kBACK_RIGHT_CONFIG = new SwerveModuleConfig(
                1,
                0,
                true,
                true,
                true,
                true,
                8,
                -0.98,
                kSHARED_GENERAL_CONFIG);

        this.kSWERVE_DRIVETRAIN_CONFIG = new SwerveDrivetrainConfig(
                4,
                3.5,
                3.7,
                12,
                new Translation2d(0.38, 0.38),
                new Translation2d(0.38, -0.38),
                new Translation2d(-0.38, 0.38),
                new Translation2d(-0.38, -0.38),
                0 //dont compensate for drift
            );
        
        this.kPATHPLANNER_ROBOT_CONFIG = new RobotConfig(
            27.088, 
            3.5,
            new ModuleConfig(
                kSHARED_GENERAL_CONFIG.kDRIVE_WHEEL_RADIUS_METERS,
                kSHARED_GENERAL_CONFIG.kDRIVE_TRUE_MAX_VELOCITY_METERS_PER_SEC,
                1.2,
                // TODO: CHECK THIS REDUCTION!!!
                DCMotor.getFalcon500(1).withReduction(kSHARED_GENERAL_CONFIG.kDRIVE_MOTOR_TO_OUTPUT_SHAFT_RATIO),
                kSHARED_GENERAL_CONFIG.kDRIVE_STATOR_CURRENT_LIMIT,
                1
            ),
            kSWERVE_DRIVETRAIN_CONFIG.kFRONT_LEFT_POSITION_METERS,
            kSWERVE_DRIVETRAIN_CONFIG.kFRONT_RIGHT_POSITION_METERS,
            kSWERVE_DRIVETRAIN_CONFIG.kBACK_LEFT_POSITION_METERS,
            kSWERVE_DRIVETRAIN_CONFIG.kBACK_RIGHT_POSITION_METERS
        );
        ArrayList<double[]> kDRIVE_FF_POINTS = new ArrayList<double[]>();
        kDRIVE_FF_POINTS.add(new double[] { 0, 0, 0 });
        kDRIVE_FF_POINTS.add(new double[] {
                this.kSWERVE_DRIVETRAIN_CONFIG.kMAX_DRIVETRAIN_TRANSLATIONAL_VELOCITY_METERS_PER_SEC / 2, 0, 0 });
        kDRIVE_FF_POINTS
                .add(new double[] { kSWERVE_DRIVETRAIN_CONFIG.kMAX_DRIVETRAIN_TRANSLATIONAL_VELOCITY_METERS_PER_SEC,
                        0, 0 });
        kDRIVE_FF_POINTS.add(new double[] { 0,
                kSWERVE_DRIVETRAIN_CONFIG.kMAX_DRIVETRAIN_ANGULAR_VELOCITY_RADIANS_PER_SEC / 2, 0 });
        kDRIVE_FF_POINTS.add(
                new double[] { 0, kSWERVE_DRIVETRAIN_CONFIG.kMAX_DRIVETRAIN_ANGULAR_VELOCITY_RADIANS_PER_SEC, 0 });
        kDRIVE_FF_POINTS
                .add(new double[] { kSWERVE_DRIVETRAIN_CONFIG.kMAX_DRIVETRAIN_TRANSLATIONAL_VELOCITY_METERS_PER_SEC,
                        kSWERVE_DRIVETRAIN_CONFIG.kMAX_DRIVETRAIN_ANGULAR_VELOCITY_RADIANS_PER_SEC, 0 }); // 1.6
        kDRIVE_FF_POINTS.add(
                new double[] { kSWERVE_DRIVETRAIN_CONFIG.kMAX_DRIVETRAIN_TRANSLATIONAL_VELOCITY_METERS_PER_SEC / 2,
                        kSWERVE_DRIVETRAIN_CONFIG.kMAX_DRIVETRAIN_ANGULAR_VELOCITY_RADIANS_PER_SEC, 0 }); // .6
        kDRIVE_FF_POINTS
                .add(new double[] { kSWERVE_DRIVETRAIN_CONFIG.kMAX_DRIVETRAIN_TRANSLATIONAL_VELOCITY_METERS_PER_SEC,
                        kSWERVE_DRIVETRAIN_CONFIG.kMAX_DRIVETRAIN_ANGULAR_VELOCITY_RADIANS_PER_SEC / 2, 0 });// 0.2
        kDRIVE_FF_POINTS.add(
                new double[] { kSWERVE_DRIVETRAIN_CONFIG.kMAX_DRIVETRAIN_TRANSLATIONAL_VELOCITY_METERS_PER_SEC / 2,
                        kSWERVE_DRIVETRAIN_CONFIG.kMAX_DRIVETRAIN_ANGULAR_VELOCITY_RADIANS_PER_SEC / 2, 0 }); // .2

        this.kSWERVE_DRIVETRAIN_CONTROLLER_CONFIG = new SwerveControllerConfig(
                kDRIVE_FF_POINTS,
                new PIDController(0, 0, 0),
                new PIDController(0, 0, 0),
                new PIDController(0, 0, 0),
                0.5 * this.kSWERVE_DRIVETRAIN_CONFIG.kMAX_DRIVETRAIN_ANGULAR_VELOCITY_RADIANS_PER_SEC);
    }
}
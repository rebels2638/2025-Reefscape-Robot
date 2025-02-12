package frc.robot.constants.swerve.drivetrainConfigs;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.constants.swerve.moduleConfigs.comp.SwerveModuleGeneralConfigComp;

public class SwerveDrivetrainConfigComp extends SwerveDrivetrainConfigBase {

    public static SwerveDrivetrainConfigComp instance = null;
    public static SwerveDrivetrainConfigComp getInstance() {
        if (instance == null) {
            instance = new SwerveDrivetrainConfigComp();
        }
        return instance;
    }

    private SwerveDrivetrainConfigComp() {}

    private final double maxTranslationalVelocity = 4.0;
    private final double maxTranslationalAcceleration = 1.5;
    private final double maxAngularVelocity = 3.7;
    private final double maxAngularAcceleration = 12.0;
    private final double maxAutoModuleVelocity = 5.0;

    private final Translation2d frontLeftPosition = new Translation2d(0.24, 0.24);
    private final Translation2d frontRightPosition = new Translation2d(0.24, -0.24);
    private final Translation2d backLeftPosition = new Translation2d(-0.24, 0.24);
    private final Translation2d backRightPosition = new Translation2d(-0.24, -0.24);

    private final double rotationCompensationCoefficient = 0.0;

    private final RobotConfig autoConfig = new RobotConfig(
            37.88,
            13.5,
            new ModuleConfig(
                SwerveModuleGeneralConfigComp.getInstance().getDriveWheelRadiusMeters(), 
                5.4, 
                1.2, 
                DCMotor.getKrakenX60Foc(1).
                    withReduction(
                        SwerveModuleGeneralConfigComp.getInstance().getDriveMotorToOutputShaftRatio()
                    ),
                SwerveModuleGeneralConfigComp.getInstance().getDriveStatorCurrentLimit(), 
                1
            ),
            frontLeftPosition, 
            frontRightPosition, 
            backLeftPosition, 
            backRightPosition
        );

    private final PIDConstants steerPIDConstants = new PIDConstants(1,0,0,0);
    private final PIDConstants drivePIDConstants = new PIDConstants(4,0,0,0);

    @Override
    public double getMaxDrivetrainTranslationalVelocityMetersPerSec() {
        return maxTranslationalVelocity;
    }

    @Override
    public double getMaxDrivetrainTranslationalAccelerationMetersPerSecSec() {
        return maxTranslationalAcceleration;
    }

    @Override
    public double getMaxDrivetrainAngularVelocityRadiansPerSec() {
        return maxAngularVelocity;
    }

    @Override
    public double getMaxDrivetrainAngularAccelerationRadiansPerSecSec() {
        return maxAngularAcceleration;
    }

    @Override
    public Translation2d getFrontLeftPositionMeters() {
        return frontLeftPosition;
    }

    @Override
    public Translation2d getFrontRightPositionMeters() {
        return frontRightPosition;
    }

    @Override
    public Translation2d getBackLeftPositionMeters() {
        return backLeftPosition;
    }

    @Override
    public Translation2d getBackRightPositionMeters() {
        return backRightPosition;
    }

    @Override
    public double getRotationCompensationCoefficient() {
        return rotationCompensationCoefficient;
    }

    @Override
    public PIDController getAutoAlignProfiledTranslationController() {
        PIDController p = new PIDController(3, 0, 0.01);
        p.setTolerance(Math.sqrt(getAutoAlignTranslationTolerance()));

        return p;
    }

    @Override
    public PIDController getAutoAlignProfiledRotationController() {
        PIDController p = new PIDController(.5, 0, 0);
        p.setTolerance(getAutoAlignRotationTolerance());
        p.enableContinuousInput(-Math.PI, Math.PI);

        return p;
    }

    @Override
    public double getAutoAlignTranslationTolerance() {
        return 0.02;
    }

    @Override
    public double getAutoAlignTranslationVeloTolerance() {
        return 0.03;
    }

    @Override
    public double getAutoAlignRotationTolerance() {
        return Math.toRadians(5);
    }

    @Override
    public double getAutoAlignRotationVeloTolerance() {
        return Math.toRadians(3);
    }

    @Override
    public RobotConfig getRobotConfig() {
        return autoConfig;
    }

    @Override
    public PIDConstants getPathplannerSteerPIDConfig() {
        return steerPIDConstants;
    }

    @Override
    public PIDConstants getPathplannerDrivePIDConfig() {
        return drivePIDConstants;
    }

    @Override
    public double getMaxAutoModuleVelocity() {
        return maxAutoModuleVelocity;
    }

    @Override
    public double getBumperLengthMeters() {
        return .88;
    }

    @Override
    public Translation2d getCoralOffsetFromRobotCenter() {
        return new Translation2d(0.0,0.15); // -0.019
    }

    @Override
    public Translation2d getAlgayOffsetFromRobotCenter() {
        return new Translation2d(0, 0);
    }
}

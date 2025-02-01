package frc.robot.constants.swerve.drivetrainConfigs;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.constants.swerve.moduleConfigs.comp.SwerveModuleGeneralConfigComp;

public class SwerveDrivetrainConfigProto extends SwerveDrivetrainConfigBase {

    public static SwerveDrivetrainConfigProto instance = null;
    public static SwerveDrivetrainConfigProto getInstance() {
        if (instance == null) {
            instance = new SwerveDrivetrainConfigProto();
        }
        return instance;
    }

    private SwerveDrivetrainConfigProto() {}

    private final double maxTranslationalVelocity = 4.0;
    private final double maxTranslationalAcceleration = 3.5;
    private final double maxAngularVelocity = 3.7;
    private final double maxAngularAcceleration = 12.0;
    private final double maxAutoModuleVelocity = 5.0;

    private final Translation2d frontLeftPosition = new Translation2d(0.38, 0.38);
    private final Translation2d frontRightPosition = new Translation2d(0.38, -0.38);
    private final Translation2d backLeftPosition = new Translation2d(-0.38, 0.38);
    private final Translation2d backRightPosition = new Translation2d(-0.38, -0.38);

    private final double rotationCompensationCoefficient = 0.0;

    private final RobotConfig autoConfig = new RobotConfig(
            27.88,
            3.5,
            new ModuleConfig(
                SwerveModuleGeneralConfigComp.getInstance().getDriveWheelRadiusMeters(), 
                5.4, 
                1.2, 
                DCMotor.getFalcon500(1).
                    withReduction(
                        SwerveModuleGeneralConfigComp.getInstance().getDriveMotorToOutputShaftRatio()
                    ),
                SwerveModuleGeneralConfigComp.getInstance().getDriveStatorCurrentLimit(), 
                1
            ),
            SwerveDrivetrainConfigComp.getInstance().getFrontLeftPositionMeters(), 
            SwerveDrivetrainConfigComp.getInstance().getFrontRightPositionMeters(), 
            SwerveDrivetrainConfigComp.getInstance().getBackLeftPositionMeters(), 
            SwerveDrivetrainConfigComp.getInstance().getBackRightPositionMeters()
        );

    private final PIDConstants steerPIDConstants = new PIDConstants(0,0,0,0);
    private final PIDConstants drivePIDConstants = new PIDConstants(0,0,0,0);

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
        return new PIDController(0, 0, 0);
    }

    @Override
    public PIDController getAutoAlignProfiledRotationController() {
        return new PIDController(0, 0, 0);
    }


    @Override
    public RobotConfig getRobotConfig() {
        return autoConfig;
    }

    @Override
    public PIDConstants getSteerPIDConfig() {
        return steerPIDConstants;
    }

    @Override
    public PIDConstants getDrivePIDConfig() {
        return drivePIDConstants;
    }

    @Override
    public double getMaxAutoModuleVelocity() {
        return maxAutoModuleVelocity;
    }

}

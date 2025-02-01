package frc.robot.constants.swerve.drivetrainConfigs;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;

public class SwerveDrivetrainConfigSim extends SwerveDrivetrainConfigBase {

    public static SwerveDrivetrainConfigSim instance = null;
    public static SwerveDrivetrainConfigSim getInstance() {
        if (instance == null) {
            instance = new SwerveDrivetrainConfigSim();
        }
        return instance;
    }

    private SwerveDrivetrainConfigSim() {}
    
    private final double maxTranslationalVelocity = 4.0;
    private final double maxTranslationalAcceleration = 3.5;
    private final double maxAngularVelocity = 3.7;
    private final double maxAngularAcceleration = 12.0;

    private final Translation2d frontLeftPosition = new Translation2d(0.38, 0.38);
    private final Translation2d frontRightPosition = new Translation2d(0.38, -0.38);
    private final Translation2d backLeftPosition = new Translation2d(-0.38, 0.38);
    private final Translation2d backRightPosition = new Translation2d(-0.38, -0.38);

    private final double rotationCompensationCoefficient = 0.0;

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
}

package frc.robot.subsystems.drivetrain.swerve;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.subsystems.drivetrain.swerve.controllers.DriveFFController;
import frc.robot.subsystems.drivetrain.swerve.gyro.GyroIO;
import frc.robot.subsystems.drivetrain.swerve.gyro.GyroIOInputsAutoLogged;
import frc.robot.subsystems.drivetrain.swerve.gyro.GyroIONavX;
import frc.robot.subsystems.drivetrain.swerve.gyro.GyroIOPigeon2;
import frc.robot.subsystems.drivetrain.swerve.module.ModuleIO;
import frc.robot.subsystems.drivetrain.swerve.module.ModuleIOInputsAutoLogged;
import frc.robot.subsystems.drivetrain.swerve.module.ModuleIOSim;
import frc.robot.subsystems.drivetrain.swerve.module.ModuleIOTalonFX;
import frc.robot.constants.*;
import frc.robot.constants.swerve.SwerveConfigBase;
import frc.robot.constants.swerve.SwerveCrescendoRobotBaseConfig;
import frc.robot.constants.swerve.SwerveCompConfig;
import frc.robot.constants.swerve.SwerveSimConfig;

public class SwerveDrive extends SubsystemBase {
    private final DriveFFController driveFFController;
    private final PIDController rotationalVelocityFeedbackController;
    private final PIDController translationalVelocityFeedbackController;
    private final PIDController rotationalPositionFeedbackController;

    private Rotation2d rotationLock;

    private ModuleIO[] modules;

    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                new Translation2d(0.38, -0.38),
                new Translation2d(-0.38, -0.38),
                new Translation2d(0.38, 0.38),
                new Translation2d(-0.38, 0.38));

    private ModuleIOInputsAutoLogged[] moduleInputs = {
            new ModuleIOInputsAutoLogged(),
            new ModuleIOInputsAutoLogged(),
            new ModuleIOInputsAutoLogged(),
            new ModuleIOInputsAutoLogged()
    };

    private final GyroIO gyroIO;
    private GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private final SwerveConfigBase config;

    private boolean isTranslationSlowdownEnabled = false;
    private boolean isRotationSlowdownEnabled = false;
    private boolean isChassisSpeedsDiscretizeEnabled = true;
    private boolean isRotationLockEnabled = false;

    private double lastTimestamp = Double.NEGATIVE_INFINITY;
    private double translationCoeffecient = 0.5;
    private double rotationCoefficient = 0.5;

    @SuppressWarnings("static-access")
    public SwerveDrive() {
        switch (Constants.currentMode) {
            case Comp:
                config = new SwerveCompConfig();
                modules = new ModuleIO[] {
                    new ModuleIOTalonFX(config, config.getFrontLeftConfig().kSPECIFIC_CONFIG, 0),
                    new ModuleIOTalonFX(config, config.getFrontRightConfig().kSPECIFIC_CONFIG, 1),
                    new ModuleIOTalonFX(config, config.getBackLeftConfig().kSPECIFIC_CONFIG, 2),
                    new ModuleIOTalonFX(config, config.getBackRightConfig().kSPECIFIC_CONFIG, 3)
                };
                
                gyroIO = new GyroIOPigeon2();
                Phoenix6Odometry.getInstance().start();
                break;

            case CrescendoRobotBase:
                config = new SwerveCrescendoRobotBaseConfig();
                modules = new ModuleIO[] {
                    new ModuleIOTalonFX(config, config.getFrontLeftConfig().kSPECIFIC_CONFIG, 0),
                    new ModuleIOTalonFX(config, config.getFrontRightConfig().kSPECIFIC_CONFIG, 1),
                    new ModuleIOTalonFX(config, config.getBackLeftConfig().kSPECIFIC_CONFIG, 2),
                    new ModuleIOTalonFX(config, config.getBackRightConfig().kSPECIFIC_CONFIG, 3)
                };
                
                gyroIO = new GyroIONavX();
                Phoenix6Odometry.getInstance().start();
                break;

            case SIM:
                config = new SwerveSimConfig();
                modules = new ModuleIO[] {
                    new ModuleIOSim(config, 0),
                    new ModuleIOSim(config, 1),
                    new ModuleIOSim(config, 2),
                    new ModuleIOSim(config, 3)
                };

                gyroIO = new GyroIO() {};
                break;

            default:
                config = new SwerveCompConfig();
                modules = new ModuleIO[] {
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {}
                };

                gyroIO = new GyroIOPigeon2();

                break;
        }

        driveFFController = new DriveFFController(config);

        rotationalVelocityFeedbackController = config.getSwerveDrivetrainControllerConfig().kROTATIONAL_VELOCITY_FEEDBACK_CONTROLLER;
        translationalVelocityFeedbackController = config.getSwerveDrivetrainControllerConfig().kTRANSLATION_VELOCITY_FEEDBACK_CONTROLLER;
        rotationalPositionFeedbackController = config.getSwerveDrivetrainControllerConfig().kROTATIONAL_POSITION_FEEDBACK_CONTROLLER;
    }

    @Override
    public void periodic() {
        var odometryTimestamp = 0.0;
        // Thread safe reading of the gyro and swerve inputs.
        // The read lock is released only after inputs are written via the write lock
        Phoenix6Odometry.getInstance().stateLock.readLock().lock();
        try {
            Logger.recordOutput("SwerveDrive/stateLockAcquired", true);
            gyroIO.updateInputs(gyroInputs);
            Logger.processInputs("SwerveDrive/gyro", gyroInputs);

            for (int i = 0; i < 4; i++) {
                modules[i].updateInputs(moduleInputs[i]);
                Logger.processInputs("SwerveDrive/module" + i, moduleInputs[i]);
                odometryTimestamp = Math.max(odometryTimestamp, moduleInputs[i].timestamp);
            }
        } finally {
            Phoenix6Odometry.getInstance().stateLock.readLock().unlock();
        }
        if (odometryTimestamp == 0.0) {
            odometryTimestamp = HALUtil.getFPGATime() / 1.0e6;
        }

        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
        SwerveModuleState[] moduleStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            modulePositions[i] = modules[i].getPosition();
            moduleStates[i] = modules[i].getState();
        }
        RobotState.getInstance()
            .addOdometryObservation(
                new RobotState.OdometryObservation(
                    modulePositions,
                    new Rotation2d(gyroInputs.orientation.getZ()),
                    odometryTimestamp
                    ));

        // ChassisSpeeds actualChassisSpeeds = RobotState.getInstance().getChassisSpeeds();

        Logger.recordOutput("SwerveDrive/measuredModuleStates", moduleStates);
        Logger.recordOutput("SwerveDrive/measuredModulePositions", modulePositions);
        // Logger.recordOutput("SwerveDrive/measuredChassisSpeeds", actualChassisSpeeds);

    }

    public void setTargetSpeed(ChassisSpeeds speeds) {
        ChassisSpeeds desiredSpeeds = speeds;

        if (isTranslationSlowdownEnabled) {
            desiredSpeeds.vxMetersPerSecond *= this.translationCoeffecient;
            desiredSpeeds.vyMetersPerSecond *= this.translationCoeffecient;
        }

        if (isRotationSlowdownEnabled) {
            desiredSpeeds.omegaRadiansPerSecond *= this.rotationCoefficient;
        }

        var angularVelocity = new Rotation2d(gyroInputs.angularVelocityRadPerSec*0.5); // TODO: make a coeff
        if (angularVelocity.getRadians() != 0.0) {
            desiredSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds( // why should this be split into two?
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond,
                speeds.omegaRadiansPerSecond,
                RobotState.getInstance().getOdometryPose().getRotation().plus(angularVelocity));
        }

        ChassisSpeeds adjustedSpeeds = desiredSpeeds;

        double timestamp = HALUtil.getFPGATime()/ 1.0e6;

        if (isChassisSpeedsDiscretizeEnabled && lastTimestamp > 0.0) {
            double dt = timestamp-lastTimestamp;

            adjustedSpeeds = ChassisSpeeds.discretize(
                desiredSpeeds.vxMetersPerSecond, 
                desiredSpeeds.vyMetersPerSecond,
                desiredSpeeds.omegaRadiansPerSecond,
                dt);
        }

        lastTimestamp = timestamp;

        if (isRotationLockEnabled) { // get the curr rot from robotstate and the argument from here
            double rotatationalVelocity = MathUtil.clamp(
                rotationalVelocityFeedbackController.calculate(
                    RobotState.getInstance().getOdometryPose().getRotation().getRadians(),
                    this.rotationLock.getRadians()), 
                -config.getSwerveDrivetrainControllerConfig().kROTATIONAL_POSITION_MAX_OUTPUT_RAD_SEC, 
                config.getSwerveDrivetrainControllerConfig().kROTATIONAL_POSITION_MAX_OUTPUT_RAD_SEC);

            ChassisSpeeds fieldRel = ChassisSpeeds.fromRobotRelativeSpeeds(adjustedSpeeds, RobotState.getInstance().getOdometryPose().getRotation());
            fieldRel.omegaRadiansPerSecond = rotatationalVelocity;
            adjustedSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRel, RobotState.getInstance().getOdometryPose().getRotation());
        }

        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(adjustedSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
            setpointStates, 
            adjustedSpeeds, 
            4, 
            4, 
            210); // TODO: Grabage values + create constants
        
            SwerveModuleState[] optimizedSetpoints = new SwerveModuleState[4];
            for (int i = 0; i < 4; i++) {
                optimizedSetpoints[i] = modules[i].setTargetState(setpointStates[i]); // setTargetState's helper method is kind of funny
            }

            // TODO: add logging statements
    }

    public double gyroAngularVelocity() {
        return gyroInputs.angularVelocityRadPerSec;
    }

    public void setRotationLock() {
        isRotationLockEnabled = true;
        this.rotationLock = rotationLock;
    }

    public void disableRotationLock() {
        isRotationLockEnabled = false;
    }

    public void setSlowdownCoeffs(double transCoeff, double rotCoeff) {
        this.translationCoeffecient = transCoeff;
        this.rotationCoefficient = rotCoeff;
        isTranslationSlowdownEnabled = true;
        isRotationSlowdownEnabled = true;
    }

    public void diableSlowdownCoeffs() {
        isTranslationSlowdownEnabled = false;
        isRotationSlowdownEnabled = false;
    }
}

package frc.robot.subsystems.drivetrain.swerve;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
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
    private final PIDController rotationalVelocityFeedbackController;

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
    private double translationCoefficient = 0.5;
    private double rotationCoefficient = 0.5;

    SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
    SwerveModuleState[] moduleStates = { // has to be set to a value so not null
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
    };

    private final SwerveSetpointGenerator swerveSetpointGenerator;
    private SwerveSetpoint previousSetpoint = new SwerveSetpoint(
        new ChassisSpeeds(), 
        moduleStates, 
        DriveFeedforwards.zeros(4)
    );

    double previousSetpointCallTime = Timer.getFPGATimestamp();

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

        // driveFFController = new DriveFFController(config);
        swerveSetpointGenerator = new SwerveSetpointGenerator(
            config.getPathplannerRobotConfig(), 
            Units.rotationsToRadians(
                config.getSharedGeneralConfig().kSTEER_MOTION_MAGIC_CRUISE_VELOCITY_ROTATIONS_PER_SEC)
        );
        
        rotationalVelocityFeedbackController = config.getSwerveDrivetrainControllerConfig().kROTATIONAL_VELOCITY_FEEDBACK_CONTROLLER;
        // translationalVelocityFeedbackController = config.getSwerveDrivetrainControllerConfig().kTRANSLATION_VELOCITY_FEEDBACK_CONTROLLER;
        // rotationalPositionFeedbackController = config.getSwerveDrivetrainControllerConfig().kROTATIONAL_POSITION_FEEDBACK_CONTROLLER;
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

        for (int i = 0; i < 4; i++) {
            modulePositions[i] = new SwerveModulePosition(moduleInputs[i].drivePositionMeters, moduleInputs[i].steerPosition);
            moduleStates[i] = new SwerveModuleState(moduleInputs[i].driveVelocityMetersPerSec, moduleInputs[i].steerPosition);
        }

        RobotState.getInstance()
            .addOdometryObservation(
                new RobotState.OdometryObservation(
                    modulePositions,
                    moduleStates,
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
        Logger.recordOutput("SwerveDrive/desiredSpeeds", desiredSpeeds);

        if (isTranslationSlowdownEnabled) {
            desiredSpeeds.vxMetersPerSecond *= this.translationCoefficient;
            desiredSpeeds.vyMetersPerSecond *= this.translationCoefficient;
        }

        if (isRotationSlowdownEnabled) {
            desiredSpeeds.omegaRadiansPerSecond *= this.rotationCoefficient;
        }

        var angularVelocity = new Rotation2d(gyroInputs.angularVelocityRadPerSec * config.getSwerveDrivetrainConfig().kROTATION_COMPENSATION_COEFFICIENT); // TODO: make a coeff
        if (angularVelocity.getRadians() != 0.0) {
            desiredSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds( // why should this be split into two?
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond,
                speeds.omegaRadiansPerSecond,
                RobotState.getInstance().getEstimatedPose().getRotation().plus(angularVelocity));
        }

        if (isRotationLockEnabled) { // get the curr rot from robotstate and the argument from here
            double rotationalVelocity = MathUtil.clamp(
                rotationalVelocityFeedbackController.calculate(
                    RobotState.getInstance().getEstimatedPose().getRotation().getRadians(),
                    this.rotationLock.getRadians()), 
                -config.getSwerveDrivetrainControllerConfig().kROTATIONAL_POSITION_MAX_OUTPUT_RAD_SEC, 
                config.getSwerveDrivetrainControllerConfig().kROTATIONAL_POSITION_MAX_OUTPUT_RAD_SEC);

            desiredSpeeds.omegaRadiansPerSecond = rotationalVelocity;
        }

        Logger.recordOutput("SwerveDrive/lockedRotationLock", desiredSpeeds);
        
        SwerveSetpoint swerveSetpoint = swerveSetpointGenerator.generateSetpoint(
            previousSetpoint,
            desiredSpeeds,
            Timer.getFPGATimestamp() - previousSetpointCallTime // between calls of generate setpoint
        );
        previousSetpointCallTime = Timer.getFPGATimestamp();
        previousSetpoint = swerveSetpoint;

        Logger.recordOutput("SwerveDrive/SetpointDT", previousSetpointCallTime);

        Logger.recordOutput("SwerveDrive/generatedRobotRelativeSpeeds", swerveSetpoint.robotRelativeSpeeds());

        SwerveModuleState[] optimizedSetpoints = swerveSetpoint.moduleStates();
        for (int i = 0; i < 4; i++) {
            optimizedSetpoints[i] = modules[i].setTargetState(optimizedSetpoints[i]); // setTargetState's helper method is kind of funny
        }

        Logger.recordOutput("SwerveDrive/optimizedModuleStates", optimizedSetpoints);

    }

    public double gyroAngularVelocity() {
        return gyroInputs.angularVelocityRadPerSec;
    }

    public void setRotationLock() {
        isRotationLockEnabled = true;
    }

    public void disableRotationLock() {
        isRotationLockEnabled = false;
    }

    public void setSlowdownCoeffs(double transCoeff, double rotCoeff) {
        this.translationCoefficient = transCoeff;
        this.rotationCoefficient = rotCoeff;
        isTranslationSlowdownEnabled = true;
        isRotationSlowdownEnabled = true;
    }

    public void disableSlowdownCoeffs() {
        isTranslationSlowdownEnabled = false;
        isRotationSlowdownEnabled = false;
    }
}

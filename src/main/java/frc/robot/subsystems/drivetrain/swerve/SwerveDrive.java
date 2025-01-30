package frc.robot.subsystems.drivetrain.swerve;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.constants.Constants;
import frc.robot.constants.swerve.controllerConfigs.SwerveControllerConfigBase;
import frc.robot.constants.swerve.controllerConfigs.SwerveControllerConfigComp;
import frc.robot.constants.swerve.controllerConfigs.SwerveControllerConfigProto;
import frc.robot.constants.swerve.controllerConfigs.SwerveControllerConfigSim;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigBase;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigComp;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigProto;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigSim;
import frc.robot.constants.swerve.moduleConfigs.SwerveModuleGeneralConfigBase;
import frc.robot.constants.swerve.moduleConfigs.comp.SwerveModuleGeneralConfigComp;
import frc.robot.constants.swerve.moduleConfigs.comp.SwerveModuleSpecificBLConfigComp;
import frc.robot.constants.swerve.moduleConfigs.comp.SwerveModuleSpecificBRConfigComp;
import frc.robot.constants.swerve.moduleConfigs.comp.SwerveModuleSpecificFLConfigComp;
import frc.robot.constants.swerve.moduleConfigs.comp.SwerveModuleSpecificFRConfigComp;
import frc.robot.constants.swerve.moduleConfigs.proto.SwerveModuleGeneralConfigProto;
import frc.robot.constants.swerve.moduleConfigs.proto.SwerveModuleSpecificBLConfigProto;
import frc.robot.constants.swerve.moduleConfigs.proto.SwerveModuleSpecificBRConfigProto;
import frc.robot.constants.swerve.moduleConfigs.proto.SwerveModuleSpecificFLConfigProto;
import frc.robot.constants.swerve.moduleConfigs.proto.SwerveModuleSpecificFRConfigProto;
import frc.robot.constants.swerve.moduleConfigs.sim.SwerveModuleGeneralConfigSim;
import frc.robot.constants.swerve.pathplannerConfigs.SwervePathplannerConfigBase;
import frc.robot.constants.swerve.pathplannerConfigs.SwervePathplannerConfigComp;
import frc.robot.constants.swerve.pathplannerConfigs.SwervePathplannerConfigProto;
import frc.robot.constants.swerve.pathplannerConfigs.SwervePathplannerConfigSim;
import frc.robot.subsystems.drivetrain.swerve.gyro.GyroIO;
import frc.robot.subsystems.drivetrain.swerve.gyro.GyroIOInputsAutoLogged;
import frc.robot.subsystems.drivetrain.swerve.gyro.GyroIONavX;
import frc.robot.subsystems.drivetrain.swerve.gyro.GyroIOPigeon2;
import frc.robot.subsystems.drivetrain.swerve.module.ModuleIO;
import frc.robot.subsystems.drivetrain.swerve.module.ModuleIOInputsAutoLogged;
import frc.robot.subsystems.drivetrain.swerve.module.ModuleIOSim;
import frc.robot.subsystems.drivetrain.swerve.module.ModuleIOTalonFX;

public class SwerveDrive extends SubsystemBase {
    private static SwerveDrive instance = null;
    public static SwerveDrive getInstance() {
        if (instance == null) {
            instance = new SwerveDrive();
        }

        return instance;
    }

    private final PIDController rotationalVelocityFeedbackController;

    private Rotation2d rotationLock;

    private ModuleIO[] modules;

    private ModuleIOInputsAutoLogged[] moduleInputs = {
            new ModuleIOInputsAutoLogged(),
            new ModuleIOInputsAutoLogged(),
            new ModuleIOInputsAutoLogged(),
            new ModuleIOInputsAutoLogged()
    };

    private final GyroIO gyroIO;
    private GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private boolean isTranslationSlowdownEnabled = false;
    private boolean isRotationSlowdownEnabled = false;
    private boolean isRotationLockEnabled = false;

    private double translationCoefficient = 0.5;
    private double rotationCoefficient = 0.5;

    private SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
    private SwerveModuleState[] moduleStates = { // has to be set to a value so not null
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
    };

    private final SwerveSetpointGenerator swerveSetpointGenerator;
    private SwerveSetpoint previousSetpoint = new SwerveSetpoint(
        RobotState.getInstance().getRobotRelativeSpeeds(), 
        moduleStates, 
        DriveFeedforwards.zeros(4)
    );

    double previousSetpointCallTime = Timer.getFPGATimestamp();

    private final SwerveModuleGeneralConfigBase moduleGeneralConfig;
    private final SwerveDrivetrainConfigBase drivetrainConfig;
    private final SwervePathplannerConfigBase pathplannerConfig;
    private final SwerveControllerConfigBase controllerConfig; 

    @SuppressWarnings("static-access")
    private SwerveDrive() {
        switch (Constants.currentMode) {
            case COMP:
                drivetrainConfig = SwerveDrivetrainConfigComp.getInstance();
                pathplannerConfig = SwervePathplannerConfigComp.getInstance();
                moduleGeneralConfig = SwerveModuleGeneralConfigComp.getInstance();
                controllerConfig = SwerveControllerConfigComp.getInstance();

                modules = new ModuleIO[] {
                    new ModuleIOTalonFX(moduleGeneralConfig, SwerveModuleSpecificFLConfigComp.getInstance(), 0),
                    new ModuleIOTalonFX(moduleGeneralConfig, SwerveModuleSpecificFRConfigComp.getInstance(), 1),
                    new ModuleIOTalonFX(moduleGeneralConfig, SwerveModuleSpecificBLConfigComp.getInstance(), 2),
                    new ModuleIOTalonFX(moduleGeneralConfig, SwerveModuleSpecificBRConfigComp.getInstance(), 3)
                };
                
                gyroIO = new GyroIOPigeon2();
                Phoenix6Odometry.getInstance().start();

                break;

            case PROTO:
                drivetrainConfig = SwerveDrivetrainConfigProto.getInstance();
                pathplannerConfig = SwervePathplannerConfigProto.getInstance();
                moduleGeneralConfig = SwerveModuleGeneralConfigProto.getInstance();
                controllerConfig = SwerveControllerConfigProto.getInstance();

                modules = new ModuleIO[] {
                    new ModuleIOTalonFX(moduleGeneralConfig, SwerveModuleSpecificFLConfigProto.getInstance(), 0),
                    new ModuleIOTalonFX(moduleGeneralConfig, SwerveModuleSpecificFRConfigProto.getInstance(), 1),
                    new ModuleIOTalonFX(moduleGeneralConfig, SwerveModuleSpecificBLConfigProto.getInstance(), 2),
                    new ModuleIOTalonFX(moduleGeneralConfig, SwerveModuleSpecificBRConfigProto.getInstance(), 3)
                };
                
                gyroIO = new GyroIONavX();
                Phoenix6Odometry.getInstance().start();
                break;

            case SIM:
                drivetrainConfig = SwerveDrivetrainConfigSim.getInstance();
                pathplannerConfig = SwervePathplannerConfigSim.getInstance();
                moduleGeneralConfig = SwerveModuleGeneralConfigSim.getInstance();
                controllerConfig = SwerveControllerConfigSim.getInstance();

                modules = new ModuleIO[] {
                    new ModuleIOSim(moduleGeneralConfig, 0),
                    new ModuleIOSim(moduleGeneralConfig, 1),
                    new ModuleIOSim(moduleGeneralConfig, 2),
                    new ModuleIOSim(moduleGeneralConfig, 3)
                };

                gyroIO = new GyroIO() {};
                break;

            case REPLAY:
                drivetrainConfig = SwerveDrivetrainConfigComp.getInstance();
                pathplannerConfig = SwervePathplannerConfigComp.getInstance();
                moduleGeneralConfig = SwerveModuleGeneralConfigComp.getInstance();
                controllerConfig = SwerveControllerConfigComp.getInstance();

                modules = new ModuleIO[] {
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {}
                };

                gyroIO = new GyroIOPigeon2();

                break;

            default:
                drivetrainConfig = SwerveDrivetrainConfigComp.getInstance();
                pathplannerConfig = SwervePathplannerConfigComp.getInstance();
                moduleGeneralConfig = SwerveModuleGeneralConfigComp.getInstance();
                controllerConfig = SwerveControllerConfigComp.getInstance();

                modules = new ModuleIO[] {
                    new ModuleIOTalonFX(moduleGeneralConfig, SwerveModuleSpecificFLConfigComp.getInstance(), 0),
                    new ModuleIOTalonFX(moduleGeneralConfig, SwerveModuleSpecificFRConfigComp.getInstance(), 1),
                    new ModuleIOTalonFX(moduleGeneralConfig, SwerveModuleSpecificBLConfigComp.getInstance(), 2),
                    new ModuleIOTalonFX(moduleGeneralConfig, SwerveModuleSpecificBRConfigComp.getInstance(), 3)
                };
                
                gyroIO = new GyroIOPigeon2();
                Phoenix6Odometry.getInstance().start();

                break;
                
        }

        // driveFFController = new DriveFFController(config);
        swerveSetpointGenerator = new SwerveSetpointGenerator(
            pathplannerConfig.getRobotConfig(), 
            Units.rotationsToRadians(
                moduleGeneralConfig.getSteerMotionMagicCruiseVelocityRotationsPerSec())
        );
                
        rotationalVelocityFeedbackController = controllerConfig.getRotationalPositionFeedbackController();
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

        Logger.recordOutput("SwerveDrive/measuredModuleStates", moduleStates);
        Logger.recordOutput("SwerveDrive/measuredModulePositions", modulePositions);

        RobotState.getInstance()
            .addOdometryObservation(
                new RobotState.OdometryObservation(
                    modulePositions.clone(),
                    moduleStates.clone(),
                    gyroInputs.isConnected ? 
                        new Rotation2d(gyroInputs.orientation.getZ()) :
                        null,
                    odometryTimestamp
                    ));
    }

    private ChassisSpeeds compensateRobotRelativeSpeeds(ChassisSpeeds speeds) {
        Rotation2d angularVelocity = new Rotation2d(gyroInputs.angularVelocityRadPerSec * drivetrainConfig.getRotationCompensationCoefficient());
        if (angularVelocity.getRadians() != 0.0) {
            speeds = ChassisSpeeds.fromRobotRelativeSpeeds( // why should this be split into two?
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond,
                speeds.omegaRadiansPerSecond,
                RobotState.getInstance().getEstimatedPose().getRotation().plus(angularVelocity));
        }

        return ChassisSpeeds.fromFieldRelativeSpeeds(speeds, RobotState.getInstance().getEstimatedPose().getRotation());
    }
    
    public void driveRobotRelative(ChassisSpeeds desiredSpeeds) {
        Logger.recordOutput("SwerveDrive/desiredSpeeds", desiredSpeeds);

        if (isTranslationSlowdownEnabled) {
            desiredSpeeds.vxMetersPerSecond *= this.translationCoefficient;
            desiredSpeeds.vyMetersPerSecond *= this.translationCoefficient;
        }

        if (isRotationSlowdownEnabled) {
            desiredSpeeds.omegaRadiansPerSecond *= this.rotationCoefficient;
        }

        desiredSpeeds = compensateRobotRelativeSpeeds(desiredSpeeds);

        if (isRotationLockEnabled) { // get the curr rot from robotstate and the argument from here
            double rotationalVelocity = MathUtil.clamp(
                rotationalVelocityFeedbackController.calculate(
                    RobotState.getInstance().getEstimatedPose().getRotation().getRadians(),
                    this.rotationLock.getRadians()), 
                -controllerConfig.getRotationalPositionMaxOutputRadSec(), 
                controllerConfig.getRotationalPositionMaxOutputRadSec());

            desiredSpeeds.omegaRadiansPerSecond = rotationalVelocity;
        }

        Logger.recordOutput("SwerveDrive/lockedRotationLock", desiredSpeeds);
        
        double dt = Timer.getFPGATimestamp() - previousSetpointCallTime; 
        Logger.recordOutput("SwerveDrive/dt", dt);
        previousSetpoint = swerveSetpointGenerator.generateSetpoint(
            previousSetpoint,
            desiredSpeeds,
            dt // between calls of generate setpoint
        );
        previousSetpointCallTime = Timer.getFPGATimestamp();
        Logger.recordOutput("SwerveDrive/SetpointDT", previousSetpointCallTime);
        Logger.recordOutput("SwerveDrive/generatedRobotRelativeSpeeds", previousSetpoint.robotRelativeSpeeds());

        SwerveModuleState[] optimizedSetpoints = previousSetpoint.moduleStates();
        for (int i = 0; i < 4; i++) {
            optimizedSetpoints[i].optimize(moduleStates[i].angle);
            modules[i].setState(optimizedSetpoints[i]); // setTargetState's helper method is kind of funny
        }

        Logger.recordOutput("SwerveDrive/optimizedModuleStates", optimizedSetpoints);
    }

    public void driveFieldRelative(ChassisSpeeds speeds) {
        Logger.recordOutput("SwerveDrive/driveFieldRelativeSpeeds", speeds);

        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, RobotState.getInstance().getEstimatedPose().getRotation());
        driveRobotRelative(speeds);
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

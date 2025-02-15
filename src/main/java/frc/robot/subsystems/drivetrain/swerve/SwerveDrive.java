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
import frc.robot.constants.swerve.moduleConfigs.proto.SwerveModuleSpecificFRConfigProto;
import frc.robot.constants.swerve.moduleConfigs.sim.SwerveModuleGeneralConfigSim;
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

    private double translationCoefficient = 1;
    private double rotationCoefficient = 1;

    private SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
    private SwerveModuleState[] moduleStates = { // has to be set to a value so not null
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
    };
    private SwerveModuleState[] moduleAccelerations = new SwerveModuleState[4];

    private final SwerveSetpointGenerator swerveSetpointGenerator;
    private SwerveSetpoint previousSetpoint = new SwerveSetpoint(
        RobotState.getInstance().getRobotRelativeSpeeds(), 
        moduleStates, 
        DriveFeedforwards.zeros(4)
    );

    private ChassisSpeeds desiredRobotRelativeSpeeds = new ChassisSpeeds();

    double prevLoopTime = Timer.getTimestamp();

    private final SwerveModuleGeneralConfigBase moduleGeneralConfig;
    private final SwerveDrivetrainConfigBase drivetrainConfig;
    private final SwerveControllerConfigBase controllerConfig; 

    private SwerveDrive() {
        switch (Constants.currentMode) {
            case COMP:
                drivetrainConfig = SwerveDrivetrainConfigComp.getInstance();
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
                moduleGeneralConfig = SwerveModuleGeneralConfigProto.getInstance();
                controllerConfig = SwerveControllerConfigProto.getInstance();

                modules = new ModuleIO[] {
                    new ModuleIOTalonFX(moduleGeneralConfig, SwerveModuleSpecificBLConfigProto.getInstance(), 0),
                    new ModuleIOTalonFX(moduleGeneralConfig, SwerveModuleSpecificFRConfigProto.getInstance(), 1),
                    new ModuleIOTalonFX(moduleGeneralConfig, SwerveModuleSpecificBLConfigProto.getInstance(), 2),
                    new ModuleIOTalonFX(moduleGeneralConfig, SwerveModuleSpecificBLConfigProto.getInstance(), 3)
                };
                
                gyroIO = new GyroIONavX();
                Phoenix6Odometry.getInstance().start();
                break;

            case SIM:
                drivetrainConfig = SwerveDrivetrainConfigSim.getInstance();
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
            drivetrainConfig.getRobotConfig(), 
            Units.rotationsToRadians(
                moduleGeneralConfig.getSteerMotionMagicCruiseVelocityRotationsPerSec())
        );
                
        rotationalVelocityFeedbackController = controllerConfig.getRotationalPositionFeedbackController();
    }

    @Override
    public void periodic() {
        double dt = Timer.getTimestamp() - prevLoopTime; 
        prevLoopTime = Timer.getTimestamp();

        double odometryTimestamp = 0.0;
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
            odometryTimestamp = Timer.getFPGATimestamp();
        }

        for (int i = 0; i < 4; i++) {
            modulePositions[i] = new SwerveModulePosition(moduleInputs[i].drivePositionMeters, moduleInputs[i].steerPosition);
            moduleStates[i] = new SwerveModuleState(moduleInputs[i].driveVelocityMetersPerSec, moduleInputs[i].steerPosition);
            moduleAccelerations[i] = new SwerveModuleState(moduleInputs[i].driveAccelerationMetersPerSecSec, moduleInputs[i].steerPosition);
        }

        Logger.recordOutput("SwerveDrive/measuredModuleStates", moduleStates);
        Logger.recordOutput("SwerveDrive/measuredModulePositions", modulePositions);

        RobotState.getInstance()
            .addOdometryObservation(
                new RobotState.OdometryObservation(
                    modulePositions.clone(),
                    moduleStates.clone(),
                    moduleAccelerations.clone(),
                    gyroInputs.isConnected ? 
                        gyroInputs.orientation :
                        null,
                    gyroInputs.isConnected ? 
                        gyroInputs.rates :
                        null,
                    gyroInputs.isConnected ? 
                        gyroInputs.fieldRelativeAccelerationMetersPerSecSec :
                        null, 
                    odometryTimestamp
                )
            );
        

        // TODO: WANT TO UPDATE MODULES IN PERIODIC IN ORDER TO TO ENSURE THAT COMMANDS DO NOT DOUBLE SCHEDULE SWERVE (EDGE CASE)
        if (isTranslationSlowdownEnabled) {
            desiredRobotRelativeSpeeds.vxMetersPerSecond *= this.translationCoefficient;
            desiredRobotRelativeSpeeds.vyMetersPerSecond *= this.translationCoefficient;
        }

        if (isRotationSlowdownEnabled) {
            desiredRobotRelativeSpeeds.omegaRadiansPerSecond *= this.rotationCoefficient;
        }

        desiredRobotRelativeSpeeds = compensateRobotRelativeSpeeds(desiredRobotRelativeSpeeds);
        Logger.recordOutput("SwerveDrive/compensatedRobotRelativeSpeeds", previousSetpoint.robotRelativeSpeeds());

        if (isRotationLockEnabled) { // get the curr rot from robot state and the argument from here
            double rotationalVelocity = MathUtil.clamp(
                rotationalVelocityFeedbackController.calculate(
                    RobotState.getInstance().getEstimatedPose().getRotation().getRadians(),
                    this.rotationLock.getRadians()), 
                -controllerConfig.getRotationalPositionMaxOutputRadSec(), 
                controllerConfig.getRotationalPositionMaxOutputRadSec());

            desiredRobotRelativeSpeeds.omegaRadiansPerSecond = rotationalVelocity;
        }

        Logger.recordOutput("SwerveDrive/lockedRotationLock", desiredRobotRelativeSpeeds);
        
        Logger.recordOutput("SwerveDrive/dt", dt);
        
        previousSetpoint = swerveSetpointGenerator.generateSetpoint(
            previousSetpoint,
            desiredRobotRelativeSpeeds,
            dt // between calls of generate setpoint
        );
        
        Logger.recordOutput("SwerveDrive/generatedRobotRelativeSpeeds", previousSetpoint.robotRelativeSpeeds());

        SwerveModuleState[] optimizedSetpoints = previousSetpoint.moduleStates();
        for (int i = 0; i < 4; i++) {
            optimizedSetpoints[i].optimize(moduleStates[i].angle);
            modules[i].setState(optimizedSetpoints[i]); // setTargetState's helper method is kind of funny
        }

        Logger.recordOutput("SwerveDrive/optimizedModuleStates", optimizedSetpoints);
    }

    private ChassisSpeeds compensateRobotRelativeSpeeds(ChassisSpeeds speeds) {
        Rotation2d angularVelocity = new Rotation2d(speeds.omegaRadiansPerSecond * drivetrainConfig.getRotationCompensationCoefficient());
        if (angularVelocity.getRadians() != 0.0) {
            speeds =  ChassisSpeeds.fromFieldRelativeSpeeds(
                ChassisSpeeds.fromRobotRelativeSpeeds( // why should this be split into two?
                    speeds.vxMetersPerSecond,
                    speeds.vyMetersPerSecond,
                    speeds.omegaRadiansPerSecond,
                    RobotState.getInstance().getEstimatedPose().getRotation().plus(angularVelocity)
                ),
                RobotState.getInstance().getEstimatedPose().getRotation()
            );
        }

        return speeds;
    }
    
    public void driveRobotRelative(ChassisSpeeds speeds) {
        Logger.recordOutput("SwerveDrive/desiredRobotRelativeSpeeds", speeds);

        desiredRobotRelativeSpeeds = speeds;
    }

    public void driveFieldRelative(ChassisSpeeds speeds) {
        Logger.recordOutput("SwerveDrive/driveFieldRelativeSpeeds", speeds);

        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, RobotState.getInstance().getEstimatedPose().getRotation());
        driveRobotRelative(speeds);
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

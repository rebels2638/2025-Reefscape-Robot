package frc.robot.subsystems.drivetrain.swerve;

import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    private double lastTimestamp = 0;

    // kinematics and module states
    private SwerveDriveKinematics kinematics;

    private SwerveModulePosition[] currentMeasuredModulePositions = {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
    };

    private SwerveModulePosition[] previousMeasuredModulePositions = {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
    };

    private SwerveModuleState[] measuredModuleStates = {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
    };

    // pose estimation
    private Rotation2d yaw = new Rotation2d(0);
    private SwerveDrivePoseEstimator poseEstimator;

    // Chassis speeds
    private ChassisSpeeds measuredRobotRelativeSpeeds = new ChassisSpeeds(0, 0, 0);
    private ChassisSpeeds measuredFieldRelativeSpeeds = new ChassisSpeeds(0, 0, 0);

    private ChassisSpeeds desiredRobotRelativeSpeeds = new ChassisSpeeds(0, 0, 0);
    private ChassisSpeeds desiredFieldRelativeSpeeds = new ChassisSpeeds(0, 0, 0);
    private ChassisSpeeds previouslyDesiredFieldRelativeSpeeds = new ChassisSpeeds(0, 0, 0);

    // Feedback and feedforward controllers
    private final DriveFFController driveFFController;
    private final PIDController rotationalVelocityFeedbackController;
    private final PIDController translationalVelocityFeedbackController;
    private final PIDController rotationalPositionFeedbackController;

    private final SwerveSetpointGenerator swerveSetpointGenerator;
    private SwerveSetpoint previousSetpoint = new SwerveSetpoint(
        new ChassisSpeeds(), 
        measuredModuleStates, 
        DriveFeedforwards.zeros(4)
    );

    // IO
    private ModuleIO[] modules;

    private ModuleIOInputsAutoLogged[] moduleInputs = {
            new ModuleIOInputsAutoLogged(),
            new ModuleIOInputsAutoLogged(),
            new ModuleIOInputsAutoLogged(),
            new ModuleIOInputsAutoLogged()
    };

    private final GyroIO gyroIO;
    private GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private final SwerveConfigBase config;

    private double initialRotationLockRad = 0;
    private boolean isRotationLocked = false;

    @SuppressWarnings("static-access")
    public SwerveDrive() {
        // IO
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

        kinematics = new SwerveDriveKinematics(
                config.getSwerveDrivetrainConfig().kFRONT_LEFT_POSITION_METERS,
                config.getSwerveDrivetrainConfig().kFRONT_RIGHT_POSITION_METERS,
                config.getSwerveDrivetrainConfig().kBACK_LEFT_POSITION_METERS,
                config.getSwerveDrivetrainConfig().kBACK_RIGHT_POSITION_METERS);

        // initialize pose estimator
        poseEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                yaw,
                new SwerveModulePosition[] {
                        new SwerveModulePosition(0, new Rotation2d()),
                        new SwerveModulePosition(0, new Rotation2d()),
                        new SwerveModulePosition(0, new Rotation2d()),
                        new SwerveModulePosition(0, new Rotation2d())
                },
                new Pose2d());

        
        driveFFController = new DriveFFController(config);
        swerveSetpointGenerator = new SwerveSetpointGenerator(
            config.getPathplannerRobotConfig(), 
            Units.rotationsToRadians(
                config.getSharedGeneralConfig().kSTEER_MOTION_MAGIC_CRUISE_VELOCITY_ROTATIONS_PER_SEC)
        );

        rotationalVelocityFeedbackController = config.getSwerveDrivetrainControllerConfig().kROTATIONAL_VELOCITY_FEEDBACK_CONTROLLER;
        translationalVelocityFeedbackController = config.getSwerveDrivetrainControllerConfig().kTRANSLATION_VELOCITY_FEEDBACK_CONTROLLER;
        rotationalPositionFeedbackController = config.getSwerveDrivetrainControllerConfig().kROTATIONAL_POSITION_FEEDBACK_CONTROLLER;
    }

    @Override
    public void periodic() {
        double dt = Timer.getFPGATimestamp() - lastTimestamp;

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

        // update module positions
        previousMeasuredModulePositions = currentMeasuredModulePositions;
        currentMeasuredModulePositions = new SwerveModulePosition[] {
                new SwerveModulePosition(moduleInputs[0].drivePositionMeters, moduleInputs[0].steerPosition),
                new SwerveModulePosition(moduleInputs[1].drivePositionMeters, moduleInputs[1].steerPosition),
                new SwerveModulePosition(moduleInputs[2].drivePositionMeters, moduleInputs[2].steerPosition),
                new SwerveModulePosition(moduleInputs[3].drivePositionMeters, moduleInputs[3].steerPosition)
        };

        // update the yaw of the robot
        if (gyroInputs.isConnected) {
            yaw = new Rotation2d(gyroInputs.orientation.getZ());
        } else {
            Twist2d twist = kinematics.toTwist2d(previousMeasuredModulePositions, currentMeasuredModulePositions);
            yaw = yaw.plus(new Rotation2d(twist.dtheta));
        }

        // add a odometry observation to the pose estimator
        poseEstimator.updateWithTime(odometryTimestamp, yaw, currentMeasuredModulePositions);

        // update field and robot relative measured speeds
        measuredModuleStates = new SwerveModuleState[] {
                new SwerveModuleState(moduleInputs[0].driveVelocityMetersPerSec, moduleInputs[0].steerPosition),
                new SwerveModuleState(moduleInputs[1].driveVelocityMetersPerSec, moduleInputs[1].steerPosition),
                new SwerveModuleState(moduleInputs[2].driveVelocityMetersPerSec, moduleInputs[2].steerPosition),
                new SwerveModuleState(moduleInputs[3].driveVelocityMetersPerSec, moduleInputs[3].steerPosition)
        };

        Logger.recordOutput("SwerveDrive/measuredModuleStates", measuredModuleStates);

        measuredRobotRelativeSpeeds = kinematics.toChassisSpeeds(measuredModuleStates);
        measuredFieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(measuredRobotRelativeSpeeds, yaw);

        Logger.recordOutput("SwerveDrive/estimatedYaw", yaw.getRadians());
        Logger.recordOutput("SwerveDrive/estimatedPose", getPose());

        double estimatedVyDriftMetersPerSecond = driveFFController.calculate(
                desiredFieldRelativeSpeeds.vxMetersPerSecond, desiredFieldRelativeSpeeds.omegaRadiansPerSecond);
        double estimatedVxDriftMetersPerSecond = driveFFController.calculate(
                desiredFieldRelativeSpeeds.vyMetersPerSecond, desiredFieldRelativeSpeeds.omegaRadiansPerSecond);
        Logger.recordOutput("SwerveDrive/estimatedVxDriftMetersPerSecond", estimatedVxDriftMetersPerSecond);
        Logger.recordOutput("SwerveDrive/estimatedVyDriftMetersPerSecond", estimatedVyDriftMetersPerSecond);

        ChassisSpeeds correctedSpeeds = desiredFieldRelativeSpeeds;
        correctedSpeeds.vxMetersPerSecond = correctedSpeeds.vxMetersPerSecond +
                estimatedVxDriftMetersPerSecond /*+
                translationalVelocityFeedbackController.calculate(
                        measuredFieldRelativeSpeeds.vxMetersPerSecond,
                        previouslyDesiredFieldRelativeSpeeds.vxMetersPerSecond // correct for past error
                )*/;

        correctedSpeeds.vyMetersPerSecond = correctedSpeeds.vyMetersPerSecond -
                estimatedVyDriftMetersPerSecond/*  +
                translationalVelocityFeedbackController.calculate(
                        measuredFieldRelativeSpeeds.vyMetersPerSecond,
                        previouslyDesiredFieldRelativeSpeeds.vyMetersPerSecond // correct for past error
                )*/;

        // lock the rotation of the drivetrain when no rotational input is given
        if (desiredFieldRelativeSpeeds.omegaRadiansPerSecond == 0 && !isRotationLocked) {
            initialRotationLockRad = yaw.getRadians();
            isRotationLocked = true;
        } else if (desiredFieldRelativeSpeeds.omegaRadiansPerSecond != 0) {
            isRotationLocked = false;
        }

        if (isRotationLocked) {
            correctedSpeeds.omegaRadiansPerSecond = rotationalPositionFeedbackController.calculate(
                    yaw.getRadians(),
                    initialRotationLockRad);
        } else {
            correctedSpeeds.omegaRadiansPerSecond = correctedSpeeds.omegaRadiansPerSecond +
                    rotationalPositionFeedbackController.calculate(
                            measuredFieldRelativeSpeeds.omegaRadiansPerSecond,
                            previouslyDesiredFieldRelativeSpeeds.omegaRadiansPerSecond // correct for past error
                    );
        }

        correctedSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(correctedSpeeds, yaw);

        // SwerveSetpoint swerveSetpoint = swerveSetpointGenerator.generateSetpoint(
        //     previousSetpoint,
        //     correctedSpeeds,
        //     dt // between calls of generate setpoint
        // );
        // previousSetpoint = swerveSetpoint;
        Logger.recordOutput("SwerveDrive/correctedSpeeds", correctedSpeeds);

        // set the desired module statesx
        // SwerveModuleState[] desiredModuleStates = swerveSetpoint.moduleStates();
        SwerveModuleState[] desiredModuleStates = kinematics.toSwerveModuleStates(correctedSpeeds);
        for (int i = 0; i < 4; i++) {
            desiredModuleStates[i].optimize(measuredModuleStates[i].angle);
            modules[i].setState(desiredModuleStates[i]);
        }
        Logger.recordOutput("SwerveDrive/desiredModuleStates", desiredModuleStates);

        lastTimestamp = Timer.getFPGATimestamp();
    }

    public void driveFieldRelative(ChassisSpeeds speeds) {
        previouslyDesiredFieldRelativeSpeeds = desiredFieldRelativeSpeeds;
        desiredFieldRelativeSpeeds = speeds;
        Logger.recordOutput("SwerveDrive/desiredFieldRelativeSpeeds", desiredFieldRelativeSpeeds);
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        driveFieldRelative(ChassisSpeeds.fromRobotRelativeSpeeds(speeds, yaw));
    }

    public void resetPose(Pose2d pose) {
        gyroIO.resetGyro(pose.getRotation());
        if (gyroInputs.isConnected) {
            yaw = new Rotation2d(gyroInputs.orientation.getMeasureZ().in(Radians));
        }
        poseEstimator.resetPosition(yaw, currentMeasuredModulePositions, pose);
        yaw = pose.getRotation(); // update the initial yaw of the robot
    }

    public void zeroGyro() {
        resetPose(new Pose2d(getPose().getTranslation(), new Rotation2d(0)));
    }

    public ChassisSpeeds getMeasuredRobotRelativeSpeeds() {
        return measuredRobotRelativeSpeeds;
    }

    public ChassisSpeeds getMeasuredFieldRelativeSpeeds() {
        return measuredFieldRelativeSpeeds;
    }

    // TODO: no need to lock stop odom thread?
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        return currentMeasuredModulePositions;
    }
}

package frc.robot.subsystems.drivetrain.swerve;

import org.littletonrobotics.junction.Logger;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.swerve.controllers.DriveFFController;
import frc.robot.subsystems.drivetrain.swerve.gyro.GyroIO;
import frc.robot.subsystems.drivetrain.swerve.gyro.GyroIOInputsAutoLogged;
import frc.robot.subsystems.drivetrain.swerve.gyro.GyroIOPigeon2;
import frc.robot.subsystems.drivetrain.swerve.module.ModuleIO;
import frc.robot.subsystems.drivetrain.swerve.module.ModuleIOInputsAutoLogged;
import frc.robot.subsystems.drivetrain.swerve.module.ModuleIOSim;
import frc.robot.subsystems.drivetrain.swerve.module.ModuleIOTalonFX;
import frc.robot.constants.*;

public class SwerveDrive extends SubsystemBase {
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
            case REAL:
                config = new SwerveRealConfig();

                modules[0] = new ModuleIOTalonFX(config, config.kFRONT_LEFT_CONFIG.kSPECIFIC_CONFIG);
                modules[1] = new ModuleIOTalonFX(config, config.kFRONT_RIGHT_CONFIG.kSPECIFIC_CONFIG);
                modules[2] = new ModuleIOTalonFX(config, config.kBACK_LEFT_CONFIG.kSPECIFIC_CONFIG);
                modules[3] = new ModuleIOTalonFX(config, config.kBACK_RIGHT_CONFIG.kSPECIFIC_CONFIG);

                gyroIO = new GyroIOPigeon2();

                Phoenix6Odometry.getInstance().start();

                break;

            case SIM:
                config = new SwerveSimConfig();

                modules[0] = new ModuleIOSim(config);
                modules[1] = new ModuleIOSim(config);
                modules[2] = new ModuleIOSim(config);
                modules[3] = new ModuleIOSim(config);

                gyroIO = new GyroIO() {
                };

                break;

            default:
                config = new SwerveRealConfig();

                modules[0] = new ModuleIOTalonFX(config, config.kFRONT_LEFT_CONFIG.kSPECIFIC_CONFIG);
                modules[1] = new ModuleIOTalonFX(config, config.kFRONT_RIGHT_CONFIG.kSPECIFIC_CONFIG);
                modules[2] = new ModuleIOTalonFX(config, config.kBACK_LEFT_CONFIG.kSPECIFIC_CONFIG);
                modules[3] = new ModuleIOTalonFX(config, config.kBACK_RIGHT_CONFIG.kSPECIFIC_CONFIG);

                gyroIO = new GyroIOPigeon2();

                break;
        }
        kinematics = new SwerveDriveKinematics(
                config.K_SWERVE_DRIVETRAIN_CONFIG.kFRONT_LEFT_POSITION_METERS,
                config.K_SWERVE_DRIVETRAIN_CONFIG.kFRONT_RIGHT_POSITION_METERS,
                config.K_SWERVE_DRIVETRAIN_CONFIG.kBACK_LEFT_POSITION_METERS,
                config.K_SWERVE_DRIVETRAIN_CONFIG.kBACK_RIGHT_POSITION_METERS);

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
        rotationalVelocityFeedbackController = config.K_SWERVE_DRIVETRAIN_CONTROLLER_CONFIG.kROTATIONAL_VELOCITY_FEEDBACK_CONTROLLER;
        translationalVelocityFeedbackController = config.K_SWERVE_DRIVETRAIN_CONTROLLER_CONFIG.kTRANSLATION_VELOCITY_FEEDBACK_CONTROLLER;
        rotationalPositionFeedbackController = config.K_SWERVE_DRIVETRAIN_CONTROLLER_CONFIG.kROTATIONAL_POSITION_FEEDBACK_CONTROLLER;
    }

    @Override
    public void periodic() {
        var odometryTimestamp = 0.0;
        // Thread safe reading of the gyro and swerve inputs.
        // The read lock is released only after inputs are written via the write lock
        Phoenix6Odometry.getInstance().stateLock.readLock().lock();
        try {
            gyroIO.updateInputs(gyroInputs);
            for (int i = 0; i < 4; i++) {
                modules[i].updateInputs(moduleInputs[i]);
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
        if (gyroInputs.connected) {
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
                estimatedVxDriftMetersPerSecond +
                translationalVelocityFeedbackController.calculate(
                        measuredFieldRelativeSpeeds.vxMetersPerSecond,
                        previouslyDesiredFieldRelativeSpeeds.vxMetersPerSecond // correct for past error
                );

        correctedSpeeds.vyMetersPerSecond = correctedSpeeds.vyMetersPerSecond -
                estimatedVyDriftMetersPerSecond +
                translationalVelocityFeedbackController.calculate(
                        measuredFieldRelativeSpeeds.vyMetersPerSecond,
                        previouslyDesiredFieldRelativeSpeeds.vyMetersPerSecond // correct for past error
                );

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

        correctedSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(correctedSpeeds, getPose().getRotation());
        Logger.recordOutput("SwerveDrive/correctedSpeeds", correctedSpeeds);

        // set the desired module states
        SwerveModuleState[] desiredModuleStates = kinematics.toSwerveModuleStates(correctedSpeeds);
        for (int i = 0; i < 4; i++) {
            modules[i].setState(desiredModuleStates[i]);
            Logger.recordOutput("SwerveDrive/unoptimizedDesiredModuleStates", desiredModuleStates);
        }
        Logger.recordOutput("SwerveDrive/desiredModuleStates", desiredModuleStates);
    }

    public void driveFieldRelative(ChassisSpeeds speeds) {
        previouslyDesiredFieldRelativeSpeeds = desiredFieldRelativeSpeeds;
        desiredFieldRelativeSpeeds = speeds;
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        driveFieldRelative(ChassisSpeeds.fromRobotRelativeSpeeds(speeds, yaw));
    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(yaw, currentMeasuredModulePositions, pose);
        yaw = pose.getRotation(); // update the initial yaw of the robot
    }

    public void zeroGyro() {
        resetPose(getPose());
    }

    public ChassisSpeeds getMeasuredRobotRelativeSpeeds() {
        return measuredRobotRelativeSpeeds;
    }

    public ChassisSpeeds getMeasuredFeildRelativeSpeeds() {
        return measuredFieldRelativeSpeeds;
    }

    // TODO: no need to lock stop odom thread?
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    // public Pose2d getPoseAtTimestamp(double time) {
    // double lowestError = Double.MAX_VALUE;
    // Pose2d pose = poseQueue.peek().getFirst();
    // Logger.recordOutput("SwerveDrive/queueLength", poseQueue.size());
    // for (Pair<Pose2d, Double> pair : poseQueue) {
    // double currentError = Math.abs(time - pair.getSecond().doubleValue());
    // if (currentError < lowestError) {
    // lowestError = time - pair.getSecond().doubleValue();
    // pose = pair.getFirst();
    // } else {
    // break;
    // }
    // }

    // return pose;
    // }

    public SwerveModulePosition[] getSwerveModulePositions() {
        return currentMeasuredModulePositions;
    }
}

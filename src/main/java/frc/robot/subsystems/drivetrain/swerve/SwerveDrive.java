package frc.robot.subsystems.drivetrain.swerve;

import java.util.LinkedList;
import java.util.Queue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.lib.util.RebelUtil;

public class SwerveDrive extends SubsystemBase {

    private SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
        Constants.DrivetrainConstants.kFRONT_LEFT_POSITION_METERS,
        Constants.DrivetrainConstants.kFRONT_RIGHT_POSITION_METERS,
        Constants.DrivetrainConstants.kBACK_LEFT_POSITION_METERS,
        Constants.DrivetrainConstants.kBACK_RIGHT_POSITION_METERS);

    private ModuleIO[] modules;
    private ModuleIOInputsAutoLogged[] moduleInputs = {
        new ModuleIOInputsAutoLogged(),
        new ModuleIOInputsAutoLogged(),
        new ModuleIOInputsAutoLogged(),
        new ModuleIOInputsAutoLogged()
    };

    private ChassisSpeeds desiredRobotRelativeSpeeds = new ChassisSpeeds(0,0,0);
    private ChassisSpeeds desiredFieldRelativeSpeeds = new ChassisSpeeds(0,0,0);

    private final GyroIO gyroIO;
    private GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private int syncCount = 0;

    private Rotation2d yaw = new Rotation2d(0);

    private SwerveDrivePoseEstimator m_poseEstimator;

    private final Notifier odometryThread;
    private final Lock odometryLock = new ReentrantLock();

    private ChassisSpeeds measuredRobotRelativeSpeeds = new ChassisSpeeds(0,0,0);
    private ChassisSpeeds measuredFieldRelativeSpeeds = new ChassisSpeeds(0,0,0);

    private PIDController m_angleFeedbackController = new PIDController(1, 0, 0);;
    private DriveFFController driveFFController = new DriveFFController();

    private PIDController m_translationalFeedbackController = new PIDController(1, 0, 0);

    private SwerveModuleState[] measuredModuleStates = {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
    };

    private SwerveModuleState[] previousDesiredStates = {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
    };

    private double prevTime = Timer.getFPGATimestamp();
    
    private Queue<Pair<Pose2d,Double>> poseQueue = new LinkedList<Pair<Pose2d,Double>>();

    private final SlewRateLimiter vxSlewRateLimiter = 
        new SlewRateLimiter(
            Constants.DrivetrainConstants.kMAX_DRIVETRAIN_TRANSLATIONAL_ACCELERATION_METERS_PER_SECOND_SQUARED, 
            -Constants.DrivetrainConstants.kMAX_DRIVETRAIN_TRANSLATIONAL_DECELERATION_METERS_PER_SECOND_SQUARED,
            0);
    private final SlewRateLimiter vySlewRateLimiter = 
        new SlewRateLimiter(
            Constants.DrivetrainConstants.kMAX_DRIVETRAIN_TRANSLATIONAL_ACCELERATION_METERS_PER_SECOND_SQUARED, 
            -Constants.DrivetrainConstants.kMAX_DRIVETRAIN_TRANSLATIONAL_DECELERATION_METERS_PER_SECOND_SQUARED,
            0);
    private final SlewRateLimiter omegaSlewRateLimiter = 
        new SlewRateLimiter(
            Constants.DrivetrainConstants.kMAX_DRIVETRAIN_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED, 
            -Constants.DrivetrainConstants.kMAX_DRIVETRAIN_ANGULAR_DECELERATION_RADIANS_PER_SECOND_SQUARED,
            0);
    private final SlewRateLimiter[] moduleDriveSlewRateLimiters = new SlewRateLimiter[4];

    private double prevDiscretizationTime = 0;
    private double prevGyroTime = 0;
    private double prevGyroYawDeg = 0;
    
    public SwerveDrive() {
        switch (Constants.currentMode) {
            case SIM:
                modules = new ModuleIO[] {
                    
                };
                gyroIO = new GyroIO() {};
                
                break;
            default:
                modules = new ModuleIO[] {
                    new ModuleIOTalon(0),
                    new ModuleIOTalon(1),
                    new ModuleIOTalon(2),
                    new ModuleIOTalon(3)
                };
                gyroIO = new GyroIONavX();

                break;
        }

        m_poseEstimator = new SwerveDrivePoseEstimator(
            m_kinematics,
            new Rotation2d(),
            new SwerveModulePosition[] {
                new SwerveModulePosition(0, new Rotation2d()),
                new SwerveModulePosition(0, new Rotation2d()),
                new SwerveModulePosition(0, new Rotation2d()),
                new SwerveModulePosition(0, new Rotation2d())
            },
            new Pose2d());

        odometryThread = new Notifier(this::updateOdometry);
        odometryThread.startPeriodic(0.02);

        for (int i = 0; i < 4; i++) {
            moduleDriveSlewRateLimiters[i] = 
                new SlewRateLimiter(
                    Constants.DrivetrainConstants.kMAX_MODULE_DRIVE_ACCELERATION_METERS_PER_SECOND, 
                    -Constants.DrivetrainConstants.kMAX_MODULE_DRIVE_DECELERATION_METERS_PER_SECOND,
                    0);
        }

    }

    private void updateInputs() {
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("SwerveDrive/Gyro", gyroInputs);

        for (int i = 0; i < 4; i++) {
            modules[i].updateInputs(moduleInputs[i]);
            Logger.processInputs("SwerveDrive/Module" + i, moduleInputs[i]);

            measuredModuleStates[i].angle = new Rotation2d(moduleInputs[i].anglePositionRad);
            measuredModuleStates[i].speedMetersPerSecond = moduleInputs[i].driveVelocityMps;
        }

        Logger.recordOutput("SwerveDrive/measuredModuleStates", measuredModuleStates);

        if (gyroInputs.isConnected) {
            yaw = gyroInputs.yaw;
        }
        else {
            yaw = new Rotation2d(yaw.getRadians() + 
                                m_kinematics.toChassisSpeeds(
                                measuredModuleStates[0],
                                measuredModuleStates[1], 
                                measuredModuleStates[2], 
                                measuredModuleStates[3]).omegaRadiansPerSecond * (Timer.getFPGATimestamp() - prevTime)); // TODO: discrepancy between prevTime being set and used in the calculation, causing a fixed offset

            yaw = new Rotation2d(yaw.getRadians() % (2 * Math.PI));

        }

        prevTime = Timer.getFPGATimestamp();
    }

    @Override 
    public void periodic() {
        odometryLock.lock();
        updateOdometry();
        odometryLock.unlock();

        Pair<Pose2d, Double> pair = new Pair<Pose2d, Double>(m_poseEstimator.getEstimatedPosition(), Double.valueOf(Timer.getFPGATimestamp()));
        poseQueue.add(pair);
        if (Timer.getFPGATimestamp() - poseQueue.peek().getSecond().doubleValue() > 1) {
            poseQueue.poll();
        }

        m_poseEstimator.update(yaw, new SwerveModulePosition[] {
            new SwerveModulePosition(moduleInputs[0].drivePositionMeters, new Rotation2d(moduleInputs[0].anglePositionRad)),
            new SwerveModulePosition(moduleInputs[1].drivePositionMeters, new Rotation2d(moduleInputs[1].anglePositionRad)),
            new SwerveModulePosition(moduleInputs[2].drivePositionMeters, new Rotation2d(moduleInputs[2].anglePositionRad)),
            new SwerveModulePosition(moduleInputs[3].drivePositionMeters, new Rotation2d(moduleInputs[3].anglePositionRad))
        });

        prevGyroYawDeg = yaw.getDegrees();
        prevGyroTime = Timer.getFPGATimestamp();

        measuredRobotRelativeSpeeds = m_kinematics.toChassisSpeeds(
            measuredModuleStates[0],
            measuredModuleStates[1],
            measuredModuleStates[2],
            measuredModuleStates[3]);

        measuredFieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(measuredRobotRelativeSpeeds, getPose().getRotation());
        
        Logger.recordOutput("SwerveDrive/estimYaw", yaw.getRadians());
        Logger.recordOutput("SwerveDrive/estimatedPose", new double[] {
            m_poseEstimator.getEstimatedPosition().getTranslation().getX(),
            m_poseEstimator.getEstimatedPosition().getTranslation().getY(),
            m_poseEstimator.getEstimatedPosition().getRotation().getRadians()
        });
        
        Logger.recordOutput("SwerveDrive/desiredFieldRelativeSpeeds", desiredFieldRelativeSpeeds);

        ChassisSpeeds scaledSpeeds =
            RebelUtil.scaleSpeeds(
                DrivetrainConstants.kMAX_DRIVETRAIN_TRANSLATIONAL_VELOCITY_METERS_PER_SECOND,
                desiredFieldRelativeSpeeds);
        if (Math.abs(scaledSpeeds.vxMetersPerSecond) < Math.abs(desiredFieldRelativeSpeeds.vxMetersPerSecond) ||
            Math.abs(scaledSpeeds.vyMetersPerSecond) < Math.abs(desiredFieldRelativeSpeeds.vyMetersPerSecond)) {
            desiredFieldRelativeSpeeds = scaledSpeeds;
        }

        Logger.recordOutput("SwerveDrive/scaledFieldRelativeSpeeds", scaledSpeeds);

        desiredFieldRelativeSpeeds.omegaRadiansPerSecond =
            RebelUtil.constrain(
                desiredFieldRelativeSpeeds.omegaRadiansPerSecond,
                -DrivetrainConstants.kMAX_DRIVETRAIN_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                DrivetrainConstants.kMAX_DRIVETRAIN_ANGULAR_VELOCITY_RADIANS_PER_SECOND);

        desiredFieldRelativeSpeeds.vxMetersPerSecond = Math.signum(desiredFieldRelativeSpeeds.vxMetersPerSecond) * vxSlewRateLimiter.calculate(Math.abs(desiredFieldRelativeSpeeds.vxMetersPerSecond));
        desiredFieldRelativeSpeeds.vyMetersPerSecond = Math.signum(desiredFieldRelativeSpeeds.vyMetersPerSecond) * vySlewRateLimiter.calculate(Math.abs(desiredFieldRelativeSpeeds.vyMetersPerSecond));
        desiredFieldRelativeSpeeds.omegaRadiansPerSecond = Math.signum(desiredFieldRelativeSpeeds.omegaRadiansPerSecond) * omegaSlewRateLimiter.calculate(Math.abs(desiredFieldRelativeSpeeds.omegaRadiansPerSecond));

        desiredRobotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(desiredFieldRelativeSpeeds, getPose().getRotation());
        Logger.recordOutput("SwerveDrive/desiredRobotRelativeSpeeds", desiredRobotRelativeSpeeds);
        Logger.recordOutput("SwerveDrive/measuredRobotRelativeSpeeds", measuredRobotRelativeSpeeds);
        Logger.recordOutput("SwerveDrive/measuredFieldRelativeSpeeds", measuredFieldRelativeSpeeds);
        
        double estimatedVyDriftMetersPerSecond = driveFFController.calculate(desiredFieldRelativeSpeeds.vxMetersPerSecond, desiredFieldRelativeSpeeds.omegaRadiansPerSecond);
        double estimatedVxDriftMetersPerSecond = driveFFController.calculate(desiredFieldRelativeSpeeds.vyMetersPerSecond, desiredFieldRelativeSpeeds.omegaRadiansPerSecond);
        Logger.recordOutput("SwerveDrive/estimatedVxDriftMetersPerSecond", estimatedVxDriftMetersPerSecond);
        Logger.recordOutput("SwerveDrive/estimatedVyDriftMetersPerSecond", estimatedVyDriftMetersPerSecond);

        ChassisSpeeds correctedSpeeds = desiredFieldRelativeSpeeds;
        correctedSpeeds.vxMetersPerSecond = correctedSpeeds.vxMetersPerSecond + estimatedVxDriftMetersPerSecond /* + 
                                            m_translationalFeedbackController.calculate(measuredFieldRelativeSpeeds.vxMetersPerSecond, 
                                            desiredFieldRelativeSpeeds.vxMetersPerSecond)*/;                                       
        correctedSpeeds.vyMetersPerSecond = correctedSpeeds.vyMetersPerSecond - estimatedVyDriftMetersPerSecond;
        correctedSpeeds.omegaRadiansPerSecond = correctedSpeeds.omegaRadiansPerSecond /* + m_angleFeedbackController.calculate(measuredFieldRelativeSpeeds.omegaRadiansPerSecond, desiredFieldRelativeSpeeds.omegaRadiansPerSecond)*/;

        correctedSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(correctedSpeeds, getPose().getRotation());

        Logger.recordOutput("SwerveDrive/correctedSpeeds", correctedSpeeds);

        if (Math.abs(desiredRobotRelativeSpeeds.vxMetersPerSecond) <= 0.01 && 
            Math.abs(desiredRobotRelativeSpeeds.vyMetersPerSecond) <= 0.01 && 
            Math.abs(desiredRobotRelativeSpeeds.omegaRadiansPerSecond) <= 0.1) {
            correctedSpeeds = desiredRobotRelativeSpeeds;
        } 

        SwerveModuleState[] desiredModuleStates = m_kinematics.toSwerveModuleStates(correctedSpeeds);
        double discretizationTime = Timer.getFPGATimestamp();
        ChassisSpeeds.discretize(correctedSpeeds, discretizationTime - prevDiscretizationTime);
        prevDiscretizationTime = discretizationTime;

        for (int i = 0; i < 4; i++) {
            desiredModuleStates[i] = SwerveModuleState.optimize(desiredModuleStates[i], measuredModuleStates[i].angle);
            desiredModuleStates[i].speedMetersPerSecond =
                RebelUtil.constrain(
                    desiredModuleStates[i].speedMetersPerSecond, 
                    -DrivetrainConstants.kMAX_MODULE_DRIVE_VELOCITY_METERS_PER_SECOND, 
                    DrivetrainConstants.kMAX_MODULE_DRIVE_VELOCITY_METERS_PER_SECOND);
            desiredModuleStates[i].speedMetersPerSecond = Math.signum(desiredModuleStates[i].speedMetersPerSecond) * moduleDriveSlewRateLimiters[i].calculate(Math.abs(desiredModuleStates[i].speedMetersPerSecond));
            modules[i].setState(desiredModuleStates[i]);
            
            Logger.recordOutput("SwerveDrive/unoptimizedDesiredModuleStates", desiredModuleStates);

        }

        Logger.recordOutput("SwerveDrive/desiredModuleStates", desiredModuleStates);
    }

    public void driveFieldRelative(ChassisSpeeds speeds) {
        desiredFieldRelativeSpeeds = speeds;
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        desiredFieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(speeds, getPose().getRotation());
    }

    public void resetPose(Pose2d pose) {
        odometryLock.lock();
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            modules[i].resetPosition();
            positions[i] = new SwerveModulePosition(0.0, new Rotation2d(moduleInputs[i].anglePositionRad));
        }

        // var alliance = DriverStation.getAlliance();
        // if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
        //     pose = GeometryUtil.flipFieldPose(pose);
        // } 

        yaw = pose.getRotation();
        Logger.recordOutput("IntialYAW", yaw.getDegrees());
        m_poseEstimator.resetPosition(gyroInputs.yaw, positions, pose); // TODO change this from gyroIO to yaw

        odometryLock.unlock();
    }

    public void zeroGyro() {
        gyroIO.zero();
        gyroIO.setOffset(new Rotation3d());
    }

    public ChassisSpeeds getMeasuredRobotRelativeSpeeds() {
        return measuredRobotRelativeSpeeds;
    }
    
    public ChassisSpeeds getMeasuredFeildRelativeSpeeds() {
        return measuredFieldRelativeSpeeds;
    }

    public Pose2d getPose() {
        odometryLock.lock();
        Pose2d estimate = m_poseEstimator.getEstimatedPosition();
        odometryLock.unlock();
        return estimate;
    }

    public Pose2d getPoseAtTimestamp(double time) {
        double lowestError = Double.MAX_VALUE;
        Pose2d pose = poseQueue.peek().getFirst();
        Logger.recordOutput("SwerveDrive/queueLength", poseQueue.size());
        for (Pair<Pose2d, Double> pair : poseQueue) {
            double currentError = Math.abs(time - pair.getSecond().doubleValue());
            if (currentError < lowestError) {
                lowestError = time - pair.getSecond().doubleValue();
                pose = pair.getFirst();
            }
            else {
                break;
            }
        }

        return pose;
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[] {
            new SwerveModulePosition(moduleInputs[0].drivePositionMeters, new Rotation2d(moduleInputs[0].anglePositionRad)),
            new SwerveModulePosition(moduleInputs[1].drivePositionMeters, new Rotation2d(moduleInputs[1].anglePositionRad)),
            new SwerveModulePosition(moduleInputs[2].drivePositionMeters, new Rotation2d(moduleInputs[2].anglePositionRad)),
            new SwerveModulePosition(moduleInputs[3].drivePositionMeters, new Rotation2d(moduleInputs[3].anglePositionRad))
        };
    }

    public void synchronizeModuleEncoders() {
        for (ModuleIO m : modules) {m.queueSynchronizeEncoders();}
    }

    private void updateOdometry() {
        odometryLock.lock();
        try {
            updateInputs();
            m_poseEstimator.update(yaw, getSwerveModulePositions());

            double velocitySum = 0.0;

            for (ModuleIOInputsAutoLogged m : moduleInputs) {
                velocitySum += Math.abs(m.driveVelocityMps);
            }

            if (velocitySum < 0.01 && ++syncCount > 5) {
                synchronizeModuleEncoders();
                syncCount = 0;
            }
        }

        catch (Exception e) {
            odometryLock.unlock();
            throw e;
        }

        odometryLock.unlock();
    }

    public void resetOdometry() {
        odometryLock.lock();
        m_poseEstimator.resetPosition(gyroInputs.yaw, getSwerveModulePositions(), getPose());
        odometryLock.unlock();
        m_kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, gyroInputs.yaw)); // TODO: this is yaw in radians right?
    }
}
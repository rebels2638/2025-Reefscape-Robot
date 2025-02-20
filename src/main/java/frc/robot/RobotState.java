package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.NoSuchElementException;
import java.util.Set;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.opencv.photo.AlignExposures;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.AlignmentConstants;
import frc.robot.constants.robotState.RobotStateConfigBase;
import frc.robot.constants.robotState.RobotStateConfigComp;
import frc.robot.constants.robotState.RobotStateConfigProto;
import frc.robot.constants.robotState.RobotStateConfigSim;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigBase;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigComp;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigProto;
import frc.robot.constants.swerve.drivetrainConfigs.SwerveDrivetrainConfigSim;
import frc.robot.lib.util.AlignmentUtil;

public class RobotState {

  private static RobotState instance;
  public static RobotState getInstance() {
    if (instance == null) {
        instance = new RobotState();
    }
    return instance;
  }
  
  public record OdometryObservation(
    SwerveModulePosition[]
    modulePositions, 
    SwerveModuleState[] moduleStates, 
    SwerveModuleState[] moduleAccelerations, 
    Rotation3d gyroOrientation,
    Rotation3d gyroRates,
    Translation2d fieldRelativeAccelerationMetersPerSecSec,
    double timestamp) {}
    
  public record VisionObservation(
    Pose2d visionPose, 
    double timestamp, 
    Matrix<N3, N1> stdDevs) {}

  public record ScoringObservation(
    Constants.scoredPositions position,
    Constants.level level,
    Constants.GamePiece GamePiece,
    double timestamp
  ) {}

  private static final double poseBufferSizeSeconds = 2.0;
  private final TimeInterpolatableBuffer<Pose2d> poseBuffer = TimeInterpolatableBuffer.createBuffer(poseBufferSizeSeconds);

  private final SwerveDrivePoseEstimator swerveDrivePoseEstimator;
  private double lastEstimatedPoseUpdateTime = 0;

  // Odometry
  private final SwerveDriveKinematics kinematics;
  private SwerveModulePosition[] lastWheelPositions = {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
    };

  private Rotation3d lastGyroOrientation = new Rotation3d();
  private Rotation3d lastGyroRates = new Rotation3d();
  private Translation2d lastFieldRelativeAccelerations = new Translation2d();

  private ChassisSpeeds robotRelativeVelocity = new ChassisSpeeds();

  private final SwerveDrivetrainConfigBase drivetrainConfig;
  private final RobotStateConfigBase robotStateConfig;

  private final ArrayList<Pair<Pair<Constants.scoredPositions, Constants.level>, Constants.GamePiece>> scoredPositions = 
    new ArrayList<Pair<Pair<Constants.scoredPositions, Constants.level> , Constants.GamePiece>>(); // implement something with this and figure out an elastic widget

  private RobotState() {
    switch (Constants.currentMode) {
        case COMP:
            drivetrainConfig = SwerveDrivetrainConfigComp.getInstance();
            robotStateConfig = RobotStateConfigComp.getInstance();

            break;

        case PROTO:
            drivetrainConfig = SwerveDrivetrainConfigProto.getInstance();
            robotStateConfig = RobotStateConfigProto.getInstance();

            break;
        
        case SIM:
            drivetrainConfig = SwerveDrivetrainConfigSim.getInstance();
            robotStateConfig = RobotStateConfigSim.getInstance();

            break;

        case REPLAY:
            drivetrainConfig = SwerveDrivetrainConfigComp.getInstance();
            robotStateConfig = RobotStateConfigComp.getInstance();

            break;

        default:
            drivetrainConfig = SwerveDrivetrainConfigComp.getInstance();
            robotStateConfig = RobotStateConfigComp.getInstance();


            break;
    }

    kinematics = new SwerveDriveKinematics(
        drivetrainConfig.getFrontLeftPositionMeters(),
        drivetrainConfig.getFrontRightPositionMeters(),
        drivetrainConfig.getBackLeftPositionMeters(),
        drivetrainConfig.getBackRightPositionMeters()
    ); 

    swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
        kinematics,
        lastGyroOrientation.toRotation2d(), 
        lastWheelPositions, 
        new Pose2d(),
        VecBuilder.fill(
          robotStateConfig.getOdomTranslationDevBase(),
          robotStateConfig.getOdomTranslationDevBase(),
          0
        ),
        VecBuilder.fill(
          robotStateConfig.getVisionTranslationDevBase(),
          robotStateConfig.getVisionTranslationDevBase(),
          9999999
        )
      );
  }

  /** Add odometry observation */
  public void addOdometryObservation(OdometryObservation observation) {
    Logger.recordOutput("RobotState/observation/timestamp", observation.timestamp());
    Logger.recordOutput("RobotState/observation/gyroOrientation", observation.gyroOrientation());
    Logger.recordOutput("RobotState/observation/gyroRates", observation.gyroRates());
    Logger.recordOutput("RobotState/observation/modulePositions", observation.modulePositions());
    Logger.recordOutput("RobotState/observation/moduleStates", observation.moduleStates());
    Logger.recordOutput("RobotState/lastWheelPositions", lastWheelPositions);
    Logger.recordOutput("RobotState/observation/moduleAccelerations", observation.moduleAccelerations);

    robotRelativeVelocity = kinematics.toChassisSpeeds(observation.moduleStates);

    Twist2d twist = kinematics.toTwist2d(lastWheelPositions, observation.modulePositions());
    lastWheelPositions = observation.modulePositions();

    Logger.recordOutput("RobotState/twist", twist);


    if (observation.gyroOrientation != null) {
        lastGyroOrientation = observation.gyroOrientation();
        lastGyroRates = observation.gyroRates();

        Logger.recordOutput("RobotState/isUsingTwistAngle", false);
    }
    else {
        lastGyroOrientation = new Rotation3d(0, 0, lastGyroOrientation.getZ() + twist.dtheta);
        lastGyroRates = new Rotation3d(0, 0, robotRelativeVelocity.omegaRadiansPerSecond);

        Logger.recordOutput("RobotState/dtheta", twist.dtheta);
        Logger.recordOutput("RobotState/isUsingTwistAngle", true);
    }

    if (observation.fieldRelativeAccelerationMetersPerSecSec != null) {
        lastFieldRelativeAccelerations = observation.fieldRelativeAccelerationMetersPerSecSec;
    }
    else {
        ChassisSpeeds acelSpeeds = 
            ChassisSpeeds.fromRobotRelativeSpeeds(
                kinematics.toChassisSpeeds(observation.moduleAccelerations), 
                new Rotation2d(lastGyroOrientation.getZ())
            );
        lastFieldRelativeAccelerations = 
            new Translation2d(
                acelSpeeds.vxMetersPerSecond,
                acelSpeeds.vyMetersPerSecond
            );
    }

    Logger.recordOutput("RobotState/lastGyroOrientation", lastGyroOrientation);
    Logger.recordOutput("RobotState/lastGyroRates", lastGyroRates);
    Logger.recordOutput("RobotState/lastFieldRelativeAccelerations", lastFieldRelativeAccelerations);

    // Add twist to odometry pose
    swerveDrivePoseEstimator.updateWithTime(observation.timestamp, lastGyroOrientation.toRotation2d(), lastWheelPositions);
    lastEstimatedPoseUpdateTime = Timer.getTimestamp();

    // Add pose to buffer at timestamp
    poseBuffer.addSample(lastEstimatedPoseUpdateTime, swerveDrivePoseEstimator.getEstimatedPosition()); 
  }

  public void addVisionObservation(VisionObservation observation) {
    // If measurement is old enough to be outside the pose buffer's timespan, skip.
    try {
      if (poseBuffer.getInternalBuffer().lastKey() - poseBufferSizeSeconds > observation.timestamp()) {
        return;
      }
    } 
    
    catch (NoSuchElementException ex) {
      return;
    }

    Logger.recordOutput("RobotState/vision/stdDevTranslation" ,observation.stdDevs.get(0,0));
    Logger.recordOutput("RobotState/vision/visionPose", observation.visionPose);


    swerveDrivePoseEstimator.addVisionMeasurement(observation.visionPose, observation.timestamp, observation.stdDevs);
    lastEstimatedPoseUpdateTime = Timer.getTimestamp();
  }

  public void addScoringObservation(ScoringObservation observation) {
    Pair<Constants.scoredPositions, Constants.level> inner = new Pair<>(observation.position, observation.level);
    Pair<Pair<Constants.scoredPositions, Constants.level>, Constants.GamePiece> outer = new Pair<>(inner, observation.GamePiece);
    this.scoredPositions.add(outer);

    for (int i = 0; i < scoredPositions.size(); i++) {
      System.out.println(scoredPositions.get(i));
    }
  }

  public boolean alreadyScored(Pose2d target, Constants.level requestedLevel, Constants.GamePiece gamePiece) {
    Constants.scoredPositions target_position = getScoredPosition(target);
    for (Pair<Pair<Constants.scoredPositions, Constants.level>, Constants.GamePiece> a : scoredPositions) {
      if (a.getFirst().getFirst() == target_position && a.getFirst().getSecond() == requestedLevel && a.getSecond() == gamePiece) {
        return true;
      }
    }
    return false;
  }

  public Constants.scoredPositions getScoredPosition(Pose2d arg) {
    if (arg == null) {arg = getEstimatedPose();}
    return arg.nearest(AlignmentUtil.yieldPotentialScoringAlignmentTargetsClockwise()).getTranslation().getDistance(arg.getTranslation()) < 1 ? // TODO: Constant
    Arrays.asList(
      Constants.scoredPositions.values()
      ).get(
        AlignmentUtil.yieldPotentialScoringAlignmentTargetsClockwise()
        .indexOf(arg
          .nearest(AlignmentUtil
            .yieldPotentialScoringAlignmentTargetsClockwise()
            )
          )
        ) : Constants.scoredPositions.BARGE;
  }

  /**
   * Reset estimated pose and odometry pose to pose <br>
   * Clear pose buffer
   */
  public void resetPose(Pose2d initialPose) {
    swerveDrivePoseEstimator.resetPosition(lastGyroOrientation.toRotation2d(), lastWheelPositions, initialPose);
    poseBuffer.clear();
  }

  public void zeroGyro() {
    resetPose(new Pose2d(getEstimatedPose().getTranslation(), new Rotation2d()));
  }

  @AutoLogOutput(key = "RobotState/estimatedPose")
  public Pose2d getEstimatedPose() {
    return swerveDrivePoseEstimator.getEstimatedPosition();
  }

  public Rotation3d getGyroOrientation() {
    Rotation3d rotationWithYaw = // we want to use the yaw updated by pose estimator during gyro zeroing
        new Rotation3d(
            lastGyroOrientation.getX(), 
            lastGyroOrientation.getY(), 
            getEstimatedPose().getRotation().getRadians()
        );
    return rotationWithYaw;
  }

  @AutoLogOutput(key = "RobotState/lastGyroRates")
  public Rotation3d getGyroRates() {
    return lastGyroRates;
  }
  
  @AutoLogOutput(key = "RobotState/robotRelativeSpeeds")
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return robotRelativeVelocity;
  }

  @AutoLogOutput(key = "RobotState/fieldRelativeSpeeds")
  public ChassisSpeeds getFieldRelativeSpeeds() { 
    return ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeVelocity, lastGyroOrientation.toRotation2d());
  }
  @AutoLogOutput(key = "RobotState/fieldRelativeAccelerations")
  public Translation2d getFieldRelativeAccelerations() {
    return lastFieldRelativeAccelerations;
  }

  public Pose2d getPredictedPose(double translationLookaheadS, double rotationLookaheadS) {
    return getEstimatedPose()
        .transformBy(
            new Transform2d(
                robotRelativeVelocity.vxMetersPerSecond * translationLookaheadS,
                robotRelativeVelocity.vyMetersPerSecond * translationLookaheadS,
                Rotation2d.fromRadians(robotRelativeVelocity.omegaRadiansPerSecond * rotationLookaheadS)));
  }

  public Pose2d getPredictedPose(double timestamp) {
    return getPredictedPose(timestamp - lastEstimatedPoseUpdateTime, timestamp - lastEstimatedPoseUpdateTime);
  }

  public boolean getIsElevatorExtendable() {
    return 
        // ensure that robot is below a velocity threshold
        Math.hypot(getFieldRelativeSpeeds().vxMetersPerSecond, getFieldRelativeSpeeds().vyMetersPerSecond) <= robotStateConfig.getMaxElevatorExtensionVelocity() &&
        // ensure that the robot is not decelerating / accelerating too quickly
        Math.hypot(lastFieldRelativeAccelerations.getX(), lastFieldRelativeAccelerations.getY()) < 1 &&
        getFieldRelativeSpeeds().omegaRadiansPerSecond < 1.0 &&
        // ensure that we are decelerating in each axis of movement
        Math.signum(lastFieldRelativeAccelerations.getX()) != Math.signum(getFieldRelativeSpeeds().vxMetersPerSecond) &&
        Math.signum(lastFieldRelativeAccelerations.getY()) != Math.signum(getFieldRelativeSpeeds().vyMetersPerSecond); 
  }
}
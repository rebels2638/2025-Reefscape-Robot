// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.math.*;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.swerve.*;

import java.util.NoSuchElementException;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

public class RobotState {
  private static RobotState instance;
  public static RobotState getInstance() {
    if (instance == null) instance = new RobotState(new SwerveCrescendoRobotBaseConfig());
    return instance;
  }
  
  public record OdometryObservation(SwerveModulePosition[] modulePositions, SwerveModuleState[] moduleStates, Rotation2d gyroAngle, double timestamp) {}
  public record VisionObservation(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {}


  private static final double poseBufferSizeSeconds = 2.0;
  private final TimeInterpolatableBuffer<Pose2d> poseBuffer = TimeInterpolatableBuffer.createBuffer(poseBufferSizeSeconds);

  private final SwerveDrivePoseEstimator swerveDrivePoseEstimator;
  private double lastEstimatedPoseUpdateTime = 0;

  //TODO: DO THIS
  private final Matrix<N3, N1> qStdDevs = new Matrix<>(Nat.N3(), Nat.N1());

  // Odometry
  private final SwerveDriveKinematics kinematics;
  private SwerveModulePosition[] lastWheelPositions = {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
    };

  private Rotation2d lastGyroAngle = new Rotation2d();
  private ChassisSpeeds robotRelativeVelocity = new ChassisSpeeds();

  //TODO: FIX THE CONFIGS!!!! THEY SUCK - the writer of the configs
  private RobotState(SwerveConfigBase config) {
    kinematics = new SwerveDriveKinematics(
        config.getSwerveDrivetrainConfig().kFRONT_LEFT_POSITION_METERS,
        config.getSwerveDrivetrainConfig().kFRONT_RIGHT_POSITION_METERS,
        config.getSwerveDrivetrainConfig().kBACK_LEFT_POSITION_METERS,
        config.getSwerveDrivetrainConfig().kBACK_RIGHT_POSITION_METERS
    ); 

    swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(kinematics, lastGyroAngle, lastWheelPositions, new Pose2d());
  }

  /** Add odometry observation */
  public void addOdometryObservation(OdometryObservation observation) {
    Twist2d twist = kinematics.toTwist2d(lastWheelPositions, observation.modulePositions());
    lastWheelPositions = observation.modulePositions();

    if (observation.gyroAngle != null) {
        lastGyroAngle = observation.gyroAngle();
    }
    else {
        lastGyroAngle = lastGyroAngle.plus(new Rotation2d(twist.dtheta));
    }

    // Add twist to odometry pose
    swerveDrivePoseEstimator.updateWithTime(observation.timestamp, lastGyroAngle, lastWheelPositions);
    lastEstimatedPoseUpdateTime = Timer.getFPGATimestamp();
    robotRelativeVelocity = kinematics.toChassisSpeeds(observation.moduleStates);

    // Add pose to buffer at timestamp
    poseBuffer.addSample(lastEstimatedPoseUpdateTime, swerveDrivePoseEstimator.getEstimatedPosition()); 
     
    Logger.recordOutput("RobotState/estimatedPosition", swerveDrivePoseEstimator.getEstimatedPosition());  
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

    // Get odometry based pose at timestamp
    Optional<Pose2d>  sample = poseBuffer.getSample(observation.timestamp());
    if (sample.isEmpty()) {
      // exit if not there
      return;
    }

    swerveDrivePoseEstimator.addVisionMeasurement(observation.visionPose, observation.timestamp, observation.stdDevs);
    lastEstimatedPoseUpdateTime = Timer.getFPGATimestamp();

    Logger.recordOutput("RobotState/estimatedPosition", swerveDrivePoseEstimator.getEstimatedPosition());  

  }

  /**
   * Reset estimated pose and odometry pose to pose <br>
   * Clear pose buffer
   */
  public void resetPose(Pose2d initialPose) {
    swerveDrivePoseEstimator.resetPosition(lastGyroAngle, lastWheelPositions, initialPose);
    poseBuffer.clear();
  }

  public Pose2d getEstimatedPose() {
    return swerveDrivePoseEstimator.getEstimatedPosition();
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
}
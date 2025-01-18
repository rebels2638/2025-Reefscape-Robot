// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.math.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.NoSuchElementException;
import org.littletonrobotics.junction.AutoLogOutput;

public class RobotState {
  private static RobotState instance;
  public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }
  
  public record OdometryObservation(SwerveModulePosition[] wheelPositions, Rotation2d gyroAngle, double timestamp) {}
  public record VisionObservation(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {}

  private static final double poseBufferSizeSeconds = 2.0;

  private Pose2d odometryPose = new Pose2d();
  private Pose2d estimatedPose = new Pose2d();

  private final TimeInterpolatableBuffer<Pose2d> poseBuffer = TimeInterpolatableBuffer.createBuffer(poseBufferSizeSeconds);
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
  private Twist2d robotVelocity = new Twist2d();

  private RobotState() {
    for (int i = 0; i < 3; ++i) {
      qStdDevs.set(i, 0, 0); // TODO: stdev measurements
    }
    kinematics = new SwerveDriveKinematics(
        new Translation2d(0.38, -0.38),
        new Translation2d(-0.38, -0.38),
        new Translation2d(0.38, 0.38),
        new Translation2d(-0.38, 0.38)); 
  }

  /** Add odometry observation */
  public void addOdometryObservation(OdometryObservation observation) {

    Twist2d twist = kinematics.toTwist2d(lastWheelPositions, observation.wheelPositions());
    lastWheelPositions = observation.wheelPositions();

    if (observation.gyroAngle != null) {
      // Update dtheta for twist if gyro connected
      twist = new Twist2d(twist.dx, twist.dy, observation.gyroAngle().minus(lastGyroAngle).getRadians());
      lastGyroAngle = observation.gyroAngle();
    }

    // Add twist to odometry pose
    odometryPose = odometryPose.exp(twist);
    // Add pose to buffer at timestamp
    poseBuffer.addSample(observation.timestamp(), odometryPose);
    // Calculate diff from last odometry pose and add onto pose estimate
    estimatedPose = estimatedPose.exp(twist);
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
    var sample = poseBuffer.getSample(observation.timestamp());
    if (sample.isEmpty()) {
      // exit if not there
      return;
    }

    // sample --> odometryPose transform and backwards of that
    var sampleToOdometryTransform = new Transform2d(sample.get(), odometryPose);
    var odometryToSampleTransform = new Transform2d(odometryPose, sample.get());
    // get old estimate by applying odometryToSample Transform
    Pose2d estimateAtTime = estimatedPose.plus(odometryToSampleTransform);

    // Calculate 3 x 3 vision matrix
    var r = new double[3];
    for (int i = 0; i < 3; ++i) {
      r[i] = observation.stdDevs().get(i, 0) * observation.stdDevs().get(i, 0);
    }
    // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
    // and C = I. See wpimath/algorithms.md.
    Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());
    for (int row = 0; row < 3; ++row) {
      double stdDev = qStdDevs.get(row, 0);
      if (stdDev == 0.0) {
        visionK.set(row, row, 0.0);
      } else {
        visionK.set(row, row, stdDev / (stdDev + Math.sqrt(stdDev * r[row])));
      }
    }
    // difference between estimate and vision pose
    Transform2d transform = new Transform2d(estimateAtTime, observation.visionPose());
    // scale transform by visionK
    var kTimesTransform =
        visionK.times(
            VecBuilder.fill(
                transform.getX(), transform.getY(), transform.getRotation().getRadians()));
    Transform2d scaledTransform =
        new Transform2d(
            kTimesTransform.get(0, 0),
            kTimesTransform.get(1, 0),
            Rotation2d.fromRadians(kTimesTransform.get(2, 0)));

    // Recalculate current estimate by applying scaled transform to old estimate
    // then replaying odometry data
    estimatedPose = estimateAtTime.plus(scaledTransform).plus(sampleToOdometryTransform);
  }

  public void addVelocityData(Twist2d robotVelocity) {
    this.robotVelocity = robotVelocity;
  }

//   public ModuleLimits getModuleLimits() {
//     return flywheelAccelerating && !DriverStation.isAutonomousEnabled()
//         ? DriveConstants.moduleLimitsFlywheelSpinup
//         : DriveConstants.moduleLimitsFree;
//   }

  /**
   * Reset estimated pose and odometry pose to pose <br>
   * Clear pose buffer
   */
  public void resetPose(Pose2d initialPose) {
    estimatedPose = initialPose;
    odometryPose = initialPose;
    poseBuffer.clear();
  }

  public Twist2d fieldVelocity() {

    Translation2d linearFieldVelocity =
        new Translation2d(robotVelocity.dx, robotVelocity.dy).rotateBy(estimatedPose.getRotation());

    return new Twist2d(
        linearFieldVelocity.getX(), linearFieldVelocity.getY(), robotVelocity.dtheta);
  }

  public Pose2d getEstimatedPose() {
    return estimatedPose;
  }

  public Pose2d getPredictedPose(double translationLookaheadS, double rotationLookaheadS) {
    // Twist2d velocity = DriverStation.isAutonomousEnabled() ? trajectoryVelocity : robotVelocity;
    Twist2d velocity = robotVelocity;
    return getEstimatedPose()
        .transformBy(
            new Transform2d(
                velocity.dx * translationLookaheadS,
                velocity.dy * translationLookaheadS,
                Rotation2d.fromRadians(velocity.dtheta * rotationLookaheadS)));
  }

  @AutoLogOutput(key = "RobotState/OdometryPose")
  public Pose2d getOdometryPose() {
    return odometryPose;
  }
}
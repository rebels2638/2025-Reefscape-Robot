package frc.robot.subsystems.drivetrain.swerve.gyro;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface GyroIO {
    @AutoLog
    class GyroIOInputs {
        public boolean isConnected = false;
        public Rotation2d[] orientation = new Rotation2d[] {new Rotation2d(), new Rotation2d(), new Rotation2d()};
        public Rotation2d[] rates = new Rotation2d[] {new Rotation2d(), new Rotation2d(), new Rotation2d()};
        public Translation2d fieldRelativeAccelerationMetersPerSecSec = new Translation2d();
    }

    public default void updateInputs(GyroIOInputs inputs) {}
    public default void resetGyro(Rotation2d yaw) {};
}
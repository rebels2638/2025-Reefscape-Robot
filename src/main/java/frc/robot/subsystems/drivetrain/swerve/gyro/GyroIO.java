package frc.robot.subsystems.drivetrain.swerve.gyro;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;

public interface GyroIO {
    @AutoLog
    class GyroIOInputs {
        public boolean isConnected = false;
        public Rotation3d orientation = new Rotation3d();
        public double angularVelocityRadPerSec = 0;
        public Translation2d worldAccelerationMetersPerSecSec = new Translation2d();
    }

    public default void updateInputs(GyroIOInputs inputs) {}

    public default int getCanDeviceId() {
        return 0;
    }

    public default void resetGyro(Rotation2d yaw) {};
}
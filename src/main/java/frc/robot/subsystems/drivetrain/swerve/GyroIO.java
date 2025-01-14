package frc.robot.subsystems.drivetrain.swerve;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public Rotation2d yaw = new Rotation2d();
        public boolean isConnected = false;
        public double yawRadSec = 0;
    }

    public default void updateInputs(GyroIOInputs inputs) {};
    public default void setOffset(Rotation3d offset) {};
    public default void zero() {};
    public default void reset(Rotation3d inital) {};

}

package frc.robot.subsystems.drivetrain.swerve.module;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface ModuleIO {
    @AutoLog
    class ModuleIOInputs {
        public double timestamp;

        // drive
        public double drivePositionMeters;
        public double driveVelocityMetersPerSec;

        public double driveCurrentDrawAmps;
        public double driveAppliedVolts;
        public double driveTemperatureFahrenheit;

        // steer
        public Rotation2d steerCANCODERAbsolutePosition;
        public Rotation2d steerPosition;
        public double steerVelocityRadPerSec;

        public double steerCurrentDrawAmps;
        public double steerAppliedVolts;
        public double steerTemperatureFahrenheit;
    }

    public default void updateInputs(ModuleIOInputs inputs) {}
    public default void setState(SwerveModuleState state) {}
    public default void setDriveVoltage(double baseUnitMagnitude) {}
    
}

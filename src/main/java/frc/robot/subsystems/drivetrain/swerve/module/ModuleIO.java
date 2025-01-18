package frc.robot.subsystems.drivetrain.swerve.module;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface ModuleIO {
    @AutoLog
    class ModuleIOInputs {
        public double timestamp = 0;

        // drive
        public double drivePositionMeters = 0;
        public double driveVelocityMetersPerSec = 0;

        public double driveCurrentDrawAmps = 0;
        public double driveAppliedVolts = 0;
        public double driveTemperatureFahrenheit = 0;

        // steer
        public Rotation2d steerCANCODERAbsolutePosition = new Rotation2d();
        public Rotation2d steerPosition = new Rotation2d();
        public double steerVelocityRadPerSec = 0;

        public double steerCurrentDrawAmps = 0;
        public double steerAppliedVolts = 0;
        public double steerTemperatureFahrenheit = 0;
    }

    public default void updateInputs(ModuleIOInputs inputs) {}
    public default void setState(SwerveModuleState state) {}
    public default SwerveModuleState setTargetState(SwerveModuleState state) {return new SwerveModuleState(0.0, new Rotation2d());}
    public default void setDriveVoltage(double baseUnitMagnitude) {}
    public default SwerveModuleState getState() {return new SwerveModuleState(0.0, new Rotation2d());}
    public default SwerveModulePosition getPosition() {return new SwerveModulePosition(0.0, new Rotation2d());}
    
}

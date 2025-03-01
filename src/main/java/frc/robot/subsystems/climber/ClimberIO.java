package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Rotation2d;

public interface ClimberIO {
    @AutoLog
    class ClimberIOInputs {
        public Rotation2d climberPosition = new Rotation2d();
        public double climberVelocityRadPerSec = 0;

        public double climberCurrentDrawAmps = 0;
        public double climberAppliedVolts = 0;
        public double climberTemperatureFahrenheit = 0;

        public boolean climberMotorConnected = true;
    }

    public default void updateInputs(ClimberIOInputs inputs) {}
    public default void setAngle(Rotation2d angle) {}
    public default void setTorqueCurrentFOC(double baseUnitMagnitude) {}
    public default void setVoltage(double baseUnitMagnitude) {}    
}

package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Rotation2d;

public interface PivotIO {
    @AutoLog
    class PivotIOInputs {
        public Rotation2d pivotPositionRad = new Rotation2d();
        public double pivotVelocityRadPerSec = 0;

        public double pivotCurrentDrawAmps = 0;
        public double pivotAppliedVolts = 0;
        public double pivotTemperatureFahrenheit = 0;
    }

    public default void updateInputs(PivotIOInputs inputs) {}
    public default void setAngle(Rotation2d angle) {}
    public default void setTorqueCurrentFOC(double baseUnitMagnitude) {}
    public default void setVoltage(double baseUnitMagnitude) {}    
}

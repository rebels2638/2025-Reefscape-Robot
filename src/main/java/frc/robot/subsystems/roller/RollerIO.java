package frc.robot.subsystems.roller;

import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {
    @AutoLog
    class RollerIOInputs {
        public double rollerVelocityRadPerSec = 0;

        public double rollerCurrentDrawAmps = 0;
        public double rollerAppliedVolts = 0;
        public double rollerTemperatureFahrenheit = 0;

        public boolean inRoller = true;
    }

    public default void updateInputs(RollerIOInputs inputs) {}
    public default void setTorqueCurrentFOC(double baseUnitMagnitude) {}    
    public default void setVoltage(double baseUnitMagnitude) {}
}

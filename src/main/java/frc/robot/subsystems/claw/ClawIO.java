package frc.robot.subsystems.claw;

import org.littletonrobotics.junction.AutoLog;

public interface ClawIO {
    @AutoLog
    class ClawIOInputs {
        public double clawVelocityRadPerSec = 0;

        public double clawCurrentDrawAmps = 0;
        public double clawAppliedVolts = 0;
        public double clawTemperatureFahrenheit = 0;

        public boolean inClaw = false;
        public boolean isConnected = false;
    }

    public default void updateInputs(ClawIOInputs inputs) {}
    public default void setTorqueCurrentFOC(double baseUnitMagnitude) {}    
    public default void setVoltage(double baseUnitMagnitude) {}    

}

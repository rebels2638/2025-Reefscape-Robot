package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    class ElevatorIOInputs {
        public double elevatorHeightMeters = 0;
        public double elevatorVelocityMetersPerSec = 0;

        public double elevatorCurrentDrawAmps1 = 0;
        public double elevatorAppliedVolts1 = 0;
        public double elevatorTemperatureFahrenheit1 = 0;

        public double elevatorCurrentDrawAmps2 = 0;
        public double elevatorAppliedVolts2 = 0;
        public double elevatorTemperatureFahrenheit2 = 0;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}
    public default void setHeight(double height) {}
    public default void setTorqueCurrentFOC(double baseUnitMagnitude) {}
    public default void setVoltage(double baseUnitMagnitude) {}
}

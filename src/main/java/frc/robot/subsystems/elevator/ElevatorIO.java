package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO { 
    @AutoLog
    class ElevatorIOInputsAutoLogged {
        public double timestamp = 0;

        public double m1PositionMeters = 0;
        public double m1VelocityMetersPerSec = 0;

        public double m1CurrentDrawAmps = 0;
        public double m1AppliedVolts = 0;
        public double m1TemperatureFahrenheit = 0;

        public double m2PositionMeters = 0;
        public double m2VelocityMetersPerSec = 0;

        public double m2CurrentDrawAmps = 0;
        public double m2AppliedVolts = 0;
        public double m2TemperatureFahrenheit = 0;

        public boolean reachedSetpoint = false;
    }

    public default void updateInputs(ElevatorIOInputsAutoLogged inputs) {}
    public default double getPosition() {return 0.0;}
    public default double getVelocity() {return 0.0;}
    public default void setPosition(double posiiton) {}
    public default void zeroHeight() {}
    
}

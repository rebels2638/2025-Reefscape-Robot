package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public double shooterHeightMeters;
        public double climberHeightMeters;
        public double voltageOut;
        public boolean reachedSetpoint; 
        public double goalPositionMeters;

    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void setHeightMeters(double goalPositionMeters) {}

    public default void configureController(ElevatorFeedforward pff, PIDController pfb, ElevatorFeedforward cff,  double kCLIMB_KG) {}

    public default void setVoltage(double voltage) {}
    
    public default void zeroHeight() {}
    
}

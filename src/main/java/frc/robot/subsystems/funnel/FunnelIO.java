package frc.robot.subsystems.funnel;

import org.littletonrobotics.junction.AutoLog;

public interface FunnelIO {
    @AutoLog
    class FunnelIOInputs {
        public boolean isPulled = false;
    }

    public default void updateInputs(FunnelIOInputs inputs) {}
    public default void pull() {}    
}

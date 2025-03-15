package frc.robot.subsystems.pneumatics;

import org.littletonrobotics.junction.AutoLog;

public interface PneumaticsIO {
    @AutoLog
    class PneumaticsIOInputs {
        public boolean isFunnelPulled = false;
        public boolean isRatchetPulled = false;

        public boolean isCompressredEnabled = false;
        public double pressure = 0;
    }

    public default void updateInputs(PneumaticsIOInputs inputs) {}

    public default void pullFunnel() {}  
    public default void pushFunnel() {}    

    public default void pullRatchet() {}  
    public default void pushRatchet() {}  
  
}

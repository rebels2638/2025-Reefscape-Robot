package frc.robot.constants.pivot;

public class PivotConfigSim extends PivotConfigBase {
    private static PivotConfigSim instance;
    public static PivotConfigSim getInstance() {
        if (instance == null) {
            instance = new PivotConfigSim();
        }

        return instance;
    }

    private PivotConfigSim() {}

    // CANID
    @Override
    public int getCanID() {
        return 1;
    }

    // Supply current limits
    @Override
    public double getSupplyCurrentLimit() {
        return 30.0;
    }

    @Override
    public double getSupplyCurrentLimitLowerTime() {
        return 1.0;
    }

    @Override
    public double getSupplyCurrentLimitLowerLimit() {
        return 20.0;
    }

    // Stator current limit
    @Override
    public double getStatorCurrentLimit() {
        return 35.0;
    }

    // Peak torque currents
    @Override
    public double getPeakForwardTorqueCurrent() {
        return 40.0;
    }

    @Override
    public double getPeakReverseTorqueCurrent() {
        return -40.0;
    }

    // Characterization / Gains
    @Override
    public double getKS() {
        return 0;
    }

    @Override
    public double getKV() {
        return 0;
    }

    @Override
    public double getKA() {
        return 0.0;
    }

    @Override
    public double getKP() {
        return 10;
    }

    @Override
    public double getKI() {
        return 0.0;
    }

    @Override
    public double getKD() {
        return 0.0;
    }

    @Override
    public double getKG() {
        return 0; 
    }

    // Motion magic parameters
    @Override
    public double getMotionMagicExpoKA() {
        return 11;
    }

    @Override
    public double getMotionMagicExpoKV() {
        return 10.0;
    }

    @Override
    public double getMotionMagicCruiseVelocityRotationsPerSec() {
        return 0.75;
    }

    // Neutral mode
    @Override
    public boolean isNeutralModeBrake() {
        return true;
    }

    // Gear ratio
    @Override
    public double getMotorToOutputShaftRatio() {
        return 100.0;
    }

    @Override
    public double getMaxAngleRotations() {
        return 0.75;
    }

    @Override 
    public double getMinAngleRotations() {
        return -0.25;
    }
}

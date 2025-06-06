package frc.robot.constants.climber;

public class ClimberConfigSim extends ClimberConfigBase {
    private static ClimberConfigSim instance;
    public static ClimberConfigSim getInstance() {
        if (instance == null) {
            instance = new ClimberConfigSim();
        }

        return instance;
    }

    private ClimberConfigSim() {}

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
        return 1;
    }

    @Override
    public double getKG() {
        return 0.85; 
    }

    // Motion magic parameters
    @Override
    public double getMotionMagicExpoKA() {
        return 1f;
    }

    @Override
    public double getMotionMagicExpoKV() {
        return 1;
    }

    @Override
    public double getMotionMagicCruiseVelocityRotationsPerSec() {
        return 1;
    }

    @Override
    public double getToleranceDegrees() {
        return 1;
    }

    // Neutral mode
    @Override
    public boolean isNeutralModeBrake() {
        return true;
    }

    @Override
    public boolean isInverted() {
        return true;
    }

    // Gear ratio
    @Override
    public double getMotorToOutputShaftRatio() {
        return 100.0;
    }

    @Override
    public double getMaxAngleRotations() {
        return 220/360.0;
    }

    @Override 
    public double getMinAngleRotations() {
        return -80.00/360.0;
    }

    @Override 
    public double getStartingAngleRotations() {
        return getMaxAngleRotations();
    } 
}

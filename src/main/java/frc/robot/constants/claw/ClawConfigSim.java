package frc.robot.constants.claw;

public class ClawConfigSim extends ClawConfigBase {
    private static ClawConfigSim instance;
    public static ClawConfigSim getInstance() {
        if (instance == null) {
            instance = new ClawConfigSim();
        }

        return instance;
    }

    private ClawConfigSim() {}

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

    // Neutral mode
    @Override
    public boolean isNeutralModeBrake() {
        return true;
    }

    @Override
    public double getHighPassFilterTimeConstant() {
        return 0.1;
    }

    @Override
    public double getHighPassFilterUpperTrip() {
        return 10;
    }

    @Override
    public double getHighPassFilterLowerTrip() {
        return 10;
    }

    @Override
    public boolean getIsMotorInverted() {
        return true;
    }
    
}

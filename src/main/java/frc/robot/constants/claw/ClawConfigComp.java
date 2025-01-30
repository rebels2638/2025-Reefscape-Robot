package frc.robot.constants.claw;

public class ClawConfigComp extends ClawConfigBase {
    private static ClawConfigComp instance;
    public static ClawConfigComp getInstance() {
        if (instance == null) {
            instance = new ClawConfigComp();
        }

        return instance;
    }

    private ClawConfigComp() {}
    
    // CANID
    @Override
    public int getCanID() {
        return 14;
    }

    // Supply current limits
    @Override
    public double getSupplyCurrentLimit() {
        return 15.0;
    }

    @Override
    public double getSupplyCurrentLimitLowerTime() {
        return 1.2;
    }

    @Override
    public double getSupplyCurrentLimitLowerLimit() {
        return 5;
    }

    // Stator current limit
    @Override
    public double getStatorCurrentLimit() {
        return 10;
    }

    // Peak torque currents
    @Override
    public double getPeakForwardTorqueCurrent() {
        return 10;
    }

    @Override
    public double getPeakReverseTorqueCurrent() {
        return -10;
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
}

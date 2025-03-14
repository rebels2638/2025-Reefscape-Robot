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
        return 18;
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
        return 15;
    }

    // Stator current limit
    @Override
    public double getStatorCurrentLimit() {
        return 13;
    }

    // Peak torque currents
    @Override
    public double getPeakForwardTorqueCurrent() {
        return 40;
    }

    @Override
    public double getPeakReverseTorqueCurrent() {
        return -40;
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
        return false;
    }
    
    @Override
    public double getMinInClawCurrentActivation() {
        return 5;
    }

    @Override
    public double getMaxInClawVeloRadSecActivation() {
        return 0.1;
    }
}

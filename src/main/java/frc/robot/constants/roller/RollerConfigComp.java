package frc.robot.constants.roller;

public class RollerConfigComp extends RollerConfigBase {
    private static RollerConfigComp instance;
    public static RollerConfigComp getInstance() {
        if (instance == null) {
            instance = new RollerConfigComp();
        }

        return instance;
    }

    private RollerConfigComp() {}
    
    // CANID
    @Override
    public int getCANID() {
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
}

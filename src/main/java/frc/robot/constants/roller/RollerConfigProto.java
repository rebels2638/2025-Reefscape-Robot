package frc.robot.constants.roller;

public class RollerConfigProto extends RollerConfigBase {
    private static RollerConfigProto instance;
    public static RollerConfigProto getInstance() {
        if (instance == null) {
            instance = new RollerConfigProto();
        }

        return instance;
    }

    private RollerConfigProto() {}
    
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

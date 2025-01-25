package frc.robot.constants.roller;

public abstract class RollerConfigBase {
    // CAN ID
    public abstract int getCANID();

    // Supply current limits
    public abstract double getSupplyCurrentLimit();
    public abstract double getSupplyCurrentLimitLowerTime();
    public abstract double getSupplyCurrentLimitLowerLimit();

    // Stator current limit
    public abstract double getStatorCurrentLimit();

    // Peak torque currents
    public abstract double getPeakForwardTorqueCurrent();
    public abstract double getPeakReverseTorqueCurrent();

    // Neutral mode
    public abstract boolean isNeutralModeBrake();
}

package frc.robot.constants.claw;

public abstract class ClawConfigBase {
    // CAN ID
    public abstract int getCanID();

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

    // filtering
    public abstract double getHighPassFilterTimeConstant();
    public abstract double getHighPassFilterUpperTrip();
    public abstract double getHighPassFilterLowerTrip();

    public abstract boolean getIsMotorInverted();


}

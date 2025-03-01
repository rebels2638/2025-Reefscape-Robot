package frc.robot.constants.roller;

import com.ctre.phoenix6.signals.UpdateModeValue;

public abstract class RollerConfigBase {
    // CAN ID
    public abstract int getRollerMotorCanID();
    public abstract int getCanRangeCanID();

    // Supply current limits
    public abstract double getSupplyCurrentLimit();
    public abstract double getSupplyCurrentLimitLowerTime();
    public abstract double getSupplyCurrentLimitLowerLimit();

    // Stator current limit
    public abstract double getStatorCurrentLimit();

    // Peak torque currents
    public abstract double getPeakForwardTorqueCurrent();
    public abstract double getPeakReverseTorqueCurrent();

    public abstract boolean isNeutralModeBrake();
    public abstract boolean isInverted();

    // CANrange configuration fields
    public abstract double getFOVCenterX();
    public abstract double getFOVCenterY();
    public abstract double getFOVRangeX();
    public abstract double getFOVRangeY();

    public abstract double getMinSignalStrengthForValidMeasurement();
    public abstract double getProximityHysteresis();
    public abstract double getProximityThreshold();

    public abstract double getToFUpdateFrequency();
    public abstract UpdateModeValue getToFUpdateMode();
}

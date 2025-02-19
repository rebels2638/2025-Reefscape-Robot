package frc.robot.constants.claw;

import com.ctre.phoenix6.signals.UpdateModeValue;

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

    public abstract int getCanRangeCanID();

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

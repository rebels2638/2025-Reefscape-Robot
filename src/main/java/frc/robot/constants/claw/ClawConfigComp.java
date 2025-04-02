package frc.robot.constants.claw;

import com.ctre.phoenix6.signals.UpdateModeValue;

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
    public int getMotorCanID() {
        return 18;
    }

    @Override
    public int getCanRangeCanID() {
        return 21;
    }

    // Supply current limits
    @Override
    public double getSupplyCurrentLimit() {
        return 30.0;
    }

    @Override
    public double getSupplyCurrentLimitLowerTime() {
        return 1.2;
    }

    @Override
    public double getSupplyCurrentLimitLowerLimit() {
        return 27;
    }

    // Stator current limit
    @Override
    public double getStatorCurrentLimit() {
        return 8.5;
    }

    // Peak torque currents
    @Override
    public double getPeakForwardTorqueCurrent() {
        return 8.5;
    }

    @Override
    public double getPeakReverseTorqueCurrent() {
        return -8.5;
    }

    // Neutral mode
    @Override
    public boolean isNeutralModeBrake() {
        return true;
    }

    @Override
    public boolean isInverted() {
        return false;
    }

    // CANrange configuration fields
    @Override
    public double getFOVCenterX() {
        return 0;
    }

    @Override
    public double getFOVCenterY() {
        return 0;
    }

    @Override
    public double getFOVRangeX() {
        return 6.75;
    }

    @Override
    public double getFOVRangeY() {
        return 6.75;
    }

    @Override
    public double getMinSignalStrengthForValidMeasurement() {
        return 1900;
    }

    @Override
    public double getProximityHysteresis() {
        return 0.005;
    }

    @Override
    public double getProximityThreshold() {
        return 0.13;
    }

    @Override
    public double getToFUpdateFrequency() {
        return 50;
    }

    @Override
    public UpdateModeValue getToFUpdateMode() {
        return UpdateModeValue.ShortRange100Hz;
    }
}

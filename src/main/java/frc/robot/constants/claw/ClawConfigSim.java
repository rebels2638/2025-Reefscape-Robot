package frc.robot.constants.claw;

import com.ctre.phoenix6.signals.UpdateModeValue;

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
    public int getMotorCanID() {
        return 1;
    }

    @Override
    public int getCanRangeCanID() {
        return 18;
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
        return 2000;
    }

    @Override
    public double getProximityHysteresis() {
        return 0.01;
    }

    @Override
    public double getProximityThreshold() {
        return 0.10;
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

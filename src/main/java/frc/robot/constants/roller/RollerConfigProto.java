package frc.robot.constants.roller;

import com.ctre.phoenix6.signals.UpdateModeValue;

public class RollerConfigProto extends RollerConfigBase {
    private static RollerConfigProto instance;

    public static RollerConfigProto getInstance() {
        if (instance == null) {
            instance = new RollerConfigProto();
        }
        return instance;
    }

    private RollerConfigProto() {}

    // CAN ID
    @Override
    public int getRollerMotorCanID() {
        return 1;
    }

    @Override
    public int getCanRangeCanID() {
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
        return 2500;
    }

    @Override
    public double getProximityHysteresis() {
        return 0.005;
    }

    @Override
    public double getProximityThreshold() {
        return 0.03;
    }

    @Override
    public double getToFUpdateFrequency() {
        return 100;
    }

    @Override
    public UpdateModeValue getToFUpdateMode() {
        return UpdateModeValue.ShortRange100Hz;
    }
}

package frc.robot.constants.roller;

import com.ctre.phoenix6.signals.UpdateModeValue;

public class RollerConfigComp extends RollerConfigBase {
    private static RollerConfigComp instance;

    public static RollerConfigComp getInstance() {
        if (instance == null) {
            instance = new RollerConfigComp();
        }
        return instance;
    }

    private RollerConfigComp() {}

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
        return 1;
    }

    @Override
    public double getFOVCenterY() {
        return 1;
    }

    @Override
    public double getFOVRangeX() {
        return 1;
    }

    @Override
    public double getFOVRangeY() {
        return 1;
    }

    @Override
    public double getMinSignalStrengthForValidMeasurement() {
        return 1;
    }

    @Override
    public double getProximityHysteresis() {
        return 1;
    }

    @Override
    public double getProximityThreshold() {
        return 1;
    }

    @Override
    public double getToFUpdateFrequency() {
        return 1;
    }

    @Override
    public UpdateModeValue getToFUpdateMode() {
        return UpdateModeValue.ShortRangeUserFreq;
    }
}

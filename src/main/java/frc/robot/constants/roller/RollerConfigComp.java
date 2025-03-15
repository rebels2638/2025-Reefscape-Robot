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
        return 16;
    }

    @Override
    public int getCanRangeCanID() {
        return 17;
    }
    
    // Supply current limits
    @Override
    public double getSupplyCurrentLimit() {
        return 50.0;
    }

    @Override
    public double getSupplyCurrentLimitLowerTime() {
        return 1.5;
    }

    @Override
    public double getSupplyCurrentLimitLowerLimit() {
        return 39.0;
    }

    // Stator current limit
    @Override
    public double getStatorCurrentLimit() {
        return 45.0;
    }

    // Peak torque currents
    @Override
    public double getPeakForwardTorqueCurrent() {
        return 45.0;
    }

    @Override
    public double getPeakReverseTorqueCurrent() {
        return -45.0;
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

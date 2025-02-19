package frc.robot.constants.claw;

import com.ctre.phoenix6.signals.UpdateModeValue;

public class ClawConfigProto extends ClawConfigBase {
    private static ClawConfigProto instance;
    public static ClawConfigProto getInstance() {
        if (instance == null) {
            instance = new ClawConfigProto();
        }

        return instance;
    }

    private ClawConfigProto() {}
    
    // CANID
    @Override
    public int getCanID() {
        return 14;
    }

    // Supply current limits
    @Override
    public double getSupplyCurrentLimit() {
        return 15.0;
    }

    @Override
    public double getSupplyCurrentLimitLowerTime() {
        return 1.2;
    }

    @Override
    public double getSupplyCurrentLimitLowerLimit() {
        return 5;
    }

    // Stator current limit
    @Override
    public double getStatorCurrentLimit() {
        return 10;
    }

    // Peak torque currents
    @Override
    public double getPeakForwardTorqueCurrent() {
        return 10;
    }

    @Override
    public double getPeakReverseTorqueCurrent() {
        return -10;
    }

    // Neutral mode
    @Override
    public boolean isNeutralModeBrake() {
        return true;
    }

    @Override
    public double getHighPassFilterTimeConstant() {
        return 0.1;
    }

    @Override
    public double getHighPassFilterUpperTrip() {
        return 10;
    }

    @Override
    public double getHighPassFilterLowerTrip() {
        return 10;
    }

    @Override
    public boolean getIsMotorInverted() {
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
   public int getCanRangeCanID() {
       return 17;
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

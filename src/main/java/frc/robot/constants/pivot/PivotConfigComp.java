package frc.robot.constants.pivot;

import edu.wpi.first.math.geometry.Rotation2d;

public class PivotConfigComp extends PivotConfigBase {
    private static PivotConfigComp instance;
    public static PivotConfigComp getInstance() {
        if (instance == null) {
            instance = new PivotConfigComp();
        }

        return instance;
    }
    
    private PivotConfigComp() {}
    
    // CANID
    @Override
    public int getCanID() {
        return 20;
    }

    // Supply current limits
    @Override
    public double getSupplyCurrentLimit() {
        return 35.0;
    }

    @Override
    public double getSupplyCurrentLimitLowerTime() {
        return 1.0;
    }

    @Override
    public double getSupplyCurrentLimitLowerLimit() {
        return 30.0;
    }

    // Stator current limit
    @Override
    public double getStatorCurrentLimit() {
        return 25.0;
    }

    // Peak torque currents
    @Override
    public double getPeakForwardTorqueCurrent() {
        return 25.0;
    }

    @Override
    public double getPeakReverseTorqueCurrent() {
        return -25.0;
    }

    // Characterization / Gains
    @Override
    public double getKS() {
        return 0;
    }

    @Override
    public double getKV() {
        return 0;
    }

    @Override
    public double getKA() {
        return 0.0;
    }

    @Override
    public double getKP() {
        return 450;
    }

    @Override
    public double getKI() {
        return 0.0;
    }

    @Override
    public double getKD() {
        return 50;
    }

    @Override
    public double getKG() {
        return 0.0; 
    }

    // Motion magic parameters
    @Override
    public double getMotionMagicExpoKA() {
        return 8;
    }

    @Override
    public double getMotionMagicExpoKV() {
        return 11;
    }

    @Override
    public double getMotionMagicCruiseVelocityRotationsPerSec() {
        return 3.0;
    }

    @Override
    public double getToleranceDegrees() {
        return 4.0;
    }
    
    // Neutral mode
    @Override
    public boolean isNeutralModeBrake() {
        return true;
    }

    @Override
    public boolean isInverted() {
        return true;
    }


    // Gear ratio
    @Override
    public double getMotorToOutputShaftRatio() {
        return 45.0;
    }

    @Override
    public double getMaxAngleRotations() {
        return 132.0/360.0;
    }

    @Override 
    public double getMinAngleRotations() {
        return -62.0/360.0;
    }

    @Override 
    public double getStartingAngleRotations() {
        return 132.0/360.0;
    } 
}

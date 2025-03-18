package frc.robot.constants.climber;

import edu.wpi.first.math.geometry.Rotation2d;
public class ClimberConfigComp extends ClimberConfigBase {
    private static ClimberConfigComp instance;
    public static ClimberConfigComp getInstance() {
        if (instance == null) {
            instance = new ClimberConfigComp();
        }

        return instance;
    }
    
    private ClimberConfigComp() {}
    
    // CANID
    @Override
    public int getCanID() {
        return 19;
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
        return 20.0;
    }

    // Peak torque currents
    @Override
    public double getPeakForwardTorqueCurrent() {
        return 20.0;
    }

    @Override
    public double getPeakReverseTorqueCurrent() {
        return -20.0;
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
        return 20000;
    }

    @Override
    public double getKI() {
        return 0.0;
    }

    @Override
    public double getKD() {
        return 220;
    }

    @Override
    public double getKG() {
        return 0.0; 
    }

    // Motion magic parameters
    @Override
    public double getMotionMagicExpoKA() {
        return 12;
    }

    @Override
    public double getMotionMagicExpoKV() {
        return 11;
    }

    @Override
    public double getMotionMagicCruiseVelocityRotationsPerSec() {
        return 0.25;
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
        return false;
    }


    // Gear ratio
    @Override
    public double getMotorToOutputShaftRatio() {
        return 1500;
    }

    @Override
    public double getMaxAngleRotations() {
        return 220/360.0;
    }

    @Override 
    public double getMinAngleRotations() {
        return -80.00/360.0;
    }

    @Override 
    public double getStartingAngleRotations() {
        return getMaxAngleRotations();
    } 
}

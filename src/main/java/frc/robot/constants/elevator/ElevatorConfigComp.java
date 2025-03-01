package frc.robot.constants.elevator;

public class ElevatorConfigComp extends ElevatorConfigBase {
    private static ElevatorConfigComp instance;
    public static ElevatorConfigComp getInstance() {
        if (instance == null) {
            instance = new ElevatorConfigComp();
        }

        return instance;
    }
    
    private ElevatorConfigComp() {}

    // CANID
    @Override
    public int getCanID1() {
        return 14;
    }

    @Override
    public int getCanID2() {
        return 15;
    }

    // Supply current limits
    @Override
    public double getSupplyCurrentLimit() {
        return 80;
    }

    @Override
    public double getSupplyCurrentLimitLowerTime() {
        return 1.2;
    }

    @Override
    public double getSupplyCurrentLimitLowerLimit() {
        return 60;
    }

    // Stator current limit
    @Override
    public double getStatorCurrentLimit() {
        return 65.0;
    }

    // Peak torque currents
    @Override
    public double getPeakForwardTorqueCurrent() {
        return 65.0;
    }

    @Override
    public double getPeakReverseTorqueCurrent() {
        return -65.0;
    }

    // Characterization / Gains
    @Override
    public double getKS() {
        return 4.3;
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
        return 1700;
    }

    @Override
    public double getKI() {
        return 0.0;
    }

    @Override
    public double getKD() {
        return 35;
    }

    @Override
    public double getKG() {
        return 12.2; 

    }

    // Motion magic parameters
    @Override
    public double getMotionMagicExpoKA() {
        return 7;
    }

    @Override
    public double getMotionMagicExpoKV() {
        return 10.0;
    }

    @Override
    public double getMotionMagicCruiseVelocityMetersPerSec() {
        return 0.45;
    }

    // Neutral mode
    @Override
    public boolean isNeutralModeBrake() {
        return true;
    }

    @Override
    public boolean isM1Inverted() {
        return false;
    }

    @Override
    public boolean isM2Inverted() {
        return false;
    }

    // Gear ratio
    @Override
    public double getMotorToOutputShaftRatio() {
        // rotations / meters  
        return 22 / 0.66;
    }

    @Override
    public double getMaxHeightMeters() {
        return 1.39;
    }

    @Override 
    public double getMinHeightMeters() {
        return 0.00;
    }

    @Override
    public double getToleranceMeters() {
        return 0.01;
    }

    @Override
    public double getToleranceMetersPerSec() {
        return 0.03;
    }
}

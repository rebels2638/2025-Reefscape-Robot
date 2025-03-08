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
        return 100;
    }

    @Override
    public double getSupplyCurrentLimitLowerTime() {
        return 1.2;
    }

    @Override
    public double getSupplyCurrentLimitLowerLimit() {
        return 90;
    }

    // Stator current limit
    @Override
    public double getStatorCurrentLimit() {
        return 90;
    }

    // Peak torque currents
    @Override
    public double getPeakForwardTorqueCurrent() {
        return 90.0;
    }

    @Override
    public double getPeakReverseTorqueCurrent() {
        return -90.0;
    }

    // Characterization / Gains
    @Override
    public double getKS() {
        return 2; // 
    }

    @Override
    public double getKV() {
        return 0.048; //6
    }

    @Override
    public double getKA() {
        return 1.6; //2

    }

    @Override
    public double getKP() {
        return 2000;//30/2000
    }

    @Override
    public double getKI() {
        return 50;

    }

    @Override
    public double getKD() {
        return 15;//30/15
    }

    @Override
    public double getKG() {
        return 30; 

    }

    // Motion magic parameters
    @Override
    public double getMotionMagicExpoKA() {
        return 1.5;
    }

    @Override
    public double getMotionMagicExpoKV() {
        return 3.5;
    }

    @Override
    public double getMotionMagicCruiseVelocityMetersPerSec() {
        return 5;
    }

    @Override
    public double getMotionMagicJerkMetersPerSecSecSec() {
        return 75;
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
        return 15 / 0.57;
    }

    @Override
    public double getMaxHeightMeters() {
        return 1.42;
    }

    @Override 
    public double getMinHeightMeters() {
        return 0.00;
    }

    @Override
    public double getToleranceMeters() {
        return 0.025;
    }

    @Override
    public double getToleranceMetersPerSec() {
        return 0.03;
    }
}

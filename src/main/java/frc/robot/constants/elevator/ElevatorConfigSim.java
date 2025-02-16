package frc.robot.constants.elevator;

public class ElevatorConfigSim extends ElevatorConfigBase {
    private static ElevatorConfigSim instance;
    public static ElevatorConfigSim getInstance() {
        if (instance == null) {
            instance = new ElevatorConfigSim();
        }

        return instance;
    }

    private ElevatorConfigSim() {}

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
        return 100;
    }

    @Override
    public double getKI() {
        return 0.0;
    }

    @Override
    public double getKD() {
        return 0;
    }

    @Override
    public double getKG() {
        return 0.0; 

    }

    // Motion magic parameters
    @Override
    public double getMotionMagicExpoKA() {
        return 5;
    }

    @Override
    public double getMotionMagicExpoKV() {
        return 10.0;
    }

    @Override
    public double getMotionMagicCruiseVelocityMetersPerSec() {
        return 3;
    }

    // Neutral mode
    @Override
    public boolean isNeutralModeBrake() {
        return false;
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
        return 5;
    }

    @Override
    public double getMaxHeightMeters() {
        return 1.45;
    }

    @Override 
    public double getMinHeightMeters() {
        return -0.002;
    }

    @Override
    public double getToleranceMeters() {
        return 0.02;
    }
}
 
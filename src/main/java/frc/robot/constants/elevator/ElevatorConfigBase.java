package frc.robot.constants.elevator;

public abstract class ElevatorConfigBase {
    // CAN ID
    public abstract int getCanID1();
    public abstract int getCanID2();

    // Supply current limits
    public abstract double getSupplyCurrentLimit();
    public abstract double getSupplyCurrentLimitLowerTime();
    public abstract double getSupplyCurrentLimitLowerLimit();

    // Stator current limit
    public abstract double getStatorCurrentLimit();

    // Peak torque currents
    public abstract double getPeakForwardTorqueCurrent();
    public abstract double getPeakReverseTorqueCurrent();

    // Characterization / Gains
    public abstract double getKS();
    public abstract double getKV();
    public abstract double getKA();
    public abstract double getKP();
    public abstract double getKI();
    public abstract double getKD();
    public abstract double getKG();

    // Motion magic parameters
    public abstract double getMotionMagicExpoKA();
    public abstract double getMotionMagicExpoKV();
    public abstract double getMotionMagicCruiseVelocityMetersPerSec();
    public abstract double getMotionMagicJerkMetersPerSecSecSec();

    // Neutral mode
    public abstract boolean isNeutralModeBrake();

    public abstract boolean isM1Inverted();
    public abstract boolean isM2Inverted();

    // Gear ratio
    public abstract double getMotorToOutputShaftRatio();

    public abstract double getMaxHeightMeters();
    public abstract double getMinHeightMeters();

    public abstract double getToleranceMeters();
    public abstract double getToleranceMetersPerSec();
}

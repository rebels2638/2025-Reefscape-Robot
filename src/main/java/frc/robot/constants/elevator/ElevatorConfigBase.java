package frc.robot.constants.elevator;

public abstract class ElevatorConfigBase {
    // CAN ID
    public abstract int getCANID1();
    public abstract int getCANID2();

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
    public abstract double getMotionMagicCruiseVelocityRotationsPerSec();

    // Neutral mode
    public abstract boolean isNeutralModeBrake();

    // Gear ratio
    public abstract double getMotorToOutputShaftRatio();
}

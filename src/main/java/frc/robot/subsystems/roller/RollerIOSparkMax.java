package frc.robot.subsystems.roller;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
public class RollerIOSparkMax implements RollerIO{
    
    private static final double kMOTOR_TO_OUTPUT_RATIO = 1/5.0;
    private final SparkMax m_motor = new SparkMax(13, MotorType.kBrushless);

    public RollerIOSparkMax() {
    }

    @Override
    public void updateInputs(RollerIOInputs inputs) {
        inputs.velocityRadSec = m_motor.getEncoder().getVelocity() / 60.0 * kMOTOR_TO_OUTPUT_RATIO * Math.PI * 2;
        inputs.voltage = m_motor.getAppliedOutput()*12;
        // System.out.println("LIMSWITCH: "+!limSwitch.get());
    }

    @Override
    public void setVoltage(double v) {
        m_motor.setVoltage(v);
    }

}

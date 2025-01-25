package frc.robot.subsystems.roller;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
public class RollerIOSparkMax implements RollerIO{
    
    private static final double kMOTOR_TO_OUTPUT_RATIO = 1/5.0;
    private final TalonFX m_motor = new TalonFX(0);
    private final DigitalInput lb = new DigitalInput(5);

    public RollerIOSparkMax() {
    }

    @Override
    public void updateInputs(RollerIOInputs inputs) {
        inputs.velocityRadSec = m_motor.getVelocity().getValueAsDouble() * kMOTOR_TO_OUTPUT_RATIO * Math.PI * 2;
        inputs.voltage = m_motor.getDutyCycle().getValueAsDouble()*12;
        inputs.inRoller = isInRoller();

        System.out.println("LB STATUS: " + lb.get());
        
    }

    @Override
    public void setVoltage(double v) {
        m_motor.setVoltage(v);
    }

    @Override
    public boolean isInRoller() {
        return lb.get() ? true : false;
    }

}

package frc.robot.subsystems.funnel;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class FunnelIODoubleSolenoid implements FunnelIO {
    private final DoubleSolenoid solenoidLeft;
    private final DoubleSolenoid solenoidRight;

    private boolean isPulled = false; 

    public FunnelIODoubleSolenoid() {
        this.solenoidLeft = new DoubleSolenoid(PneumaticsModuleType.REVPH,0, 1);
        this.solenoidRight = new DoubleSolenoid(PneumaticsModuleType.REVPH,0, 1);

    }
    @Override
    public void updateInputs(FunnelIOInputs inputs) {
        inputs.isPulled = isPulled;
    }

    @Override
    public void pull() {
        solenoidLeft.set(DoubleSolenoid.Value.kForward);
        solenoidRight.set(DoubleSolenoid.Value.kForward);

        isPulled = true;
    }


}
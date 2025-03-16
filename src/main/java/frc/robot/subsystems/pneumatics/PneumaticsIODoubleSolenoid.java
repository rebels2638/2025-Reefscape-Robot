package frc.robot.subsystems.pneumatics;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.subsystems.pneumatics.PneumaticsIO.PneumaticsIOInputs;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class PneumaticsIODoubleSolenoid implements PneumaticsIO {
    private final DoubleSolenoid solenoidFunnel;
    private final DoubleSolenoid solenoidRatchet;
    private final Compressor compressor;

    private boolean isFunnelPulled = false; 
    private boolean isRatchetPulled = false; 

    public PneumaticsIODoubleSolenoid() {
        this.solenoidFunnel = new DoubleSolenoid(PneumaticsModuleType.REVPH,14,15);
        this.solenoidRatchet = new DoubleSolenoid(PneumaticsModuleType.REVPH,8, 9);

        this.compressor = new Compressor(PneumaticsModuleType.REVPH);
        compressor.enableDigital();
    }
    @Override
    public void updateInputs(PneumaticsIOInputs inputs) {
        inputs.isFunnelPulled = isFunnelPulled;
        inputs.isRatchetPulled = isRatchetPulled;

        inputs.isCompressredEnabled = compressor.isEnabled();
        inputs.pressure = compressor.getPressure();
    }

    @Override
    public void pullFunnel() {
        solenoidFunnel.set(Value.kForward);
        isFunnelPulled = true;
    }

    @Override
    public void pushFunnel() {
        solenoidFunnel.set(Value.kReverse);
        isFunnelPulled = false;
    }

    @Override
    public void pullRatchet() {
        solenoidRatchet.set(Value.kForward);
        isRatchetPulled = true;
    }

    @Override
    public void pushRatchet() {
        solenoidRatchet.set(Value.kReverse);
        isRatchetPulled = false;
    }
}
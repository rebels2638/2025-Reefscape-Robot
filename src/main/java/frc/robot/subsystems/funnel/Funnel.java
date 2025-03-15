package frc.robot.subsystems.funnel;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.*;

public class Funnel extends SubsystemBase {
    private static Funnel instance = null;
    public static Funnel getInstance() {
        if (instance == null) {
            instance = new Funnel();
        }

        return instance;
    }

    private FunnelIO funnelIO;
    private FunnelIOInputsAutoLogged funnelIOInputs = new FunnelIOInputsAutoLogged();

    private Funnel() {
        // IO
        switch (Constants.currentMode) {
            case COMP:
                funnelIO = new FunnelIODoubleSolenoid();
                break;

            case PROTO:
                funnelIO = new FunnelIODoubleSolenoid();
                break;

            case SIM:
                funnelIO = new FunnelIO() {};
                break;

            case REPLAY:
                funnelIO = new FunnelIODoubleSolenoid();
                break;
            
            default:
                funnelIO = new FunnelIODoubleSolenoid();
                break;
        }
    }

    @Override
    public void periodic() {
        funnelIO.updateInputs(funnelIOInputs);
        Logger.processInputs("funnel", funnelIOInputs);
    }

    public void pull() {
        funnelIO.pull();
    }
}

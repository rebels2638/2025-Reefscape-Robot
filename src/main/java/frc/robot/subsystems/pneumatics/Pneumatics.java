package frc.robot.subsystems.pneumatics;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.*;

public class Pneumatics extends SubsystemBase {
    private static Pneumatics instance = null;
    public static Pneumatics getInstance() {
        if (instance == null) {
            instance = new Pneumatics();
        }

        return instance;
    }

    private PneumaticsIO pneumaticsIO;
    private PneumaticsIOInputsAutoLogged pneumaticsIOInputs = new PneumaticsIOInputsAutoLogged();

    private Pneumatics() {
        // IO
        switch (Constants.currentMode) {
            case COMP:
                pneumaticsIO = new PneumaticsIODoubleSolenoid();
                break;

            case PROTO:
                pneumaticsIO = new PneumaticsIODoubleSolenoid();
                break;

            case SIM:
                pneumaticsIO = new PneumaticsIO() {};
                break;

            case REPLAY:
                pneumaticsIO = new PneumaticsIODoubleSolenoid();
                break;
            
            default:
                pneumaticsIO = new PneumaticsIODoubleSolenoid();
                break;
        }
    }

    @Override
    public void periodic() {
        pneumaticsIO.updateInputs(pneumaticsIOInputs);
        Logger.processInputs("Pneumatics", pneumaticsIOInputs);
    }

    public void pullFunnel() {
        pneumaticsIO.pullFunnel();
    }   

    public void pushFunnel() {
        pneumaticsIO.pushFunnel();
    }

    public void pullRatchet() {
        pneumaticsIO.pullRatchet();
    }   

    public void pushRatchet() {
        pneumaticsIO.pushRatchet();
    }

    public boolean getRatchetState() {
        return pneumaticsIOInputs.isRatchetPulled;
    }
}

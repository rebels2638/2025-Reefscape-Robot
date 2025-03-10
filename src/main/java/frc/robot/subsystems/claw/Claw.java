package frc.robot.subsystems.claw;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.*;
import frc.robot.constants.claw.ClawConfigBase;
import frc.robot.constants.claw.ClawConfigComp;
import frc.robot.constants.claw.ClawConfigProto;
import frc.robot.constants.claw.ClawConfigSim;

public class Claw extends SubsystemBase {
    private ClawIO clawIO;
    private ClawIOInputsAutoLogged clawIOInputs = new ClawIOInputsAutoLogged();

    private final ClawConfigBase config;

    private static Claw instance = null;
    public static Claw getInstance() {
        if (instance == null) {
            instance = new Claw();
        }

        return instance;
    }
    

    public Claw() {
        // IO
        switch (Constants.currentMode) {
            case COMP:
                config = ClawConfigComp.getInstance();
                clawIO = new ClawIOTalonFX(config);

                break;

            case PROTO:
                config = ClawConfigProto.getInstance();
                clawIO = new ClawIOTalonFX(config);

                break;

            case SIM:
                config = ClawConfigSim.getInstance();
                // clawIO = new ClawIOSim(config);

                break;

            default:
                config = ClawConfigProto.getInstance();
                clawIO = new ClawIO() {};

                break;
        }
    }

    @Override
    public void periodic() {
        clawIO.updateInputs(clawIOInputs);
        Logger.processInputs("Claw", clawIOInputs);
    }

    public double getVelocityRadPerSec() {
        return clawIOInputs.clawVelocityRadPerSec;
    }

    public void setTorqueCurrentFOC(double current) {
        clawIO.setTorqueCurrentFOC(current);
    }

    public void setVoltage(double voltage) {
        clawIO.setVoltage(voltage);
    }

    public boolean inClaw() {
        return clawIOInputs.inClaw;
    }
}

package frc.robot.subsystems.claw;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.*;
import frc.robot.constants.claw.ClawConfigBase;
import frc.robot.constants.claw.ClawConfigProto;
import frc.robot.constants.claw.ClawConfigSim;

public class Claw extends SubsystemBase {
    private ClawIO clawIO;
    private ClawIOInputsAutoLogged clawIOInputs = new ClawIOInputsAutoLogged();

    private final ClawConfigBase config;

    public Claw() {
        // IO
        switch (Constants.currentMode) {
            case Comp:
                config = ClawConfigProto.getInstance();
                clawIO = new ClawIOTalonFX(config);

                break;

            case CrescendoRobotBase:
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

    public void setTorqueCurrentFOC(double current) {
        clawIO.setTorqueCurrentFOC(current);
    }
}

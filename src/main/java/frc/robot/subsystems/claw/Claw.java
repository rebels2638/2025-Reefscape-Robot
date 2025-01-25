package frc.robot.subsystems.claw;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.*;
import frc.robot.constants.pivot.PivotConfigBase;
import frc.robot.constants.pivot.PivotConfigProto;
import frc.robot.constants.pivot.PivotConfigSim;

public class Claw extends SubsystemBase {
    private ClawIO clawIO;
    private ClawIOInputsAutoLogged clawIOInputs = new ClawIOInputsAutoLogged();

    private final PivotConfigBase config;

    public Claw() {
        // IO
        switch (Constants.currentMode) {
            case Comp:
                config = PivotConfigProto.getInstance();
                clawIO = new ClawIOTalonFX(config);

                break;

            case CrescendoRobotBase:
                config = PivotConfigProto.getInstance();
                clawIO = new ClawIOTalonFX(config);

                break;

            case SIM:
                config = PivotConfigSim.getInstance();
                clawIO = new ClawIOSim(config);

                break;

            default:
                config = PivotConfigProto.getInstance();
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

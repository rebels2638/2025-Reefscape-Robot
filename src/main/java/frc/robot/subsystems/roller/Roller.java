package frc.robot.subsystems.roller;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.*;
import frc.robot.constants.roller.RollerConfigBase;
import frc.robot.constants.roller.RollerConfigComp;
import frc.robot.constants.roller.RollerConfigProto;
import frc.robot.constants.roller.RollerConfigSim;

public class Roller extends SubsystemBase {
    private RollerIO rollerIO;
    private RollerIOInputsAutoLogged rollerIOInputs = new RollerIOInputsAutoLogged();

    private final RollerConfigBase config;

    public Roller() {
        // IO
        switch (Constants.currentMode) {
            case COMP:
                config = RollerConfigComp.getInstance();
                rollerIO = new RollerIOTalonFX(config);

                break;

            case PROTO:
                config = RollerConfigProto.getInstance();
                rollerIO = new RollerIOSparkMax(config);

                break;

            case SIM:
                config = RollerConfigSim.getInstance();
                // rollerIO = new RollerIOSim(config);

                break;

            case REPLAY:
                config = RollerConfigProto.getInstance();
                rollerIO = new RollerIO() {};

                break;
            
            default:
                config = RollerConfigComp.getInstance();
                rollerIO = new RollerIOTalonFX(config);

                break;
        }
    }

    @Override
    public void periodic() {
        rollerIO.updateInputs(rollerIOInputs);
        Logger.processInputs("Roller", rollerIOInputs);
    }

    public void setTorqueCurrentFOC(double current) {
        rollerIO.setTorqueCurrentFOC(current);
    }

    public void setVoltage(double voltage) {
        rollerIO.setVoltage(voltage);
    }
}

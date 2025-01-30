package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.*;
import frc.robot.constants.elevator.ElevatorConfigBase;
import frc.robot.constants.elevator.ElevatorConfigComp;
import frc.robot.constants.elevator.ElevatorConfigProto;
import frc.robot.constants.elevator.ElevatorConfigSim;

public class Elevator extends SubsystemBase {
    private ElevatorIO elevatorIO;
    private ElevatorIOInputsAutoLogged elevatorIOInputs = new ElevatorIOInputsAutoLogged();

    private final ElevatorConfigBase config;

    public Elevator() {
        // IO
        switch (Constants.currentMode) {
            case COMP:
                config = ElevatorConfigComp.getInstance();
                elevatorIO = new ElevatorIOTalonFX(config);

                break;

            case PROTO:
                config = ElevatorConfigProto.getInstance();
                elevatorIO = new ElevatorIOTalonFX(config);

                break;

            case SIM:
                config = ElevatorConfigSim.getInstance();
                // pivotIO = new ElevatorIOSim(config);

                break;

            case REPLAY:
                config = ElevatorConfigComp.getInstance();
                elevatorIO = new ElevatorIO() {};

                break;
                
            default:
                config = ElevatorConfigComp.getInstance();
                elevatorIO = new ElevatorIO() {};

                break;
        }
    }

    @Override
    public void periodic() {
        elevatorIO.updateInputs(elevatorIOInputs);
        Logger.processInputs("Elevator", elevatorIOInputs);
    }

    public void setHeight(double height) {
        elevatorIO.setHeight(height);
    }

    public void setTorqueCurrentFOC(double torque) {
        elevatorIO.setTorqueCurrentFOC(torque);
    }

    public void setVoltage(double voltage) {
        elevatorIO.setVoltage(voltage);
    }
}

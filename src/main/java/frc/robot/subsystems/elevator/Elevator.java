package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.*;
import frc.robot.constants.elevator.ElevatorConfigBase;
import frc.robot.constants.elevator.ElevatorConfigProto;
import frc.robot.constants.elevator.ElevatorConfigSim;
import frc.robot.constants.pivot.PivotConfigBase;
import frc.robot.constants.pivot.PivotConfigProto;
import frc.robot.constants.pivot.PivotConfigSim;

public class Elevator extends SubsystemBase {
    private ElevatorIO pivotIO;
    private ElevatorIOInputsAutoLogged elevatorIOInputs = new ElevatorIOInputsAutoLogged();

    private final ElevatorConfigBase config;

    public Elevator() {
        // IO
        switch (Constants.currentMode) {
            case Comp:
                config = ElevatorConfigProto.getInstance();
                pivotIO = new ElevatorIOTalonFX(config);

                break;

            case CrescendoRobotBase:
                config = ElevatorConfigProto.getInstance();
                pivotIO = new ElevatorIOTalonFX(config);

                break;

            case SIM:
                config = ElevatorConfigSim.getInstance();
                // pivotIO = new ElevatorIOSim(config);

                break;

            default:
                config = ElevatorConfigProto.getInstance();
                pivotIO = new ElevatorIO() {};

                break;
        }
    }

    @Override
    public void periodic() {
        pivotIO.updateInputs(elevatorIOInputs);
        Logger.processInputs("Pivot", elevatorIOInputs);
    }

    public void setHeight(double height) {
        pivotIO.setHeight(height);
    }

    public void setTorqueCurrentFOC(double torque) {
        pivotIO.setTorqueCurrentFOC(torque);
    }
}

package frc.robot.subsystems.elevator;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.*;
import frc.robot.constants.elevator.ElevatorConfigBase;
import frc.robot.constants.elevator.ElevatorConfigComp;
import frc.robot.constants.elevator.ElevatorConfigProto;
import frc.robot.constants.elevator.ElevatorConfigSim;

public class Elevator extends SubsystemBase {

    public static enum height {
        IDLE,
        L1,
        L2,
        L3,
        L4
    };
    private height heightRequest = height.IDLE;
    private List<Double> extensionHeights = Arrays.asList(0.0, 0.1, 0.34, 0.77, 1.38);

    private static Elevator instance = null;
    public static Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator();
        }
        return instance;
    }

    private double setpoint = 0;

    private ElevatorIO elevatorIO;
    private ElevatorIOInputsAutoLogged elevatorIOInputs = new ElevatorIOInputsAutoLogged();

    private final ElevatorConfigBase config;

    private Elevator() {
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
                elevatorIO = new ElevatorIOSim(config);

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
        setpoint = extensionHeights.get(Arrays.asList(height.values()).indexOf(heightRequest));

        elevatorIO.setHeight(setpoint);

        Logger.recordOutput("Elevator/setpoint", setpoint);
    }

    public void requestLevel(int level) {
        switch (level) {
            case 0:
                this.heightRequest = height.IDLE;
                break;
            case 1:
                this.heightRequest = height.L1;
                break;
            case 2:
                this.heightRequest = height.L3;
                break;
            case 3:
                this.heightRequest = height.L4;
                break;
        }
    }
    
    public void setTorqueCurrentFOC(double torque) {
        elevatorIO.setTorqueCurrentFOC(torque);
    }

    public void setVoltage(double voltage) {
        elevatorIO.setVoltage(voltage);
    }

    public boolean reachedSetpoint() {
        return Math.abs(setpoint - elevatorIOInputs.elevatorHeightMeters) <= config.getToleranceMeters();
    }

    public double getHeight() {
        return elevatorIOInputs.elevatorHeightMeters;
    }
}

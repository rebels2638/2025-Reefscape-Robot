package frc.robot.subsystems.elevator;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.IntFunction;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.*;
import frc.robot.constants.elevator.ElevatorConfigBase;
import frc.robot.constants.elevator.ElevatorConfigComp;
import frc.robot.constants.elevator.ElevatorConfigProto;
import frc.robot.constants.elevator.ElevatorConfigSim;

public class Elevator extends SubsystemBase {
    private static Elevator instance = null;
    public static Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator();
        }
        return instance;
    }

    public static enum height {
        STOW,
        L1,
        L2,
        L3,
        L4
    };
    
    private height currHeightRequest = height.STOW;
    private height lastHeightRequest = currHeightRequest;
    private List<Double> extensionHeights = Arrays.asList(0.0, 0.1, 0.34, 0.77, 1.38);

    private double setpoint = 0;
    private boolean setpointModifiable = false;

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
        setpoint = extensionHeights.get(Arrays.asList(height.values()).indexOf(currHeightRequest));

        if (setpointModifiable) {
            if (currHeightRequest == height.STOW) {setpoint = extensionHeights.get(Arrays.asList(height.values()).indexOf(lastHeightRequest));}
            elevatorIO.setHeight(setpoint);
        }

        Logger.recordOutput("Elevator/setpoint", setpoint);
        Logger.recordOutput("Elevator/setPointQueued", 
            !(setpointModifiable || 
                (MathUtil.isNear(setpoint, elevatorIOInputs.elevatorVelocityMetersPerSec, 0.2) || 
                    !MathUtil.isNear(elevatorIOInputs.elevatorVelocityMetersPerSec, 0, 0.3)
                )
            )
        );
    }

    public void requestLevel(height level) {
        if (level != height.STOW) {this.lastHeightRequest = level;}
        this.currHeightRequest = level;
    }
    
    public void setSetpointSettable(boolean settable) {
        this.setpointModifiable = settable;
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

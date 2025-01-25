package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Elevator extends SubsystemBase {

    private static Elevator instance = null;
    private static ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    
    private double setpointMeters;
    private boolean reachedSetpoint;

    public Elevator(ElevatorIO io) {
        switch (Constants.currentMode) {
            case SIM:
                io = new ElevatorIOSim();
            default:
                io = new ElevatorIOFalcon();
        }

        this.setpointMeters = 0.0;
        this.reachedSetpoint = false;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
        io.setPosition(setpointMeters);
        Logger.recordOutput("Elevator/desiredHeight", setpointMeters);
    }

    public void setHeight(double height) {
        this.setpointMeters = height;
    }

    public void zeroHeight() {
        io.zeroHeight();
    }

    public boolean reachedSetpoint() {
        return inputs.reachedSetpoint;
    }

    public static Elevator getInstance() {
        if (instance == null) {
            return new Elevator(Elevator.io);
        }
        return instance;
    }

    public static Elevator setInstance(Elevator inst) {
        instance = inst;
        return inst;
    }

}

package frc.robot.subsystems.elevator;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.IntFunction;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.*;
import frc.robot.constants.elevator.ElevatorConfigBase;
import frc.robot.constants.elevator.ElevatorConfigComp;
import frc.robot.constants.elevator.ElevatorConfigProto;
import frc.robot.constants.elevator.ElevatorConfigSim;
import frc.robot.lib.util.LoggedTunableNumber;

public class Elevator extends SubsystemBase {
    private static Elevator instance = null;
    public static Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator();
        }
        return instance;
    }
    private static LoggedTunableNumber L1_height = new LoggedTunableNumber("Elevator/L1", 0.0);
    private static LoggedTunableNumber L2_height = new LoggedTunableNumber("Elevator/L2", 0.1);
    private static LoggedTunableNumber L3_height = new LoggedTunableNumber("Elevator/L3", 0.77);
    private static LoggedTunableNumber L4_height = new LoggedTunableNumber("Elevator/L4", 1.42);
    private static double[] tuningReferences = {0.0,0.1,0.34,0.77,1.42};

    public enum Height {
        STOW(0.01),
        L1(0.1),
        L2(0.34),
        L3(0.77),
        L4(1.42);
    
        private final double extensionHeight;
    
        Height(double extensionHeight) {
            this.extensionHeight = extensionHeight;
        }
    
        public double getExtensionHeight() {
            return extensionHeight;
        }
    }
    
    private Height currHeightRequest = Height.STOW;

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

        CommandScheduler.getInstance().registerSubsystem(this);
    }

    @Override
    public void periodic() {
        elevatorIO.updateInputs(elevatorIOInputs);
        Logger.processInputs("Elevator", elevatorIOInputs);

        if (Constants.isInTuningMode) {
            if (L1_height.hasChanged(hashCode())) {tuningReferences[1] = L1_height.getAsDouble();}
            if (L2_height.hasChanged(hashCode())) {tuningReferences[2] = L2_height.getAsDouble();}
            if (L3_height.hasChanged(hashCode())) {tuningReferences[3] = L3_height.getAsDouble();}
            if (L4_height.hasChanged(hashCode())) {tuningReferences[4] = L4_height.getAsDouble();}
            setpoint = tuningReferences[Arrays.asList(Height.values()).indexOf(currHeightRequest)];
        }

        else {
            setpoint = currHeightRequest.getExtensionHeight();
        }

        // setpoint = currHeightRequest.getExtensionHeight();

        if (setpointModifiable) {elevatorIO.setHeight(setpoint);}

        Logger.recordOutput("Elevator/setpoint", setpoint);
        Logger.recordOutput("Elevator/setpointModifiable", setpointModifiable);
        Logger.recordOutput("Elevator/setPointQueued", 
            !(setpointModifiable || 
                (MathUtil.isNear(setpoint, elevatorIOInputs.elevatorVelocityMetersPerSec, 0.2) || 
                    !MathUtil.isNear(elevatorIOInputs.elevatorHeightMeters, 0, 0.02)
                )
            )
        );
        Logger.recordOutput("Elevator/CurrentCommand", this.getCurrentCommand() == null ? "" : this.getCurrentCommand().toString());
    }

    public void requestLevel(Height level) {
        if (!setpointModifiable) {
            this.currHeightRequest = level;
        }
    }

    public Height getRequestedLevel() {
        return currHeightRequest;
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
        return 
            MathUtil.isNear(setpoint, elevatorIOInputs.elevatorHeightMeters, config.getToleranceMeters()) &&
            MathUtil.isNear(elevatorIOInputs.elevatorVelocityMetersPerSec, 0, config.getToleranceMetersPerSec());

    }

    public double getHeight() {
        return elevatorIOInputs.elevatorHeightMeters;
    }
}

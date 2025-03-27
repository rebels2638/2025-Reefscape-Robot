package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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

    public enum Height {
        STOW(0.035),
        L1(0.1),
        L2(0.37),
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

        setpoint = currHeightRequest.getExtensionHeight();

        // if (Climber.getInstance().getSetpoint().getDegrees() < 180) {
        //     elevatorIO.setHeight(Height.L2.getExtensionHeight());
        // }
        if (setpointModifiable) {
            elevatorIO.setHeight(setpoint);
        }

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

        Logger.recordOutput("Elevator/reachesSetpoint", reachedSetpoint());

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

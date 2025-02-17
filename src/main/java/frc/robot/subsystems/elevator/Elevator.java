package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;
import org.opencv.highgui.HighGui;

import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
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

    private ElevatorIO elevatorIO;
    private ElevatorIOInputsAutoLogged elevatorIOInputs = new ElevatorIOInputsAutoLogged();

    private final ElevatorConfigBase config;

    private double setpoint = 0;

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

        elevatorIO.setHeight(setpoint);

        Logger.recordOutput("Elevator/setpoint", setpoint);
    }

    public void setHeight(double height) {
        setpoint = height;
    }

    public Constants.level getElevatorHeight() {
        return MathUtil.isNear(0.1, elevatorIOInputs.elevatorHeightMeters, 0.01) ? Constants.level.L1 : 
            MathUtil.isNear(0.34, elevatorIOInputs.elevatorHeightMeters, 0.03)  ? Constants.level.L2 :
                MathUtil.isNear(0.77, elevatorIOInputs.elevatorHeightMeters, 0.03) ? Constants.level.L3 :
                    MathUtil.isNear(1.38, elevatorIOInputs.elevatorHeightMeters, 0.03) ? Constants.level.L3 : Constants.level.IDLE;
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
}

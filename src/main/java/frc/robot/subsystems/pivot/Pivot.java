package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.*;
import frc.robot.constants.pivot.PivotConfigBase;
import frc.robot.constants.pivot.PivotConfigComp;
import frc.robot.constants.pivot.PivotConfigProto;
import frc.robot.constants.pivot.PivotConfigSim;

public class Pivot extends SubsystemBase {
    private static Pivot instance = null;
    public static Pivot getInstance() {
        if (instance == null) {
            instance = new Pivot();
        }

        return instance;
    }

    private PivotIO pivotIO;
    private PivotIOInputsAutoLogged pivotIOInputs = new PivotIOInputsAutoLogged();

    private Rotation2d setpoint = new Rotation2d();

    private final PivotConfigBase config;

    private Pivot() {
        // IO
        switch (Constants.currentMode) {
            case COMP:
                config = PivotConfigComp.getInstance();
                pivotIO = new PivotIOTalonFX(config);

                break;

            case PROTO:
                config = PivotConfigProto.getInstance();
                pivotIO = new PivotIOTalonFX(config);

                break;

            case SIM:
                config = PivotConfigSim.getInstance();
                pivotIO = new PivotIOSim(config);

                break;

            case REPLAY:
                config = PivotConfigComp.getInstance();
                pivotIO = new PivotIO() {};

                break;

            default:
                config = PivotConfigComp.getInstance();
                pivotIO = new PivotIOTalonFX(config);

                break;
        }
    }

    @Override
    public void periodic() {
        pivotIO.updateInputs(pivotIOInputs);
        Logger.processInputs("Pivot", pivotIOInputs);

        pivotIO.setAngle(setpoint);
    }

    public void setAngle(Rotation2d angle) {
        setpoint = angle;
        Logger.recordOutput("Pivot/setpoint", angle);
    }

    public void setTorqueCurrentFOC(double torque) {
        pivotIO.setTorqueCurrentFOC(torque);
    }

    public void setVoltage(double voltage) {
        pivotIO.setVoltage(voltage);
    }

    public boolean reachedSetpoint() {
        return Math.abs(setpoint.minus(pivotIOInputs.pivotPosition).getDegrees()) <= config.getToleranceDegrees();
    }

    public Rotation2d getAngle() {
        return pivotIOInputs.pivotPosition;
    }
}

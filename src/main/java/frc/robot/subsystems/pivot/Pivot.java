package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Torque;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.*;
import frc.robot.constants.pivot.PivotConfigBase;
import frc.robot.constants.pivot.PivotConfigProto;
import frc.robot.constants.pivot.PivotConfigSim;

public class Pivot extends SubsystemBase {
    private PivotIO pivotIO;
    private PivotIOInputsAutoLogged pivotIOInputs = new PivotIOInputsAutoLogged();

    private final PivotConfigBase config;

    public Pivot() {
        // IO
        switch (Constants.currentMode) {
            case Comp:
                config = PivotConfigProto.getInstance();
                pivotIO = new PivotIOTalonFX(config);

                break;

            case CrescendoRobotBase:
                config = PivotConfigProto.getInstance();
                pivotIO = new PivotIOTalonFX(config);

                break;

            case SIM:
                config = PivotConfigSim.getInstance();
                pivotIO = new PivotIOSim(config);

                break;

            default:
                config = PivotConfigProto.getInstance();
                pivotIO = new PivotIO() {};

                break;
        }
    }

    @Override
    public void periodic() {
        pivotIO.updateInputs(pivotIOInputs);
        Logger.processInputs("Pivot", pivotIOInputs);
    }

    public void setAngle(Rotation2d angle) {
        pivotIO.setAngle(angle);
    }

    public void setTorqueCurrentFOC(double torque) {
        pivotIO.setTorqueCurrentFOC(torque);
    }

    public void setVoltage(double voltage) {
        pivotIO.setVoltage(voltage);
    }
}

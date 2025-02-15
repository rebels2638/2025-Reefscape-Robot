package frc.robot.subsystems.roller;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.*;
import frc.robot.constants.roller.RollerConfigBase;
import frc.robot.constants.roller.RollerConfigComp;
import frc.robot.constants.roller.RollerConfigProto;
import frc.robot.constants.roller.RollerConfigSim;
import frc.robot.lib.util.Elastic;
import edu.wpi.first.wpilibj.DriverStation;

public class Roller extends SubsystemBase {
    private static Roller instance = null;
    public static Roller getInstance() {
        if (instance == null) {
            instance = new Roller();
        }

        return instance;
    }

    private RollerIO rollerIO;
    private RollerIOInputsAutoLogged rollerIOInputs = new RollerIOInputsAutoLogged();

    private final Elastic.Notification disconnectAlert = new Elastic.Notification(Elastic.Notification.NotificationLevel.ERROR,
                                "Error Notification", "CANRange Disconnected, Roller may not be functioning");

    private final RollerConfigBase config;

    private Roller() {
        if (!isConnected()) {
            Elastic.sendNotification(disconnectAlert.withDisplayMilliseconds(5000));
            DriverStation.reportError("Roller CANRange Disconnected", true);
        }

        // IO
        switch (Constants.currentMode) {
            case COMP:
                config = RollerConfigComp.getInstance();
                rollerIO = new RollerIOTalonFX(config);

                break;

            case PROTO:
                config = RollerConfigProto.getInstance();
                rollerIO = new RollerIOSparkMax(config);

                break;

            case SIM:
                config = RollerConfigSim.getInstance();
                rollerIO = new RollerIOSim();

                break;

            case REPLAY:
                config = RollerConfigProto.getInstance();
                rollerIO = new RollerIO() {};

                break;
            
            default:
                config = RollerConfigComp.getInstance();
                rollerIO = new RollerIOTalonFX(config);

                break;
        }
    }

    @Override
    public void periodic() {
        rollerIO.updateInputs(rollerIOInputs);
        Logger.processInputs("Roller", rollerIOInputs);
    }

    public void setTorqueCurrentFOC(double current) {
        rollerIO.setTorqueCurrentFOC(current);
    }

    public void setVoltage(double voltage) {
        rollerIO.setVoltage(voltage);
    }

    public boolean inRoller() {
        return rollerIOInputs.inRoller;
    }

    public boolean isConnected() {
        return rollerIOInputs.isConnected;
    }
}

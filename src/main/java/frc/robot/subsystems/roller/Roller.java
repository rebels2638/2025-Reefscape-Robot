package frc.robot.subsystems.roller;

import java.util.Map;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.RobotState.ScoringObservation;
import frc.robot.constants.Constants;
import frc.robot.constants.roller.RollerConfigBase;
import frc.robot.constants.roller.RollerConfigComp;
import frc.robot.constants.roller.RollerConfigProto;
import frc.robot.constants.roller.RollerConfigSim;
import edu.wpi.first.wpilibj.Timer;

public class Roller extends SubsystemBase {
    private static Roller instance = null;
    public static Roller getInstance() {
        if (instance == null) {
            instance = new Roller();
        }

        return instance;
    }

    public TimeDiscreteBooleanBuffer status = new TimeDiscreteBooleanBuffer(1);

    private RollerIO rollerIO;
    private RollerIOInputsAutoLogged rollerIOInputs = new RollerIOInputsAutoLogged();

    private final RollerConfigBase config;

    private Roller() {
        // IO
        switch (Constants.currentMode) {
            case COMP:
                config = RollerConfigComp.getInstance();
                rollerIO = new RollerIOTalonFX(config);

                break;

            case PROTO:
                config = RollerConfigProto.getInstance();
                rollerIO = new RollerIOTalonFX(config);

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
        status.addValue(rollerIOInputs.inRoller);
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

    public void isScored(Constants.scoredPositions pos, Constants.level level) {
        double preWindow = 0.0;
        double postWindow = 0.0;

        if (status.size() > 2 && pos != null) {

            double currentTime = Timer.getFPGATimestamp();
        
            // Define time bounds for windows
            double preStart = currentTime - preWindow - postWindow;
            double preEnd = currentTime - postWindow;
            double postStart = currentTime - postWindow;

            // Count how many true/false values are in the respective windows
            int preTrueCount = 0, postFalseCount = 0;

            for (Map.Entry<Double, Boolean> entry : status.getEntrySet()) {
                double time = entry.getKey();
                boolean value = entry.getValue();

                if (time >= preStart && time <= preEnd && value) {
                    preTrueCount++;
                }
                if (time >= postStart && !value) {
                    postFalseCount++;
                }
            }

            // Define thresholds for detection (e.g., 80% of samples should match the criteria)
            double preThreshold = 0.8 * status.getSampleCount(preStart, preEnd);
            double postThreshold = 0.8 * status.getSampleCount(postStart, currentTime);

            if (preTrueCount >= preThreshold && postFalseCount >= postThreshold ) {
                RobotState.getInstance().addScoringObservation(new ScoringObservation(pos, level, Constants.GamePiece.CORAL, currentTime));
            }

        }
    }
}

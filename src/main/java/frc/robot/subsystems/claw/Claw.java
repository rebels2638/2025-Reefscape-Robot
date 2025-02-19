package frc.robot.subsystems.claw;

import java.util.Map;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.RobotState.ScoringObservation;
import frc.robot.constants.*;
import frc.robot.constants.claw.ClawConfigBase;
import frc.robot.constants.claw.ClawConfigComp;
import frc.robot.constants.claw.ClawConfigProto;
import frc.robot.constants.claw.ClawConfigSim;
import frc.robot.subsystems.roller.TimeDiscreteBooleanBuffer;

public class Claw extends SubsystemBase {
    private ClawIO clawIO;
    private ClawIOInputsAutoLogged clawIOInputs = new ClawIOInputsAutoLogged();

    public TimeDiscreteBooleanBuffer status = new TimeDiscreteBooleanBuffer(1);

    private final ClawConfigBase config;

    private static Claw instance = null;
    public static Claw getInstance() {
        if (instance == null) {
            instance = new Claw();
        }

        return instance;
    }
    

    public Claw() {
        // IO
        switch (Constants.currentMode) {
            case COMP:
                config = ClawConfigComp.getInstance();
                clawIO = new ClawIOTalonFX(config);

                break;

            case PROTO:
                config = ClawConfigProto.getInstance();
                clawIO = new ClawIOTalonFX(config);

                break;

            case SIM:
                config = ClawConfigSim.getInstance();
                // clawIO = new ClawIOSim(config);

                break;

            default:
                config = ClawConfigProto.getInstance();
                clawIO = new ClawIO() {};

                break;
        }
    }

    @Override
    public void periodic() {
        clawIO.updateInputs(clawIOInputs);
        Logger.processInputs("Claw", clawIOInputs);
        status.addValue(clawIOInputs.inClaw);
    }

    public double getVelocityRadPerSec() {
        return clawIOInputs.clawVelocityRadPerSec;
    }

    public void setTorqueCurrentFOC(double current) {
        clawIO.setTorqueCurrentFOC(current);
    }

    public void setVoltage(double voltage) {
        clawIO.setVoltage(voltage);
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

            if (preTrueCount >= preThreshold && postFalseCount >= postThreshold) {
                RobotState.getInstance().addScoringObservation(new ScoringObservation(pos, level, Constants.GamePiece.ALGAY, currentTime));
            }

        }
    }
}

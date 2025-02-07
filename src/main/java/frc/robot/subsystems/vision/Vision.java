package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.RobotState.VisionObservation;
import frc.robot.constants.*;
import frc.robot.constants.vision.VisionConfigBase;
import frc.robot.constants.vision.VisionConfigComp;
import frc.robot.constants.vision.VisionConfigProto;
import frc.robot.constants.vision.VisionConfigSim;
import frc.robot.lib.util.LimelightHelpers;

public class Vision extends SubsystemBase {
    private static Vision instance = null;
    public static Vision getInstance() {
        if (instance == null) {
            return new Vision();
        }

        return instance;
    }

    private final VisionIO visionIO[];
    private final VisionIOInputsAutoLogged visionIOInputs[];

    private final RobotState robotState = RobotState.getInstance();

    private final TimeInterpolatableBuffer<Rotation2d> rotationalRateBuffer;

    private final VisionConfigBase config;
    private Vision() {
        // IO
        switch (Constants.currentMode) {
            case COMP:
                config = VisionConfigComp.getInstance();

                visionIO = new VisionIO[config.getNames().length];
                visionIOInputs = new VisionIOInputsAutoLogged[config.getNames().length];
                for (int i = 0; i < config.getNames().length; i++) {
                    visionIO[i] = new VisionIOLimelight(config.getNames()[i]);
                    visionIOInputs[i] = new VisionIOInputsAutoLogged();
                }

                break;

            case PROTO:
                config = VisionConfigProto.getInstance();

                visionIO = new VisionIO[config.getNames().length];
                visionIOInputs = new VisionIOInputsAutoLogged[config.getNames().length];
                for (int i = 0; i < config.getNames().length; i++) {
                    visionIO[i] = new VisionIOLimelight(config.getNames()[i]);
                    visionIOInputs[i] = new VisionIOInputsAutoLogged();
                }

                break;

            case SIM:
                config = VisionConfigSim.getInstance();

                visionIO = new VisionIO[config.getNames().length];
                visionIOInputs = new VisionIOInputsAutoLogged[config.getNames().length];
                for (int i = 0; i < config.getNames().length; i++) {
                    visionIO[i] = new VisionIO() {};
                    visionIOInputs[i] = new VisionIOInputsAutoLogged();
                }

                break;

            case REPLAY:
                config = VisionConfigComp.getInstance();

                visionIO = new VisionIO[config.getNames().length];
                visionIOInputs = new VisionIOInputsAutoLogged[config.getNames().length];
                for (int i = 0; i < config.getNames().length; i++) {
                    visionIO[i] = new VisionIO() {};
                    visionIOInputs[i] = new VisionIOInputsAutoLogged();
                }


                break;

            default:
                config = VisionConfigComp.getInstance();

                visionIO = new VisionIO[config.getNames().length];
                visionIOInputs = new VisionIOInputsAutoLogged[config.getNames().length];
                for (int i = 0; i < config.getNames().length; i++) {
                    visionIO[i] = new VisionIOLimelight(config.getNames()[i]);
                    visionIOInputs[i] = new VisionIOInputsAutoLogged();
                }

                break;
        }

        rotationalRateBuffer = TimeInterpolatableBuffer.createBuffer(config.getObservationBufferSizeSeconds());
    }

    @Override
    public void periodic() {
        for (int i = 0; i < config.getNames().length; i++) {
            // update LL helper BEFORE we use it in the io
            LimelightHelpers.SetRobotOrientation( 
                config.getNames()[i],
                robotState.getEstimatedPose().getRotation().getDegrees(),
                robotState.getGyroRates().getMeasureY().in(Degrees),

                robotState.getGyroOrientation().getMeasureZ().in(Degrees),
                robotState.getGyroRates().getMeasureZ().in(Degrees),

                robotState.getGyroOrientation().getMeasureX().in(Degrees),
                robotState.getGyroRates().getMeasureX().in(Degrees)
            );

            visionIO[i].updateInputs(visionIOInputs[i]);
            Logger.processInputs("Vision" + config.getNames()[i], visionIOInputs[i]);
            
            rotationalRateBuffer.addSample(Timer.getTimestamp(), robotState.getGyroRates().toRotation2d());

            Optional<Rotation2d> rotationalRate = rotationalRateBuffer.getSample(visionIOInputs[i].timestampSeconds);
            if (Timer.getTimestamp() - visionIOInputs[i].timestampSeconds <= config.getObservationBufferSizeSeconds() && 
                rotationalRate.isPresent() && 
                visionIOInputs[i].hasValidTargets) {
                
                Logger.recordOutput("vision/addingSample", true);
                Logger.recordOutput("vision/rotationalDevContribution", 
                Math.pow(rotationalRate.get().getDegrees() ,2) / config.getTranslationDevRotationExpoDenominator());
                Logger.recordOutput("vision/taDevContribution", visionIOInputs[i].ta * config.getTranslationDevTaScaler());
                
                double translationDev = 
                    config.getTranslationDevBase() +
                    Math.pow(rotationalRate.get().getDegrees() ,2)
                    / config.getTranslationDevRotationExpoDenominator() 
                    + (100 - visionIOInputs[i].ta) * config.getTranslationDevTaScaler();
        
                robotState.addVisionObservation(
                    new VisionObservation(
                        visionIOInputs[i].estimatedPose,
                        visionIOInputs[i].timestampSeconds,
                        VecBuilder.fill(
                            translationDev,
                            translationDev,
                            9999999
                        )
                    )
                );
            }
            else {
                Logger.recordOutput("vision/addingSample", false);
            }
        }
    }
}

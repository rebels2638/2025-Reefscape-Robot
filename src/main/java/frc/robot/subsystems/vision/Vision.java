package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;

import java.lang.annotation.Target;
import java.util.ArrayList;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.RobotState.VisionObservation;
import frc.robot.RobotState.VisionObservationScale;
import frc.robot.constants.*;
import frc.robot.constants.Constants.VisionConstants;
import frc.robot.constants.vision.VisionConfigBase;
import frc.robot.constants.vision.VisionConfigComp;
import frc.robot.constants.vision.VisionConfigProto;
import frc.robot.constants.vision.VisionConfigSim;
import frc.robot.lib.util.LimelightHelpers;
import frc.robot.lib.util.RebelUtil;

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

    private VisionObservationScale requestedScale = VisionObservationScale.GLOBAL;

    private int localTagID = 18;

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
                    visionIO[i] = new VisionIOSim();
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

        robotState.registerRunnableOnGlobalVisionEstimateRequest(this::requestGlobalEstimationScale);
        robotState.registerRunnableOnLocalVisionEstimateRequest(this::requestLocalEstimationScale);

        CommandScheduler.getInstance().registerSubsystem(this);

        requestGlobalEstimationScale();
    }

    @Override
    public void periodic() {
        for (int i = 0; i < config.getNames().length; i++) {
            visionIO[i].updateInputs(visionIOInputs[i]);
            Logger.processInputs("Vision" + config.getNames()[i], visionIOInputs[i]);
            
            rotationalRateBuffer.addSample(Timer.getTimestamp(), robotState.getGyroRates()[2]);

            Optional<Rotation2d> rotationalRate = rotationalRateBuffer.getSample(visionIOInputs[i].timestampSeconds);
            if (Timer.getTimestamp() - visionIOInputs[i].timestampSeconds <= config.getObservationBufferSizeSeconds() && 
                rotationalRate.isPresent() && 
                visionIOInputs[i].hasValidTargets &&
                !(visionIOInputs[i].scale == VisionObservationScale.LOCAL && Math.abs(rotationalRate.get().getDegrees()) > 20)
            ) {

                double taDev = 
                    RebelUtil.constrain(
                        -Math.sqrt(
                            RebelUtil.constrain(
                                ((visionIOInputs[i].ta - 0.07) / 35570),
                                0.0,
                                10000.0
                            )
                        ) + 0.03,
                        0.003,
                        0.03
                    );

                double translationDev = 
                        (Math.pow(Math.abs(rotationalRate.get().getRadians()) + 0.5, config.getTranslationDevRotationExpo())
                        / config.getTranslationDevRotationExpoDenominator()) + taDev;
        
                Logger.recordOutput("Vision/" + config.getNames()[i] + "totalDev", translationDev);
                Logger.recordOutput("Vision/" + config.getNames()[i] + "taDevContribution", taDev);

                Logger.recordOutput("Vision/" + config.getNames()[i] + "/addingSample", true);
                Logger.recordOutput("Vision/" + config.getNames()[i] + "/rotationalDevContribution", translationDev - taDev);
                
                robotState.addVisionObservation(
                    new VisionObservation(
                        visionIOInputs[i].estimatedPose,
                        visionIOInputs[i].timestampSeconds,
                        VecBuilder.fill(
                            translationDev,
                            translationDev,
                            9999999
                        ),
                        visionIOInputs[i].scale
                    )
                );
            }
            else {
                Logger.recordOutput("Vision/addingSample", false);
            }

            if (requestedScale == VisionObservationScale.LOCAL && localTagID == visionIOInputs[i].primaryTagId) {
                visionIO[i].includeTagIDs(Optional.of(new int[] {localTagID}));
            }
        }
    }

    public void requestLocalEstimationScale(Translation2d endRobotPose) {
        int closest = 0;
        for (int i = 1; i < VisionConstants.kREEF_TAG_POSES.length; i++) {
            if (VisionConstants.kREEF_TAG_POSES[i].getDistance(endRobotPose) < 
                VisionConstants.kREEF_TAG_POSES[closest].getDistance(endRobotPose)) {
                closest = i;
            }
        }

        int tagID = VisionConstants.kREEF_TAG_IDS[closest];
        
        localTagID = tagID;
        Logger.recordOutput("Vision/localTagID", localTagID);
        requestedScale = VisionObservationScale.LOCAL;
    }
    
    public void requestGlobalEstimationScale() {
        for (int i = 0; i < config.getNames().length; i++) {
            visionIO[i].includeTagIDs(Optional.empty());
        }

        requestedScale = VisionObservationScale.GLOBAL;
    }
}

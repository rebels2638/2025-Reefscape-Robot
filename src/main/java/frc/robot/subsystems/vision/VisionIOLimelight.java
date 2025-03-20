package frc.robot.subsystems.vision;

import java.util.Optional;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotState;
import frc.robot.RobotState.VisionObservationScale;
import frc.robot.constants.Constants;
import frc.robot.lib.util.LimelightHelpers;

public class VisionIOLimelight implements VisionIO {

    private final String name;
    private VisionObservationScale scale = VisionObservationScale.GLOBAL;

    public VisionIOLimelight(String name) {
        this.name = name;

        RobotState.getInstance().registerRunnableOnOdometryUpdate(
            () -> {
                // update limelight orientation as soon as possible
                LimelightHelpers.SetRobotOrientation( 
                    name,
                    RobotState.getInstance().getGyroOrientation()[2].getDegrees(),
                    RobotState.getInstance().getGyroRates()[2].getDegrees(),

                    RobotState.getInstance().getGyroOrientation()[1].getDegrees(),
                    RobotState.getInstance().getGyroRates()[1].getDegrees(),

                    RobotState.getInstance().getGyroOrientation()[0].getDegrees(),
                    RobotState.getInstance().getGyroRates()[0].getDegrees()
                );
            }
        );
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
        
        inputs.hasValidTargets = mt2 != null && mt2.tagCount > 0;
        inputs.estimatedPose = mt2 != null  ? mt2.pose : inputs.estimatedPose;
        inputs.timestampSeconds = mt2 != null ? mt2.timestampSeconds : inputs.timestampSeconds;

        inputs.primaryTagId = (int) NetworkTableInstance.getDefault().getTable(name).getEntry("tid").getDouble(0);
        inputs.tx = NetworkTableInstance.getDefault().getTable(name).getEntry("tx").getDouble(0);
        inputs.ty = NetworkTableInstance.getDefault().getTable(name).getEntry("ty").getDouble(0);
        inputs.ta = NetworkTableInstance.getDefault().getTable(name).getEntry("ta").getDouble(0);

        inputs.scale = scale;
    }

    @Override
    public void includeTagIDs(Optional<int[]> tags) {
        if (tags.isPresent()) {
            LimelightHelpers.SetFiducialIDFiltersOverride(name, tags.get());
            scale = VisionObservationScale.LOCAL;
        }
        else {
            LimelightHelpers.SetFiducialIDFiltersOverride(name, Constants.VisionConstants.kALL_TAG_IDS);
            scale = VisionObservationScale.GLOBAL;
        }
    }
}
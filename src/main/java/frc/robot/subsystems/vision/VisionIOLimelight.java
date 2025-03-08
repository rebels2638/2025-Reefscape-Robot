package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.lib.util.LimelightHelpers;

public class VisionIOLimelight implements VisionIO {

    private final String name;

    public VisionIOLimelight(String name) {
        this.name = name;
    }

    public void updateInputs(VisionIOInputs inputs) {
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
        
        inputs.hasValidTargets = mt2 != null && mt2.tagCount > 0;
        inputs.estimatedPose = mt2 != null  ? mt2.pose : inputs.estimatedPose;
        inputs.timestampSeconds = mt2 != null  ? Timer.getFPGATimestamp() - mt2.latency / 1000.0 : inputs.timestampSeconds;
        
        inputs.primaryTagId = (int) NetworkTableInstance.getDefault().getTable(name).getEntry("tid").getDouble(0);
        inputs.tx = NetworkTableInstance.getDefault().getTable(name).getEntry("tx").getDouble(0);
        inputs.ty = NetworkTableInstance.getDefault().getTable(name).getEntry("ty").getDouble(0);
        inputs.ta = NetworkTableInstance.getDefault().getTable(name).getEntry("ta").getDouble(0);
    }

}
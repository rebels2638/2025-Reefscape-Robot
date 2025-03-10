package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotState;
import frc.robot.lib.util.LimelightHelpers;

public class VisionIOLimelight implements VisionIO {

    private final String name;

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
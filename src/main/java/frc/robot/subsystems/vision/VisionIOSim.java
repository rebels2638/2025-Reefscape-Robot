package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotState;

public class VisionIOSim implements VisionIO {

    public VisionIOSim() {}

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.estimatedPose = RobotState.getInstance().getEstimatedPose();
        inputs.timestampSeconds = Timer.getFPGATimestamp();
    }
}
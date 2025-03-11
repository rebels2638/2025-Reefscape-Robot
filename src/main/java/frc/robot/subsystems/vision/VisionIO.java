package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.RobotState.VisionObservationScale;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public Pose2d estimatedPose = new Pose2d();
        public double timestampSeconds = 0;
        public boolean hasValidTargets = false;
        public int primaryTagId = 0;
        public double tx = 0;
        public double ty = 0;
        public double ta = 0;

        public VisionObservationScale scale = VisionObservationScale.GLOBAL;
    }

    public default void updateInputs(VisionIOInputs inputs) {}
    public default void includeTagIDs(Optional<int[]> inclusions) {}
}
package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.lib.util.AlignmentUtil;

public class ZeroPose extends Command {
    private final Supplier<Pose2d> pose;
    public ZeroPose(Supplier<Pose2d> pose) {
        this.pose = pose;
    }

    @Override
    public void initialize() {
        AlignmentUtil.loadCandidates();
        RobotState.getInstance().resetPose(pose.get());
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.lib.util.AlignmentUtil;

public class ZeroPose extends Command {
    private final Supplier<Pose2d> pose;
    private boolean ran = false;

    public ZeroPose(Supplier<Pose2d> pose) {
        this.pose = pose;
    }

    @Override
    public void initialize() {
        AlignmentUtil.loadCandidates();
        RobotState.getInstance().resetPose(pose.get());
        ran = false;
    }

    @Override
    public void execute() {
        if (!ran) {
            AlignmentUtil.loadCandidates();
            RobotState.getInstance().resetPose(pose.get());
            ran = true;
        }
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
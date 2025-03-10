package frc.robot.commands.autoAlignment;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.simple.QueueL4Action;
import frc.robot.lib.input.XboxController;
import frc.robot.lib.util.AlignmentUtil;

public class AlignToBargeAxisLocked extends SequentialCommandGroup{
    public AlignToBargeAxisLocked(XboxController controller) {
        Pose2d targetPose = AlignmentUtil.getClosestBargePose();
        addCommands(
            new QueueL4Action(),
            new LockDriveAxis(controller, AlignmentUtil.getBargeAxis(), targetPose.getRotation())
        );
    }
}
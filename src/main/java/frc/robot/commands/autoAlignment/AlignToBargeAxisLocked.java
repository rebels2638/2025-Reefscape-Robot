package frc.robot.commands.autoAlignment;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotState;
import frc.robot.commands.elevator.simple.QueueL4Action;
import frc.robot.lib.input.XboxController;
import frc.robot.lib.util.AlignmentUtil;
import frc.robot.subsystems.claw.Claw;

public class AlignToBargeAxisLocked extends SequentialCommandGroup{
    public AlignToBargeAxisLocked(XboxController controller) {
        addCommands(
            new WaitUntilCommand(
                () -> AlignmentUtil.getClosestBargePose().getTranslation().getDistance(
                    RobotState.getInstance().getEstimatedPose().getTranslation()) <= 2
            ),
            new QueueL4Action(),
            new LockDriveAxis(controller, AlignmentUtil.getBargeAxis(), AlignmentUtil.getClosestBargePose().getRotation())
        );
    }
}